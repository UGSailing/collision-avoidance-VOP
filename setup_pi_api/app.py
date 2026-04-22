from __future__ import annotations

import os
import signal
import subprocess
import threading
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

import config


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


@dataclass
class ManagedProcess:
    name: str
    command: list[str]
    cwd: Path
    log_path: Path
    process: subprocess.Popen[str] | None = None
    _reader_thread: threading.Thread | None = field(
        default=None, init=False, repr=False
    )
    _tail: deque[str] = field(
        default_factory=lambda: deque(maxlen=config.LOG_TAIL_LINES),
        init=False,
        repr=False,
    )
    _lock: threading.Lock = field(
        default_factory=threading.Lock, init=False, repr=False
    )

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self) -> dict[str, Any]:
        with self._lock:
            if self.is_running():
                return self.snapshot()

            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            log_file = self.log_path.open("a", encoding="utf-8")
            process = subprocess.Popen(
                self.command,
                cwd=str(self.cwd),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            self.process = process
            self._reader_thread = threading.Thread(
                target=self._pump_output,
                args=(process, log_file),
                daemon=True,
            )
            self._reader_thread.start()
            return self.snapshot()

    def _pump_output(self, process: subprocess.Popen[str], log_file: Any) -> None:
        try:
            assert process.stdout is not None
            for line in process.stdout:
                text = line.rstrip("\n")
                if not text:
                    continue
                stamped = f"[{_now_iso()}] {text}"
                self._tail.append(stamped)
                log_file.write(stamped + "\n")
                log_file.flush()
        finally:
            log_file.close()

    def stop(self) -> dict[str, Any]:
        with self._lock:
            if not self.is_running():
                return self.snapshot()

            process = self.process
            assert process is not None
            try:
                if os.name != "nt":
                    os.killpg(process.pid, signal.SIGTERM)
                else:
                    process.terminate()
                process.wait(timeout=config.STOP_TIMEOUT_SECONDS)
            except Exception:
                try:
                    if os.name != "nt":
                        os.killpg(process.pid, signal.SIGKILL)
                    else:
                        process.kill()
                finally:
                    try:
                        process.wait(timeout=1.0)
                    except Exception:
                        pass

            self.process = None
            return self.snapshot()

    def snapshot(self) -> dict[str, Any]:
        running = self.is_running()
        pid = self.process.pid if running and self.process is not None else None
        return {
            "name": self.name,
            "running": running,
            "pid": pid,
            "command": self.command,
            "cwd": str(self.cwd),
            "log_path": str(self.log_path),
            "recent_logs": list(self._tail),
        }


@dataclass
class ApiState:
    last_action: str = "Dashboard online"
    message: str = "Waiting for mission command."
    updated_at: str = field(default_factory=lambda: _now_iso())

    def touch(self, last_action: str, message: str | None = None) -> None:
        self.last_action = last_action
        if message is not None:
            self.message = message
        self.updated_at = _now_iso()


app = FastAPI(title="UGent Sailing Pi API", version="0.1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

api_state = ApiState()
camera_proc = ManagedProcess(
    name="camera",
    command=config.CAMERA_COMMAND,
    cwd=config.CAMERA_CWD,
    log_path=config.LOG_DIR / "camera.log",
)
control_proc = ManagedProcess(
    name="control",
    command=config.CONTROL_COMMAND,
    cwd=config.CONTROL_CWD,
    log_path=config.LOG_DIR / "control.log",
)


def _status_payload() -> dict[str, Any]:
    camera_running = camera_proc.is_running()
    control_running = control_proc.is_running()
    return {
        "api_online": True,
        "recording": camera_running,
        "mission": control_running,
        "last_action": api_state.last_action,
        "message": api_state.message,
        "updated_at": api_state.updated_at,
        "camera": camera_proc.snapshot(),
        "control": control_proc.snapshot(),
    }


def _ensure_process_started(
    process: ManagedProcess, action_label: str, message: str
) -> dict[str, Any]:
    try:
        snapshot = process.start()
    except FileNotFoundError as exc:
        raise HTTPException(
            status_code=500, detail=f"Could not start {process.name}: {exc}"
        ) from exc
    except Exception as exc:
        raise HTTPException(
            status_code=500, detail=f"Could not start {process.name}: {exc}"
        ) from exc

    api_state.touch(action_label, message)
    return {
        "ok": True,
        **_status_payload(),
        process.name: snapshot,
    }


def _ensure_process_stopped(
    process: ManagedProcess, action_label: str, message: str
) -> dict[str, Any]:
    snapshot = process.stop()
    api_state.touch(action_label, message)
    return {
        "ok": True,
        **_status_payload(),
        process.name: snapshot,
    }


@app.get("/")
def root() -> dict[str, Any]:
    return {
        "name": "UGent Sailing Pi API",
        "status": "ok",
        "updated_at": api_state.updated_at,
    }


@app.get("/api/status")
def api_status() -> dict[str, Any]:
    return _status_payload()


@app.post("/api/recording/start")
def start_recording() -> dict[str, Any]:
    return _ensure_process_started(
        camera_proc,
        action_label="Start recording",
        message="Camera process is running.",
    )


@app.post("/api/recording/stop")
def stop_recording() -> dict[str, Any]:
    return _ensure_process_stopped(
        camera_proc,
        action_label="Stop recording",
        message="Camera process stopped.",
    )


@app.post("/api/mission/start")
def start_mission() -> dict[str, Any]:
    return _ensure_process_started(
        control_proc,
        action_label="Start mission",
        message="Control process is running.",
    )


@app.post("/api/mission/abort")
def abort_mission() -> dict[str, Any]:
    return _ensure_process_stopped(
        control_proc,
        action_label="Abort mission",
        message="Control process stopped.",
    )


@app.post("/api/all/stop")
def stop_all() -> dict[str, Any]:
    control_proc.stop()
    camera_proc.stop()
    api_state.touch("Stop all", "All processes stopped.")
    return {
        "ok": True,
        **_status_payload(),
    }
