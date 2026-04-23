#!/usr/bin/env python3
"""Simple TCP obstacle protocol tester.

Use this script on either Pi to validate the Ethernet link and obstacle message framing.

Protocol under test:
- One newline-terminated line per update.
- Payload line format: "<angle_deg>,<distance_m>[;<angle_deg>,<distance_m>;...]"
"""

from __future__ import annotations

import argparse
import socket
import sys
import time


def parse_obstacle_line(line: str, value_sep: str = ",", pair_sep: str = ";") -> list[tuple[float, float]]:
    objects: list[tuple[float, float]] = []
    for raw_pair in line.split(pair_sep):
        pair = raw_pair.strip()
        if not pair:
            continue

        parts = [p.strip() for p in pair.split(value_sep)]
        if len(parts) != 2:
            continue

        try:
            angle = float(parts[0])
            distance = float(parts[1])
        except ValueError:
            continue

        objects.append((angle, distance))

    return objects


def format_obstacle_line(objects: list[tuple[float, float]], value_sep: str = ",", pair_sep: str = ";") -> str:
    return pair_sep.join(f"{a:.2f}{value_sep}{d:.2f}" for a, d in objects)


def generate_sweep_payload(index: int) -> list[tuple[float, float]]:
    center = -30.0 + (index % 13) * 5.0
    return [
        (center, 3.0),
        (center + 12.5, 4.2),
    ]


def run_server(args: argparse.Namespace) -> int:
    total_sent = 0

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.host, args.port))
        server.listen(max(1, args.backlog))
        server.settimeout(1.0)

        print(f"Server listening on {args.host}:{args.port}")

        while True:
            if args.count > 0 and total_sent >= args.count:
                print(f"Done: sent {total_sent} lines")
                return 0

            try:
                client, addr = server.accept()
            except socket.timeout:
                continue
            except KeyboardInterrupt:
                print("Stopped by user")
                return 0

            print(f"Client connected: {addr[0]}:{addr[1]}")
            with client:
                client.settimeout(max(0.05, args.send_timeout))
                i = 0
                while True:
                    if args.count > 0 and total_sent >= args.count:
                        print(f"Done: sent {total_sent} lines")
                        return 0

                    if args.payload:
                        line = args.payload
                    else:
                        line = format_obstacle_line(
                            generate_sweep_payload(i),
                            value_sep=args.value_sep,
                            pair_sep=args.pair_sep,
                        )
                    i += 1

                    wire = (line + "\n").encode("ascii", errors="ignore")
                    try:
                        client.sendall(wire)
                    except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                        print("Client disconnected")
                        break

                    total_sent += 1
                    print(f"TX #{total_sent}: {line}")
                    time.sleep(max(0.01, args.interval))


def run_client(args: argparse.Namespace) -> int:
    received = 0
    start = time.monotonic()
    deadline = None if args.duration <= 0 else start + args.duration

    try:
        with socket.create_connection((args.host, args.port), timeout=max(0.1, args.connect_timeout)) as sock:
            print(f"Connected to {args.host}:{args.port}")
            sock.settimeout(max(0.1, args.read_timeout))
            fp = sock.makefile("r", encoding="ascii", errors="ignore", newline="\n")

            while True:
                if deadline is not None and time.monotonic() >= deadline:
                    break

                try:
                    raw = fp.readline()
                except socket.timeout:
                    continue

                if not raw:
                    print("Server closed the connection")
                    break

                line = raw.strip()
                if not line:
                    continue

                received += 1
                parsed = parse_obstacle_line(
                    line,
                    value_sep=args.value_sep,
                    pair_sep=args.pair_sep,
                )

                if not parsed:
                    print(f"RX #{received}: INVALID: {line}")
                    continue

                pretty = ", ".join(f"(angle={a:.2f}, distance={d:.2f})" for a, d in parsed)
                print(f"RX #{received}: {line} -> {pretty}")

                if args.expected_lines > 0 and received >= args.expected_lines:
                    break
    except OSError as exc:
        print(f"Connection failed: {exc}")
        return 2

    print(f"Done: received {received} valid/non-empty lines")
    if args.expected_lines > 0 and received < args.expected_lines:
        print(f"Expected at least {args.expected_lines} lines")
        return 1
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="TCP obstacle protocol connectivity tester")
    sub = parser.add_subparsers(dest="mode", required=True)

    server = sub.add_parser("server", help="Run as obstacle TCP sender")
    server.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    server.add_argument("--port", type=int, default=9000, help="Bind port (default: 9000)")
    server.add_argument("--backlog", type=int, default=1, help="Listen backlog (default: 1)")
    server.add_argument("--interval", type=float, default=0.5, help="Send interval seconds (default: 0.5)")
    server.add_argument("--count", type=int, default=0, help="Total lines to send (0 = infinite)")
    server.add_argument("--payload", default="", help="Fixed payload to send every line")
    server.add_argument("--send-timeout", type=float, default=0.2, help="Socket send timeout seconds")
    server.add_argument("--value-sep", default=",", help="Value separator (default: ,)")
    server.add_argument("--pair-sep", default=";", help="Pair separator (default: ;)")
    server.set_defaults(func=run_server)

    client = sub.add_parser("client", help="Run as obstacle TCP receiver")
    client.add_argument("--host", required=True, help="Server host to connect to")
    client.add_argument("--port", type=int, default=9000, help="Server port (default: 9000)")
    client.add_argument("--connect-timeout", type=float, default=3.0, help="Connect timeout seconds")
    client.add_argument("--read-timeout", type=float, default=1.0, help="Read timeout seconds")
    client.add_argument("--duration", type=float, default=5.0, help="Run duration in seconds (<=0 for unlimited)")
    client.add_argument("--expected-lines", type=int, default=1, help="Minimum lines expected before success")
    client.add_argument("--value-sep", default=",", help="Value separator (default: ,)")
    client.add_argument("--pair-sep", default=";", help="Pair separator (default: ;)")
    client.set_defaults(func=run_client)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
