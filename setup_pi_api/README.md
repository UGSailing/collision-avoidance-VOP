# Pi API

This folder contains the small Python API that sits on the Raspberry Pi and bridges the web dashboard to the boat scripts.

## What happens

- The browser sends HTTP requests to this API.
- The API starts or stops local Python processes for camera and control.
- The API returns JSON status so the dashboard can show live state.

## Endpoints

- `GET /api/status`
- `POST /api/recording/start`
- `POST /api/recording/stop`
- `POST /api/mission/start`
- `POST /api/mission/abort`
- `POST /api/all/stop`

## Install on the Pi

From the repository root:

```bash
cd setup_pi_api
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
uvicorn app:app --host 0.0.0.0 --port 8000
```

## How the communication works

- The website calls the Pi with `fetch()`.
- The Pi responds with JSON.
- The API stores recent process output in `setup_pi_api/logs/`.
- The dashboard polls `/api/status` every 2 seconds.

## Important

- SSH is for installation and maintenance.
- The website does not use SSH directly.
- If you want different camera or control commands, edit `config.py`.
- The default camera command is long-running so the dashboard can stop it manually.
