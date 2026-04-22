# Display Dashboard

This folder contains the web dashboard for UGent Sailing.

## What it does

- Shows boat status and a live log.
- Sends control actions to a Python API on the Raspberry Pi.
- Polls the API for state updates every 2 seconds.

## Expected API endpoints

- `GET /api/status`
- `POST /api/recording/start`
- `POST /api/recording/stop`
- `POST /api/mission/start`
- `POST /api/mission/abort`

## How it connects

- If the page is served from the same Pi, it uses the current origin.
- If the page is opened as a local file, it tries `http://127.0.0.1:8000`.
- You can override the API URL by setting `window.UGENT_SAILING_API_BASE_URL` before loading `scripts/main.js`.

## Notes

- SSH is only for setup and maintenance on the Pi.
- The browser cannot talk SSH directly.
- The dashboard must talk HTTP to a small backend service on the Pi.
- That backend can start or stop the camera and control scripts as subprocesses or services.
