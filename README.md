# collision avoidance VOP
project repo for collision avoidance VOP

## create environment
- ```cd``` into control or camera folder
- create venv: https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/, follow the commands under:
  - create venv
  - activate venv
  - update pip
- set venv as python interpreter in vsc (verify that the terminal auto opens in venv)
- download modules:
  ```bash
  pip install -r requirements.txt
  ```

## Camerasystem boat startup (objectdetectie + stereocamera)
- ```cd``` into camera folder
- from the project root, run:
  ```bash
python start_boat_mission.py --camera-left 0 --camera-right 1 --duration 60 --depth-calculation dual-camera
  ```
- testen zonder camera hardware (mock mode):
  ```bash
  python camera/start_boat_mission.py --mock-no-cameras --duration 10
  ```
- in mock mode blijft de timing/lus actief en bevat de log `"-"` placeholders voor cameradata.
- Dit start de recording van beide camera's en stopt ze in 2 mp4 files + schrijft 10 keer per seconde een detectielog weg
- output is stored under:
  - `camera/recordings/<timestamp>/camera0.mp4`
  - `camera/recordings/<timestamp>/camera1.mp4`
  - `camera/recordings/<timestamp>/detections.jsonl`
  - `camera/recordings/<timestamp>/mission.log`

example log line in `detections.jsonl`:
```json
{"timestamp_utc":"2026-03-11T12:00:00.100Z","elapsed_s":0.1,"detections":[{"camera":0,"label":"duck","confidence":0.91,"distance_m":5.2,"axis_offset_m":-0.4,"bbox_xyxy":[100.0,180.0,260.0,360.0]}]}
```

notes:
- the script uses `camera/calibration_yamls/calib_cam0.yaml` by default for focal length.
- label remapping defaults to `{\"bird\": \"duck\"}`.
- distance is a monocular estimate based on known object widths. tune with:
  ```bash
  --object-widths-json "{\"duck\":0.25,\"buoy\":0.30}"
  ```
- create `env.py` and fill in credentials
  ````python
  NTRIP_USER = "username"
  NTRIP_PASSWORD = "password"

obstacle Pi
```bash
sudo nmcli con show
sudo nmcli con mod "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.50.2/24 ipv4.gateway "" ipv4.dns "" ipv6.method ignore connection.interface-name eth0 connection.autoconnect yes
sudo nmcli con down "Wired connection 1"; sudo nmcli con up "Wired connection 1"
ip -4 addr show dev eth0
```

control Pi
```bash
sudo nmcli con show
sudo nmcli con mod "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.50.3/24 ipv4.gateway "" ipv4.dns "" ipv6.method ignore connection.interface-name eth0 connection.autoconnect yes
sudo nmcli con down "Wired connection 1"; sudo nmcli con up "Wired connection 1"
ip -4 addr show dev eth0
```