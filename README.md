# collision avoidance VOP
project repo for collision avoidance VOP

The code is divided into two subprojects: control & camera. Both are supposed to run on a dedicated Raspberry Pi. The control Pi has to be connected to the GPS over GPIO, and to the ESP32 over USB. The camera Pi has to be connected to an AI Hat+ and to the cameras. Both Pi's are connected over ethernet to each other.

## set up the control Pi
### Python
- ```cd``` into control or camera folder
- create venv: https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/, follow the commands under:
  - create venv
  - activate venv
  - update pip
- set venv as python interpreter in vsc (verify that the terminal auto opens in venv)
- download packages:
  ```bash
  pip install -r requirements.txt
  ```
- create `env.py` and fill in credentials
  ````python
  NTRIP_USER = "username"
  NTRIP_PASSWORD = "password"
  ````

### GPS
The GPS requires serial connections to be enabled. Enable this by running:
```bash
sudo raspi-config
```
- choose ```3) Interface Options```
- choose ```6) Serial Port```
- choose ```no```  
- choose ```yes```


### Pi-Pi connection
Run these commands to allow the code to set up the TCP connection between the Pi's.

Run:
```bash
sudo nmcli con show
```
Replace ```"netplan-eth0``` in the commands below with the name of the eth0 connection.
#### obstacle Pi
```bash
sudo nmcli con mod "netplan-eth0" ipv4.method manual ipv4.addresses 192.168.50.2/24 ipv4.gateway "" ipv4.dns "" ipv6.method ignore connection.interface-name eth0 connection.autoconnect yes
sudo nmcli con down "netplan-eth0"
sudo nmcli con up "netplan-eth0"
ip -4 addr show dev eth0
```

#### control Pi
```bash
sudo nmcli con show
sudo nmcli con mod "netplan-eth0" ipv4.method manual ipv4.addresses 192.168.50.3/24 ipv4.gateway "" ipv4.dns "" ipv6.method ignore connection.interface-name eth0 connection.autoconnect yes
sudo nmcli con down "netplan-eth0"
sudo nmcli con up "netplan-eth0"
ip -4 addr show dev eth0
```


## Camerasystem boat startup (objectdetectie + stereocamera)
TODO: translate to English

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

## set up autorun on boot
### Control Pi
Copy over the service script to systemd:
```bash
cp control/autostart_VOP.service /etc/systemd/system/autostart_VOP.service
```

Reload and enable the service:
```bash
sudo systemctl daemon-reload
sudo systemctl restart autostart_VOP.service
```

Get the status of the service:
```bash
sudo systemctl status autostart_VOP.service
```

### Camera Pi
TODO
