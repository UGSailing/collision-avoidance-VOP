# Setup and Installation
## Setting up the Control Pi
### Python Environment
- `cd` into the `control` folder
- Create a virtual environment by following [the official guide](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/):
  - create venv
  - activate venv
  - update pip
- Set the venv as the Python interpreter in your IDE (e.g., VS Code) and verify the terminal auto-opens in the venv.
- Install dependencies:
  ```bash
  pip install -r requirements.txt
  ```
- Create a `env.py` file in the source directory and fill in your credentials:
  ```python
  NTRIP_USER = "username"
  NTRIP_PASSWORD = "password"
  ```

### GPS Serial Connections
The GPS receiver connects over GPIO/serial. Enable serial connections by running:
```bash
sudo raspi-config
```
- Choose `3) Interface Options`
- Choose `6) Serial Port`
- Choose `no` (for login shell over serial)
- Choose `yes` (for hardware serial port enabled)

### Pi-Pi connection
see [below](#pi-pi-connection)

### Optional: autorun the script on boot
To start the control system automatically when the Raspberry Pi boots:

1. Copy the systemd service script:
   ```bash
   cp control/autostart_VOP.service /etc/systemd/system/autostart_VOP.service
   ```
2. Reload the daemon and enable the service:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable autostart_VOP.service
   sudo systemctl start autostart_VOP.service
   ```
3. Check the status:
   ```bash
   sudo systemctl status autostart_VOP.service
   ```

## Setting up the Camera Pi
### Python Environment
- `cd` into the `camera` folder
- Create a virtual environment by following [the official guide](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/):
  - create venv
  - activate venv
  - update pip
- Set the venv as the Python interpreter in your IDE (e.g., VS Code) and verify the terminal auto-opens in the venv.
- Install dependencies:
  ```bash
  pip install -r requirements.txt
  ```


### Pi-Pi connection
see [below](#pi-pi-connection)

### Optional: autorun the script on boot
To start the camerasystem automatically when the Raspberry Pi boots:

1. Copy the systemd service script:
   ```bash
   cp camera/autostart_VOP.service /etc/systemd/system/autostart_VOP.service
   ```
2. Reload the daemon and enable the service:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable autostart_VOP.service
   sudo systemctl start autostart_VOP.service
   ```
3. Check the status:
   ```bash
   sudo systemctl status autostart_VOP.service
   ```

## Pi-Pi Ethernet Connection
The Control Pi and Camera Pi communicate over a direct Ethernet TCP connection. Set up static IPs using `nmcli`.

Run the following to list network connections:
```bash
sudo nmcli con show
```
*Note: Replace `"netplan-eth0"` in the commands below with your actual eth0 connection name.*

#### Camera Pi (192.168.50.2)
```bash
sudo nmcli con mod "netplan-eth0" ipv4.method manual ipv4.addresses 192.168.50.2/24 ipv4.gateway "" ipv4.dns "" ipv6.method ignore connection.interface-name eth0 connection.autoconnect yes
sudo nmcli con down "netplan-eth0"
sudo nmcli con up "netplan-eth0"
ip -4 addr show dev eth0
```

#### Control Pi (192.168.50.3)
```bash
sudo nmcli con show
sudo nmcli con mod "netplan-eth0" ipv4.method manual ipv4.addresses 192.168.50.3/24 ipv4.gateway "" ipv4.dns "" ipv6.method ignore connection.interface-name eth0 connection.autoconnect yes
sudo nmcli con down "netplan-eth0"
sudo nmcli con up "netplan-eth0"
ip -4 addr show dev eth0
```