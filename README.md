# collision avoidance VOP
project repo for collision avoidance VOP

The code is divided into two subprojects: control & camera. Both are supposed to run on a dedicated Raspberry Pi. The control Pi has to be connected to the GPS over GPIO, and to the ESP32 over USB. The camera Pi has to be connected to an AI Hat+ and to the cameras. Both Pi's are connected over ethernet to each other.
The GPS has two antennas. The actual position is measured from the antenna plugged into port 1. Consider this when connecting the antennas.

This project has documentation split across multiple markdown files. See below for setup and operation references.

## Documentation

- **[Setup & Installation](docs/setup.md)**: Hardware instructions, Python dependencies, Raspberry Pi OS networking configurations (TCP), and setting up systemd services.
- **[Architecture & Data Flow](docs/architecture.md)**: The internal code structure, multi-threading overviews, and how the Camera Pi communicates with the Control Pi.
- **[Operation & Missions](docs/operation.md)**: Details on running interactive test dashboards, starting an autonomous vessel script, and starting camera missions (with or without hardware).
