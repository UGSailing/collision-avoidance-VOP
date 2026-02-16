# collision avoidance VOP
project repo for collision avoidance VOP

## framework
ROS2: Robot Operating System 2 \
info: https://www.ros.org/ \
installation: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

## extra requirements
### extra packages
- nmea-navsat-driver (to read in location data)
- ntrip-client (for centimeter level precision)
### extra python modules
- python3-dotenv

## usage
### auto activate ROS2 environment
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### build a package
```bash
colcon build --packages-select package_name
source install/setup.bash # reload packages
```

### run a node
```bash
ros2 run package_name node_name
``` 

### listen to a topic
```bash
ros2 topic echo /topic_name
```