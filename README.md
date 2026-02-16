# collision avoidance VOP
project repo for collision avoidance VOP

## framework
ROS2: Robot Operating System 2 \
info: https://www.ros.org/ \
installation: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

## required packages
- nmea-navsat-driver (to read in location data)
- ntrip-client (for centimeter level precision)

## usage
### environment
activate ROS2 environment
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### build a package
```bash
colcon build --packages-select package_name
```
reload the new packages
```bash
source install/setup.bash
```

### run a node
```bash
ros2 run package_name node_name
``` 

### listen to a topic
```bash
ros2 topic echo /topic_name
```