# collision avoidance VOP
project repo for collision avoidance VOP

## framework
ROS2: Robot Operating System 2 \
info: https://www.ros.org/ \
installation: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

## usage
activate ROS2 environment
```bash
source /opt/ros/jazzy/setup.bash
```

## build a package
build
```bash
colcon build --packages-select package_name
```
reload the new packages
```bash
source install/setup.bash
```

## run nodes
```bash
ros2 run package_name node_name
``` 

## listen to a topic
```bash
ros2 topic echo /topic_name
```