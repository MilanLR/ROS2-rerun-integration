# ROS2-rerun-integration

Nodes to transform ROS2 data to be used in Rerun.

## Build the package

```bash
colcon build --symlink-install
```

## Source the setup
```bash
. install/setup.bash
```

## Run the node
```bash
ros2 run rerun_visualization rerun_node
```


# Depth camera setup
This installation guide worked for the Realsense D435, on a ubuntu 22.04 VM. It will probably also work for other realsense cameras, but is not tested.
Make sure to use a USB3 cable.

```bash
sudo apt install ros-humble-realsense2_camera*
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```
