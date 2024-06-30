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

# Mini Pupper 2 setup

#### First, flash Ubuntu for the raspberry pi 4 onto an micro SD card, see `https://ubuntu.com/download/raspberry-pi`

#### On the Pupper, install the board support package (bsp)

```bash
cd
git clone <https://github.com/mangdangroboticsclub/mini_pupper_2_bsp.git> mini_pupper_bsp
cd mini_pupper_bsp
./install.sh
reboot
```

##### On the Pupper, install Ros 2 Humble

```bash
cd
git clone <https://github.com/mangdangroboticsclub/mini_pupper_ros.git> -b ros2-dev mini_pupper_ros
cd mini_pupper_ros
./install.sh
```

#### On the Pupper, download the urdf file

```bash
cd
git clone <https://github.com/mangdangroboticsclub/mini_pupper_ros.git> -b ros2-dev mini_pupper_ros_urdf
```

#### For simplicity, always source ros 2

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
reboot
```

#### On the host machine, install rust

```bash
curl --proto '=https' --tlsv1.2 -sSf <https://sh.rustup.rs> | sh
```

#### Then install rerun

```bash
cargo install rerun-cli
rerun
```

#### Now in one terminal on the Pupper, run bringup

```bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

#### In another terminal, run Swanky

```bash
cd
git clone <https://github.com/MilanLR/ROS2-rerun-integration> -b node
cd ROS2-rerun-integration
```

#### Run Swanky

###### Optionally, change the IP address in `rerun_visualization/rerun_node.py` to your hosts' IP address first

```bash
colcon build --symlink-install && \
    . install/setup.bash && \
    ros2 run rerun_visualization rerun_node
```
