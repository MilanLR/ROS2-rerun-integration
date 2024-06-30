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

# Turtle-bot setup

#### First, flash Ubuntu for the raspberry pi 4 onto an micro SD card, see `https://ubuntu.com/download/raspberry-pi`. For additional setup of the network and ssh see `https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup`.

#### Install ros2 humble following `https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html`

#### Install ros2 cartographer
```bash
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```

#### Install TurtleBot3 packages and drivers
```bash
sudo apt install ros-humble-hls-lfcd-lds-driver
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-dynamixel-sdk
sudo apt install libudev-dev
```

#### Clone the nodes necessary for bringup and teleop
```bash
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
cd ~/turtlebot3_ws/
```

#### Build the nodes
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
colcon build --symlink-install --parallel-workers 1
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

#### Setup the USB port for OpenCR
```bash
sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### Set the LDS model type
```bash
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc # or LDS-01
source ~/.bashrc
```

#### Install packages to be able flash OpenCR board
```bash
sudo dpkg --add-architecture armhf
sudo apt update
sudo apt install libc6:armhf
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
rm -rf ./opencr_update.tar.bz2
```

#### Flash the OpenCR board
```bash
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xvf ./opencr_update.tar.bz2
cd ~/opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

#### Run the bringup node
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
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
