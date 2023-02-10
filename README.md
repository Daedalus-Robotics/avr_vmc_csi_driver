# Deadalus Robotics Bell AVR 2023 CSI Driver

A simple ROS2 node that publishes frames from a CSI camera on a Jetson Nano

## Build

Go to your workspace `src` drirectiory

```bash
git clone https://github.com/Daedalus-Robotics/avr-vmc-2023-csi-driver.git csi_driver
git clone https://github.com/ros-perception/image_common.git --branch 3.0.0 --single-branch
cd ..
colcon build --symlink-install
```

## Run

```bash
. install/local_setup.bash
ros2 launch csi_driver csi_driver.launch.py
```
