# Deadalus Robotics Bell AVR 2023 CSI Streamer

A simple ROS2 node that publishes frames from a CSI camera on a Jetson Nano

## Build

Go to your workspace `src` drirectiory

```bash
git clone https://github.com/Daedalus-Robotics/avr-vmc-2023-csi-camera.git
git clone https://github.com/ros-perception/image_common.git --branch 3.0.0 --single-branch
cd ..
colcon build --symlink-install
```

## Run

```bash
. install/local_setup.bash
ros2 run csi_camera streamer --ros-args --params-file src/csi_camera/config/example.yaml
```
