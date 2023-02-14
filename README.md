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

Replace `<YOUR INFO FILE>` with the absolute path to your info file.

```bash
. install/local_setup.bash
ros2 launch csi_driver csi_driver.launch.py framerate:=15 info_file:=<YOUR INFO FILE>
```

## Calibrate

Calibrate undistortion and rectification. Download and print
[this](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=get&target=check-108.pdf)
calibration pattern. To start the camera driver, go to your workspace `src` drirectiory.
Replace `<CAMERA_WIDTH>` and `<CAMERA_HEIGHT>` with the width and height of your camera.

```bash
. install/local_setup.bash
ros2 launch csi_driver csi_driver_raw.launch.py framerate:=15 width:=<CAMERA_WIDTH> height:=<CAMERA_HEIGHT>
```

In a new terminal go to your workspace `src` drirectiory.
Replace `<SQUARE_SIZE>` with the size of a square in meters on the camibration image.

```bash
. install/local_setup.bash
ros2 run camera_calibration cameracalibrator --size 6x8 --square <SQUARE_SIZE> image:=/csi_camera/image_raw camera:=/csi_camera
```

Hold your calibration image in front of the camera
and move it and rotate it in all directions until the calibrate button lights up.
Then press the calibrate button and then the save button. Then run:

```bash
tar -xzf /tmp/calibrationdata.tar.gz ost.yaml
```

Then to run the driver with that calibration file, run:

```bash
ros2 launch csi_driver csi_driver.launch.py framerate:=15 info_file:=`pwd`/ost.yaml
```