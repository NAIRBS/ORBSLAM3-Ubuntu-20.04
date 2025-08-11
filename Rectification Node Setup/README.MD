---

# ROS2 (Foxy)  + Stereo rectification with IP Camera Streaming

This guide walks through setting up **ROS2 Foxy**, **stereo_camera_pipeline ROS2 Package**, and using **ESP32 cameras** for monocular and stereo SLAM.

---

## ✅ Prerequisites

* Ubuntu 20.04 (on the Jetson Xavier NX)
* [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
* Using Mobile Hotspot for the local network is the easiest, connect the ESP32 Cameras and your Jetson Xavier NX/Windows Machine running WSL/Linux VM to the Mobile Hotspot. 
* IP cameras with HTTP MJPEG streaming URLs, ESP32 Camera Webservers for our use case.

---

## 📦 Install ROS2 Foxy

Follow the [official installation guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) or use the steps below:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y

sudo apt install ros-foxy-desktop python3-argcomplete ros-dev-tools
```

Source ROS2 (you need to do this again if this is not added to your bashrc script)

```bash
source /opt/ros/foxy/setup.bash
```

## Usage with ESP32 Camera Modules
First setup your ROS2 Environment according to this guide: [ROS2 Setup](https://github.com/NAIRBS/ORBSLAM3-Ubuntu-20.04/tree/main/Rectification%20Node%20Setup)
Use this ROS2 node to publish rectified Stereo ESP32 Camera Input: [stereo_camera_pipeline](https://github.com/Shye0930/stereo_camera_pipeline)
1. Instructions on optimal ESP32 Camera flashing and calibration here: [Calibration Guide](https://github.com/NAIRBS/ORBSLAM3-Ubuntu-20.04/tree/main/ESP32%20Calibration%20Setup)
2. Once you've flashed your ESP32s (pinhole cameras) and obtained calibration matrixes/data from step 1, you need to connect your ESP32s (with hardcoded SSID login) to the local wifi/hotspot, you also need to connect your machine to the same wifi/hotspot. You also need to format the calibration data into the yaml config file format.
3. Make sure to update the ip addresses assigned by your wifi/hotspot and the calibration file location in stereo_pipeline.launch.py (these files have been added for the original author's convenience), then run the following to start publishing video frames in ROS2 topics:
```
    $: ros2 launch stereo_camera_pipeline stereo_pipeline.launch.py
```
After you've started publishing the stereo rectified (and raw frames) from running the previous pipeline node, you should see the available topics named:
```
$: ros2 topic list
/parameter_events
/rosout
/stereo/left/camera_info
/stereo/left/image_raw
/stereo/left/rectified_images
/stereo/right/camera_info
/stereo/right/image_raw
/stereo/right/rectified_images
```
