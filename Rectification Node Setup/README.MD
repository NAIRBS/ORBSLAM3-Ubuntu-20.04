---

# ROS2 (Foxy)  + ORB-SLAM3 (Community  Version) Integration with IP Camera Streaming

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

