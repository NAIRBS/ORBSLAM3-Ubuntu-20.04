---

# ROS2 (Foxy)  + ORB-SLAM3 (Community  Version) Integration with IP Camera Streaming

This guide walks through setting up **ROS2 Foxy**, **ORB-SLAM3**, and using **ESP32 cameras** for monocular and stereo SLAM using `cv2.VideoCapture` streams.

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

Source ROS2:

```bash
source /opt/ros/foxy/setup.bash
```

---

## 🧠 Install ORB-SLAM3 ROS2 Wrapper

Clone the ROS2 wrapper for ORB-SLAM3:

```bash
sudo apt install ros-foxy-vision-opencv ros-foxy-message-filters

mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/doeswork/EASY-ORB-SLAM3.git

cp -r EASY-ORB-SLAM3/src/orbslam3_ros2 ./orbslam3
rm -rf EASY-ORB-SLAM3
```

Edit `CMakeLists.txt` (line 5):

```cmake
set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages:/usr/local/lib/python3.8/dist-packages")
```

Update CMakeModules to find ORBSLAM3 Folder, adjust when necessary when using different builds of ORBSLAM3:
```bash
nano ~/colcon_ws/src/orbslam3/CMakeModules/FindORB_SLAM3.cmake
```
Change the line below to point to the right folder:
```bash
set(ORB_SLAM3_ROOT_DIR "~/ORB_SLAM3")
# to
set(ORB_SLAM3_ROOT_DIR "~/ORB_SLAM_COMMUNITY")
```

---

## 🛠️ Environment Setup

Set the CMake prefix path:

For WSL 2.0
```bash
export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
export CMAKE_PREFIX_PATH=~/vcpkg/installed/x64-linux/share/pangolin
export CMAKE_PREFIX_PATH="$HOME/vcpkg/installed/x64-linux/share/:$CMAKE_PREFIX_PATH"
```

> If using ARM64 or WSL2, adjust paths accordingly:

For Ubuntu 20.04
```bash
export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
export CMAKE_PREFIX_PATH=~/vcpkg/installed/arm64-linux/share/pangolin
export CMAKE_PREFIX_PATH="$HOME/vcpkg/installed/arm64-linux/share/:$CMAKE_PREFIX_PATH"
```

Add to `~/.bashrc` (only have to do this once, will be persistent across sessions):

```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🔨 Build ORB-SLAM3 Wrapper

```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-select orbslam3
```

---

## 📁 Setup ORB Vocabulary

```bash
cd ~/colcon_ws/src/orbslam3/vocabulary
tar -xzf ORBvoc.txt.tar.gz
```

---

## 🎥 Monocular Camera Publisher (Python)

Create the script:

```bash
nano ~/colcon_ws/src/orbslam3/src/url_camera_publisher.py
```

## Paste the following (remember to update the URL based on the IP assigned to your camera by your Mobile Hotspot): [Here](https://github.com/NAIRBS/ORBSLAM3-Ubuntu-20.04/blob/main/ROS2%20Node%20Setup/url_camera_publisher.py)

Make it an executable:

```bash
chmod +x ~/colcon_ws/src/orbslam3/src/url_camera_publisher.py
```

---

## ▶️ Run Monocular ORB-SLAM3
### You need to run this on 2 separate terminals:
**Terminal 1 – Camera Publisher:**

```bash
python3 ~/colcon_ws/src/orbslam3/src/url_camera_publisher.py
```

**Terminal 2 – SLAM Node:**

```bash
ros2 run orbslam3 mono ~/colcon_ws/src/orbslam3/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3/config/monocular/TUM1.yaml
```

---

## 🎥 Stereo Camera Publisher

Create the script:

```bash
nano ~/colcon_ws/src/orbslam3/src/stereo_url_camera_publisher.py
```

## Paste the following (remember to update the URL based on the IP assigned to your camera by your Mobile Hotspot): [Here](https://github.com/NAIRBS/ORBSLAM3-Ubuntu-20.04/blob/main/ROS2%20Node%20Setup/stereo_url_camera_publisher.py)

Make it executable:

```bash
chmod +x ~/colcon_ws/src/orbslam3/src/stereo_url_camera_publisher.py
```

---

## ▶️ Run Stereo ORB-SLAM3
### You need to run this on 2 separate terminals:
**Terminal 1 – Stereo Camera Publisher:**

```bash
python3 ~/colcon_ws/src/orbslam3/src/stereo_url_camera_publisher.py
```

**Terminal 2 – SLAM Node (Note that this runs Fisheye lens, which ESP32 Cameras by default do not use, this will launch a GUI but will likely not work if at all:**

```bash
ros2 run orbslam3 stereo ~/colcon_ws/src/orbslam3/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3/config/stereo/TUM-VI.yaml false
```

Or with your own config such as Pinhole lens which the ESP32 Cameras use by default:

```bash
ros2 run orbslam3 stereo ~/colcon_ws/src/orbslam3/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3/config/stereo/config.yaml false
```

Create the config file config.yaml with:

```bash
nano ~/colcon_ws/src/orbslam3/config/stereo/config.yaml
```

## Paste the contents of the file from [Here](https://github.com/NAIRBS/ORBSLAM3-Ubuntu-20.04/blob/main/ROS2%20Node%20Setup/config.yaml)

---



