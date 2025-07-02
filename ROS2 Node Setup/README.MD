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

Paste the following (remember to update the URL based on the IP assigned to your camera by your Mobile Hotspot):

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.bridge = CvBridge()
        self.video_url = 'http://192.168.121.123:81/stream' # UPDATE URL HERE
        self.running = True
        self.thread = threading.Thread(target=self.stream_loop)
        self.thread.start()

    def stream_loop(self):
        self.get_logger().info(f"Connecting to {self.video_url}")
        cap = cv2.VideoCapture(self.video_url)
        if not cap.isOpened():
            self.get_logger().error("Failed to open stream.")
            return
        while self.running:
            ret, frame = cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher_.publish(msg)
            else:
                self.get_logger().warn("Failed to read frame.")
        cap.release()

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

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

Paste the following:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class StereoCameraPublisher(Node):
    def __init__(self):
        super().__init__('stereo_camera_publisher')
        self.left_pub = self.create_publisher(Image, '/camera/left', 10)
        self.right_pub = self.create_publisher(Image, '/camera/right', 10)
        self.bridge = CvBridge()
        self.left_url = 'http://192.168.121.123:81/stream'
        self.right_url = 'http://192.168.121.153:81/stream'
        self.running = True
        self.thread = threading.Thread(target=self.stream_loop)
        self.thread.start()

    def stream_loop(self):
        cap_left = cv2.VideoCapture(self.left_url)
        cap_right = cv2.VideoCapture(self.right_url)
        if not cap_left.isOpened() or not cap_right.isOpened():
            self.get_logger().error("Failed to open one or both streams.")
            return
        while self.running:
            ret_l, frame_l = cap_left.read()
            ret_r, frame_r = cap_right.read()
            if ret_l and ret_r:
                now = self.get_clock().now().to_msg()
                msg_l = self.bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
                msg_r = self.bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')
                msg_l.header.stamp = now
                msg_r.header.stamp = now
                self.left_pub.publish(msg_l)
                self.right_pub.publish(msg_r)
            else:
                self.get_logger().warn("Failed to read one or both frames.")
        cap_left.release()
        cap_right.release()

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

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

Paste the following:

```bash
%YAML:1.0

#--------------------------------------------------------------------------------------------
# System config
#--------------------------------------------------------------------------------------------

# When the variables are commented, the system doesn't load a previous session or not store the current one

# If the LoadFile doesn't exist, the system give a message and create a new Atlas from scratch
#System.LoadAtlasFromFile: "Session_MH01_MH02_MH03_Stereo60_Pseudo"

# The store file is created from the current session, if a file with the same name exists it is deleted
#System.SaveAtlasToFile: "Session_MH01_MH02_MH03_Stereo60_Pseudo"

#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------
File.version: "1.0"

Camera.type: "PinHole"

# Camera calibration and distortion parameters (OpenCV)
# Camera1.fx: 458.654
# Camera1.fy: 457.296
# Camera1.cx: 367.215
# Camera1.cy: 248.375

# Camera1.k1: -0.28340811
# Camera1.k2: 0.07395907
# Camera1.p1: 0.00019359
# Camera1.p2: 1.76187114e-05

Camera1.fx: 275.0
Camera1.fy: 275.0
Camera1.cx: 275.0
Camera1.cy: 275.0

Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0

#Camera2.fx: 457.587
#Camera2.fy: 456.134
#Camera2.cx: 379.999
#Camera2.cy: 255.238

#Camera2.k1: -0.28368365
#Camera2.k2: 0.07451284
#Camera2.p1: -0.00010473
#Camera2.p2: -3.55590700e-05

Camera2.fx: 275.0
Camera2.fy: 275.0
Camera2.cx: 275.0
Camera2.cy: 275.0

Camera2.k1: 0.0
Camera2.k2: 0.0
Camera2.p1: 0.0
Camera2.p2: 0.0

#Camera.width: 752
#Camera.height: 480

Camera.width: 320
Camera.height: 480

# Camera frames per second
Camera.fps: 20

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

Camera.bf: 13.75 # 0.05 m × 275 fx

Stereo.ThDepth: 60.0
ThDepth: 60.0
Stereo.T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
#  data: [0.999997256477797,-0.002317135723275,-0.000343393120620,0.110074137800478,
#         0.002312067192432,0.999898048507103,-0.014090668452683,-0.000156612054392,
#         0.000376008102320,0.014089835846691,0.999900662638081,0.000889382785432,
#         0,0,0,1.000000000000000]
  data: [1.0, 0.0, 0.0, 0.05,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0]


#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1000

# ORB Extractor: Scale factor between levels in the scale pyramid
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
Viewer.imageViewScale: 1.0
```
---



