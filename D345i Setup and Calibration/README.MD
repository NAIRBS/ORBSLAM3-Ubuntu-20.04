# Intel RealSense D345i Setup and Calibration on WSL 2.0

## USB Device Sharing with WSL 2.0

First, install **USBipd** on Windows to forward USB ports to WSL 2.0:  
[https://learn.microsoft.com/en-us/windows/wsl/connect-usb](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)

After plugging in your RGBD Camera, open **Windows PowerShell (Administrator) and your WSL Terminal** and run:

```powershell
PS C:\Windows\system32> usbipd list
BUSID  VID:PID    DEVICE                                                        STATE
1-1    046d:c539  USB Input Device                                              Not shared
1-3    058f:9540  Alcor Micro USB Smart Card Reader                             Not shared
1-5    5986:1141  Integrated IR Camera                                          Not shared
1-7    8087:0a2b  Intel(R) Wireless Bluetooth(R)                                Not shared
1-8    5986:2113  Integrated Camera                                             Shared
1-10   2386:350e  USB Input Device                                              Not shared
1-14   8086:0b3a  Intel(R) RealSense(TM) Depth Camera 435i Depth, Intel(R) ...  Shared
````

Note your `BUSID` (e.g., `1-14` for the RealSense device).

Then, in your PowerShell Terminal, continue to run:

```bash
usbipd bind --busid 1-14    # To "share" the USB device through usbipd
usbipd attach --busid 1-14 --wsl  # Attach the USB device to the current WSL 2 session
```

> You may need to reattach the USB device after unplugging or restarting the terminal.

---

## Installing Intel RealSense SDK (Ubuntu 20.04)

### 1. Update and install dependencies

```bash
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo apt-get install git wget build-essential
sudo apt update
sudo apt install build-essential libtool autoconf unzip openssl libssl-dev
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
```

### 2. Clone librealsense repository and set up (assuming you've already set up ROS2 Foxy from previous guides)

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
sudo apt install v4l-utils
sudo ./scripts/setup_udev_rules.sh
```

### 3. Patch kernel if needed

Check kernel version:

```bash
uname -r
```

* If kernel version is **above 6.5**, run:

  ```bash
  ./scripts/patch-realsense-ubuntu-lts-hwe.sh
  ```

* If kernel version is **below 5.13**, run:

  ```bash
  ./scripts/patch-realsense-ubuntu-lts.sh
  ```

*Note: The patch scripts may not be necessary depending on your kernel.*

### 4. Build and install librealsense

```bash
cmake ./ -DCMAKE_BUILD_TYPE=Release
make << this could take more than 30 mins
sudo make -j$(($(nproc)-1)) install
```

---

## Running Calibration Software

To open the calibration GUI:

```bash
~/librealsense/Release/realsense-viewer
```

* Open the hamburger menu
* Click **Calibration Data**
* Save the calibration to a file

---

## Building Intel RealSense ROS2 Package

```bash
cd ~/colcon/src/
git clone --branch 4.56.3 https://github.com/IntelRealSense/realsense-ros.git
cd ~/colcon
sudo apt-get install python3-rosdep -y
rm -rf ~/.ros/rosdep/sources.cache
sudo rosdep init
rosdep update --include-eol-distros
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
sudo apt install ros-foxy-diagnostic-updater
colcon build << this could take up to 15 mins
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Launching Intel RealSense Camera Nodes

### Launch the camera with IR streams enabled (disable colour and depth)

```bash
ros2 launch realsense2_camera rs_launch.py enable_color:=false enable_depth:=false enable_infra1:=true enable_infra2:=true depth_module.infra_profile:=640x480x30
```

### Disable IR emitter (run in a separate terminal)

```bash
ros2 param set /camera/camera depth_module.emitter_enabled 0
```

---

## Running RGBD Camera Stream Server

In your project folder for AlvaAR, launch the ROS2 stream server:

```bash
python3 rgbd_ros2_ws_server.py
```

And then start the web client in the examples folder:

```bash
npm start
```

---
