First, you need to download USBipd if you're using WSL 2.0 to push the USB ports from windows to WSL [here](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)

After plugging in the RGBD Camera, In Windows Powershell (Administrator):
```
PS C:\Windows\system32> usbipd list
BUSID  VID:PID    DEVICE                                                        STATE
1-1    046d:c539  USB Input Device                                              Not shared
1-3    058f:9540  Alcor Micro USB Smart Card Reader                             Not shared
1-5    5986:1141  Integrated IR Camera                                          Not shared
1-7    8087:0a2b  Intel(R) Wireless Bluetooth(R)                                Not shared
1-8    5986:2113  Integrated Camera                                             Shared
1-10   2386:350e  USB Input Device                                              Not shared
1-14   8086:0b3a  Intel(R) RealSense(TM) Depth Camera 435i Depth, Intel(R) ...  Shared
```
Once you've run usbipd list and gotten your busID (eg: 1-14), open the WSL 2.0 Terminal and then run the following:
```
usbipd bind --busid 1-14 << To "Share" input thorugh usbipd
usbipd attach --busid 1-14 << Attach the USB input to flow through the currently open WSL 2.0
```

After this initial setup, you may have to reattach the busid whenever you plug out the D345i, which will be required when the terminal prompts you during calibration and setup.

## Installing Intel Realsense SDK for Calibration (on 20.04) [Source](https://github.com/IntelRealSense/librealsense)
First, install the required dependencies:
```
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo apt-get install git wget build-essential
sudo apt update
sudo apt install build-essential libtool autoconf unzip openssl libssl-dev
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
```

You should already have ROS2 Foxy setup (refer to previous guides so continue with:
```
cd ~/colcon/src
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
sudo apt install v4l-utils
sudo ./scripts/setup_udev_rules.sh

uname -r << If this returns above  6.5 run the following
./scripts/patch-realsense-ubuntu-lts-hwe.sh << this is to patch kernel, but I did not need to do this
# else if it returns < 5.13
./scripts/patch-realsense-ubuntu-lts.sh << this is to patch kernel, but I did not need to do this

# While still in the intel realsense folder
cmake ./ -DCMAKE_BUILD_TYPE=Release
make
sudo make -j$(($(nproc)-1)) install
```

To open the calibration software:
```
~/librealsense/Release/realsense-viewer
```
After opening the software, go to the hamburger menu, click calibration data and save it as a file.

To build the ROS2 package for the Intel RealSense to pass frames to ROS2 Topics:
```
cd ~/colcon/src/
git clone --branch 4.56.3 https://github.com/IntelRealSense/realsense-ros.git
cd ~/colcon
sudo apt-get install python3-rosdep -y
rm -rf ~/.ros/rosdep/sources.cache
sudo rosdep init
rosdep update --include-eol-distros
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
sudo apt install ros-foxy-diagnostic-updater
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```
To use the Intel RealSense
# Launch this in its own terminal
ros2 launch realsense2_camera rs_launch.py enable_color:=false enable_depth:=false enable_infra1:=true enable_infra2:=true depth_module.infra_profile:=640x480x30 

# Disable IR emitter, Launch this in its own terminal
ros2 param set /camera/camera depth_module.emitter_enabled 0

#To launch rgbd camera
python3 rgbd_ros2_ws_server.py
npm start at the example folder
