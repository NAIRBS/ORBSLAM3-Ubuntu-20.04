# ORBSLAM3-Ubuntu-20.04
Instructions on how to build and run ORBSLAM3 on the Jetson Xavier NX (on 20.04)

Go through the instructions listed in README of the 3 folders starting from:
1. Jetson Xavier Initial Setup << Dependencies and general setup for the Jetson Xavier
2. ORBSLAM3 Setup << Specifically ORB_SLAM_COMMUNITY
3. ROS2 Node Setup << A ROS2 Wrapper linked with ORBSLAM3 to run with local network ESP32 Camera Modules

## After the setup for the 3 READMEs mentioned above, run the following in every new terminal session if rebuilding ORBSLAM3 or the ROS2 Wrapper
### Adjust paths where necessary, this assumes that all dependencies are stored in the home directory for convenience, and using WSL 2.0 or default Ubuntu 20.04 for the Jetson Xavier NX in Jetpack 5.1.5
### For WSL 2.0
```
export CMAKE_PREFIX_PATH=~/vcpkg/installed/x64-linux/share/pangolin
export CMAKE_PREFIX_PATH="$HOME/vcpkg/installed/x64-linux/share/:$CMAKE_PREFIX_PATH"
export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
export OpenCV_DIR=~/vcpkg/installed/x64-linux/share/opencv4
export PATH=$PATH:~/vcpkg/installed/x64-linux/bin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/vcpkg/installed/x64-linux/lib/pkgconfig
```

### For Ubuntu 20.04
```
export CMAKE_PREFIX_PATH=~/vcpkg/installed/arm64-linux/share/pangolin
export CMAKE_PREFIX_PATH="$HOME/vcpkg/installed/arm64-linux/share/:$CMAKE_PREFIX_PATH"
export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
export OpenCV_DIR=~/vcpkg/installed/arm64-linux/share/opencv4
export PATH=$PATH:~/vcpkg/installed/arm64-linux/bin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/vcpkg/installed/arm64-linux/lib/pkgconfig
```

# Explanation of Terminal Setup Commands:
To find where your VCPKG folder is on your system, specifically for linking the Pangolin GUI libraries:
```
find ~/vcpkg/installed -name "PangolinConfig.cmake"

# Returns something like:
# WSL
/home/[USERNAME]/vcpkg/installed/x64-linux/share/pangolin/PangolinConfig.cmake

# Ubuntu 20.04
/home/[USERNAME]/vcpkg/installed/arm64-linux/share/pangolin/PangolinConfig.cmake
```
## Depending on if you are on WSL or Native Ubuntu 20.04:
```
# WSL
export CMAKE_PREFIX_PATH=~/vcpkg/installed/x64-linux/share/pangolin
export CMAKE_PREFIX_PATH="$HOME/vcpkg/installed/x64-linux/share/:$CMAKE_PREFIX_PATH"

# Ubuntu 20.04
export CMAKE_PREFIX_PATH=~/vcpkg/installed/arm64-linux/share/pangolin
export CMAKE_PREFIX_PATH="$HOME/vcpkg/installed/arm64-linux/share/:$CMAKE_PREFIX_PATH"
```
## If you encountered issues with linking Eigen while building the ROS2 Wrapper, you only need to run this once to set up a symbolic link during setup, but it is mentioned here again:
```
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```
## Run the following for the ROS2 Wrapper/ORBSLAM to be able to find OpenCV 4.11 and other dependencies
```
# WSL
export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
export OpenCV_DIR=~/vcpkg/installed/x64-linux/share/opencv4
export PATH=$PATH:~/vcpkg/installed/x64-linux/bin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/vcpkg/installed/x64-linux/lib/pkgconfig

# Ubuntu 20.04
export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
export OpenCV_DIR=~/vcpkg/installed/arm64-linux/share/opencv4
export PATH=$PATH:~/vcpkg/installed/arm64-linux/bin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/vcpkg/installed/arm64-linux/lib/pkgconfig
```
