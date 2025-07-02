# ORB-SLAM3 Setup Guide (with Pangolin, OpenCV, and VCPKG)

## [Dependencies Required](https://github.com/UZ-SLAMLab/ORB_SLAM3)
```
| Dependency     | Notes                                                |
| -------------- | ---------------------------------------------------- |
| Pangolin       | Visualization library                                |
| OpenCV         | Computer vision library                              |
| Eigen3         | Linear algebra library                               |
| DBoW2 / g2o    | Already included in `Thirdparty` folder of ORB-SLAM3 |
| Python         | For dataset scripts and bindings                     |
| ROS (Optional) | For ROS integration                                  |
```

## 1. Pangolin Installation (via VCPKG)

> Reference: [Lindevs vcpkg Ubuntu Guide](https://lindevs.com/install-vcpkg-on-ubuntu)

### 1.1 Install VCPKG

```bash
cd ~

# Install zip and unzip packages + for libraries compilation, you need to install build-essential and pkg-config packages:
sudo apt update
sudo apt install -y zip unzip build-essential pkg-config

# Download vcpkg from GitHub repository:
wget -qO vcpkg.tar.gz https://github.com/microsoft/vcpkg/archive/master.tar.gz

# Create a new directory to store vcpkg and extract the tar.gz file to it:
sudo mkdir /opt/vcpkg
sudo tar xf vcpkg.tar.gz --strip-components=1 -C /opt/vcpkg

# Run the following command to build vcpkg itself:
sudo /opt/vcpkg/bootstrap-vcpkg.sh

# In /usr/local/bin directory, we can create a symbolic link to the vcpkg command:
sudo ln -s /opt/vcpkg/vcpkg /usr/local/bin/vcpkg

# Now vcpkg can be used as a system-wide command for all users. We can check vcpkg version with command:
vcpkg version

# The tar.gz file is no longer needed, remove it:
rm -rf vcpkg.tar.gz
```

### 1.2 Install Pangolin Dependencies

```bash
sudo apt-get install -y nasm ffmpeg libmpfr-dev libgmp3-dev libmpc-dev
sudo apt-get install -y libxmu-dev libxi-dev libgl-dev
sudo apt install cmake
sudo apt install libgl1-mesa-dev libegl1-mesa-dev libglu1-mesa-dev libx11-dev
```

### 1.3 Build Pangolin and Install Catch2

```bash
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin/
./scripts/install_prerequisites.sh --dry-run recommended
./scripts/install_prerequisites.sh -m apt all

cd ~
git clone https://github.com/catchorg/Catch2.git
cd Catch2
cmake -Bbuild -H. -DBUILD_TESTING=OFF
sudo cmake --build build/ --target install
```

### 1.4 Install Pangolin via [vcpkg](https://github.com/microsoft/vcpkg)

```bash
cd ~/vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
./vcpkg install pangolin
```

Check for Pangolin config path:

```bash
find ~/vcpkg/installed -name "PangolinConfig.cmake"
```

Expected path (adjust as needed):

```bash
# On WSL 2.0
~/vcpkg/installed/x64-linux/share/pangolin/PangolinConfig.cmake

# On Ubuntu 20.04
~/vcpkg/installed/arm64-linux/share/pangolin/PangolinConfig.cmake
```

Set environment variables:

```bash
# On WSL 2.0
export CMAKE_PREFIX_PATH=~/vcpkg/installed/x64-linux/share/pangolin
export CMAKE_PREFIX_PATH="$HOME/vcpkg/installed/x64-linux/share/:$CMAKE_PREFIX_PATH"

# On Ubuntu 20.04
export CMAKE_PREFIX_PATH=~/vcpkg/installed/arm64-linux/share/pangolin
export CMAKE_PREFIX_PATH="$HOME/vcpkg/installed/arm64-linux/share/:$CMAKE_PREFIX_PATH"
```

---

## 2. OpenCV Installation with [vcpkg](https://github.com/microsoft/vcpkg)

```bash
sudo apt-get install python3-opencv
sudo apt upgrade libopencv-dev python3-opencv
```

### Additional OpenCV Setup (VCPKG)

```bash
sudo apt update
sudo apt install -y libgtk2.0-dev pkg-config bison libxi-dev libxtst-dev
sudo apt install -y libx11-dev libxft-dev libxext-dev libxrandr-dev
pip3 install jinja2
python3 -m pip install meson ninja mako

cd ~/vcpkg
git pull
./vcpkg update
./vcpkg install libsystemd:arm64-linux
# Note that this builds 28-29 dependencies from source and can take up to 3-5 hours on the Jetson Xavier NX, but only needs to be built once
./vcpkg install opencv4
```

### Fix for libffi error (if any)

Edit:

```bash
nano ~/vcpkg/ports/libffi/portfile.cmake
```

Comment out:

```cmake
# vcpkg_cmake_get_vars(...)
# include("${cmake_vars_file}")
```

### Final OpenCV Configuration

```bash
# On WSL 2.0
export OpenCV_DIR=~/vcpkg/installed/x64-linux/share/opencv4
export PATH=$PATH:~/vcpkg/installed/x64-linux/bin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/vcpkg/installed/x64-linux/lib/pkgconfig

# On Ubuntu 20.04
export OpenCV_DIR=~/vcpkg/installed/arm64-linux/share/opencv4
export PATH=$PATH:~/vcpkg/installed/arm64-linux/bin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/vcpkg/installed/arm64-linux/lib/pkgconfig

pkg-config --modversion opencv4
```

Upgrade Python packages:

```bash
pip install --upgrade numpy opencv-python
```

---

## 3. Eigen Installation

```bash
sudo apt update
sudo apt install libeigen3-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

---

## 4. ORB-SLAM3 Build Setup (ORB_SLAM_COMMUNITY Recommended)

### Clone Repository

```bash
git clone https://github.com/jeremysalwen/ORB_SLAM_COMMUNITY

# You can also try ORBSLAM3, but it requires a lot more configuration to get working and it's just generally worse.
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3

cd ORB_SLAM_COMMUNITY 
# or: cd ORB_SLAM3
chmod +x build.sh
```

### Required Libraries

```bash
sudo apt install libcurl4-openssl-dev libssh-dev libssl-dev
sudo apt-get install libboost-all-dev
```

### Modify Source Files

Edit only if you plan to run Monocular Example:

```bash
cd ~/ORB_SLAM_COMMUNITY/Examples/Monocular
# or: cd ~/ORB_SLAM3/Examples/Monocular
nano mono_tum_vi.cc
```
```cpp
ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false, 0, file_name);
# Change the above false to true to enable the visualizer, else no GUI will open when running the monocular example with mono_tum_vi datasets
ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true, 0, file_name);
```

Only if you are using ORBSLAM3, NOT the community version, you have to change the constructor line, update `CMakeLists.txt` to use C++14:

```cmake
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
```

---

## 5. Add Swapfiles for Compilation for Jetson Xavier, note that it will crash without this while building

Install ZRAM (compressed RAM, 4GB by default):

```bash
sudo apt install zram-config
```

Setup Swapfile on Jetson Xavier NX:

```bash
sudo swapoff /swapfile
sudo rm /swapfile
# 16 GB should be enough, but I've personally seen it go past 17 GB while compiling, so to be safe use 32 GB
sudo fallocate -l 32G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

---

## 6. Build ORB-SLAM3

```bash
cd ~/ORB_SLAM_COMMUNITY
./build.sh
```

---


## 7. Testing with TUM-VI Dataset

```bash
mkdir -p ~/Datasets/TUM-VI
cd ~/Datasets/TUM-VI
wget https://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-corridor4_512_16.tar
tar -xf dataset-corridor4_512_16.tar
```

Create and run test script:

```bash
cd ~/ORB_SLAM3
nano tum_vi_examples.sh
```

## You can find the original example shell scripts [here](https://github.com/JiangPei-lab/ORB-SLAM3-JiangPei/tree/master), the original ORBSLAM3 repository does not have the example scripts.
## Copy the relevant TUM-VI script and adjust the relevant dataset paths if you don't leave everything in the home directory. Remember to change [USERNAME] if you do.

```bash
#!/bin/bash
pathDatasetTUM_VI='/home/[USERNAME]/Datasets/TUM-VI/dataset-corridor4_512_16' # REMEMBER TO UPDATE THIS TO YOUR PATH
data_name='dataset-corridor4_512'

#++++++++++++++++++++++++++++++++++++
# Monocular Examples
echo "Launching Room 1 with Monocular sensor"
./Examples/Monocular/mono_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular/TUM-VI.yaml "$pathDatasetTUM_VI"/mav0/cam0/data Examples/Monocular/TUM_TimeStamps/dataset-corridor4_512.txt dataset-corridor4_512_mono
```

# After you've created the tum_vi_examples.sh file, run it with:
```
chmod +x tum_vi_examples.sh
./tum_vi_examples.sh
```
# You should observe the terminal load in the parameters, and eventually open 2 GUIs, one for the monocular camera input and the other for the SLAM key feature mapping and localisation, as well as camera trajectory.

