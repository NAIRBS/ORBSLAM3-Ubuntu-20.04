# Jetson Xavier NX Setup Guide (8 GB Model) (Based on Jetpack 5.1.5)
> 📺 **Reference Guide**:
> [YouTube Walkthrough](https://www.youtube.com/watch?v=LuWYXKrQcJ8&list=PLGs0VKk2DiYw_Xx7c3M_9HMEcn9hPhFJV&index=2)

> Try not to deviate from the recommended versions and instructions, the Jetson Xavier NX is very picky on what it can and cannot run, especially since it's quite old since [release](https://developer.nvidia.com/blog/nvidia-releases-jetson-xavier-nx-developer-kit-with-cloud-native-support/) 5 years and counting from the writing of this README.

> Almost every single modern/recent dependency and library is incompatible with the Jetson Xavier NX; usually, stock Ubuntu 20.04 apt installations will be used, which in itself is ALSO outdated. [vcpkg](https://github.com/microsoft/vcpkg) will be used where possible to avoid building everything from source if recent dependencies are required by ORBSLAM3/ROS2, such as OpenCV.

> Do not bother with optional steps if unnecessary for your needs, you will find that it barely works at a serviceable level, and is often not worth the effort.

> It is HIGHLY RECOMMENDED to do all ORBSLAM3 + ROS2 development work (if changing the source code is necessary) on a Linux VM or WSL 2.0 before installation on the Jetson Xavier NX. The Jetson Xavier NX itself is VERY SLOW when rebuilding/installing, ORBSLAM3 can take up to 3-4 hours to build and install.

## 🗂️ 1. SD Card Preparation

* Use the **largest SD card** you have available
- ≥64GB is recommended, ~4GB for OS + ~10+ GB Dependencies + ~16 GB Swapfile + Additional storage for ORBSLAM3 + ROS2 + Example files to test ORBSLAM3 (each minimally 5-10 GB large).
* Format the card as **exFAT** (required for files > 4GB, which consists of mostly ORBSLAM3 test files that come in large compressed files).
* A **32GB SD card is NOT sufficient** for Jetpack and ORBSLAM3 development needs.

## ⚙️ 2. Flash Jetpack 5.1.5

* Ensure **QSPI firmware** has been updated to support Jetpack versions **5.x and above**, once you reflash the firmware following this [YouTube Walkthrough](https://www.youtube.com/watch?v=NJyHJzG6On0), it will persist across multiple failed Ubuntu installations and only needs to be done once.
* Flash the SD card with [Jetpack 5.1.5](http://developer.nvidia.com/embedded/jetpack-sdk-515) with another machine (preferably Windows), following this [YouTube Walkthrough](https://youtu.be/LuWYXKrQcJ8?list=PLGs0VKk2DiYw_Xx7c3M_9HMEcn9hPhFJV&t=658)
* Insert the SD card into the Jetson NX and follow the initial setup steps in the [YouTube Walkthrough](https://youtu.be/LuWYXKrQcJ8?list=PLGs0VKk2DiYw_Xx7c3M_9HMEcn9hPhFJV&t=17132)
* Note that you need a Wired USB-A Mouse (or wireless with a dongle), Keyboard (wireless should work, but there are supposedly some issues with Bluetooth), a HDMI/DISPLAY-PORT wired Display and at least 1-3 Hours.

## 🖥️ 3. Initial Ubuntu Configuration

1. **Display Settings**:

   * Set **Scale** to **125%** under Ubuntu Display Settings.

2. **UI Preferences**:

   * Change the **theme to Dark**.
   * Disable **Power Saving > Blank Screen**.
   * Disable **Screen Lock** completely.

3. **Terminal Font Configuration**:

   * Open Terminal → Preferences → *Unnamed* Profile → Text → Enable **Custom Font** → Set **Font Size** to **17**.

## 🔄 4. Software Updates

* Accept the Ubuntu **Software Updater** prompt and allow it to complete system updates. This will take ~ 1 Hour.

## 🐍 5. Install pip and jtop

```bash
sudo apt-get install python3-pip
sudo pip3 install -U jetson-stats
```

## ⚡ 6. Power Mode Settings

* Set to **20W 6-core mode** through the GUI on the top right dropdown.

---

## 🔧 7. Install Essential Utilities

```bash
sudo apt install nano apt-utils
```

## 🧰 8. (Optional) Install Visual Studio Code (v1.5 to v1.65.2), however, this takes a long time to open, not recommended if not necessary.

> Note: Jetson requires a compatible ARM64 version AND VSCode will not work past certain versions according to [forum posters](https://forums.developer.nvidia.com/t/vs-code-can-t-launch-with-jetpack-5-0/213980/13)
> v1.65.2 is recommended as it works better with certain plugins.
```bash
wget https://update.code.visualstudio.com/1.50.0/linux-deb-arm64/stable -O stable.deb
sudo dpkg -i stable.deb
```

## ☀️ 9. (Optional) Install [Sunshine](https://github.com/LizardByte/Sunshine/releases/tag/v0.22.2) 
### Useful if you want to work on the Jetson through a Windows machine, note that performance is serviceable but still not good, still requires a display connected to the Jetson Xavier NX.

* Install **Sunshine v0.22.2** – is the latest version that supports **Ubuntu 20.04** on Jetpack 5.1.5, anything past this might not work.

## 🖱️ 10. (Optional) Install Barrier (for KVM/Mouse Sharing)
### Both Windows Machine and the Jetson Xavier NX need to be connected to the same Wifi, recommended if working on both Windows + Jetson Xavier NX at the same time, side by side

* **Jetson**: The Latest Barrier version for Linux is fine.
* **Server (PC/Laptop)**: Install [**Barrier v2.3.4**](https://github.com/debauchee/barrier/releases/tag/v2.3.4) to ensure compatibility; it will not work with any other version

## 🧭 11. Install ORB_SLAM_COMMUNITY 
### Contains merged improvements and bug fixes from original ORBSLAM3 after 3 years of inactivity, some functionality will not work without these improvements to the original codebase.

* Follow the ORB-SLAM3 installation guide [here](https://github.com/NAIRBS/ORBSLAM3-Ubuntu-20.04/tree/main/ORBSLAM3%20Setup)
* Clone and build [ORB_SLAM_COMMUNITY](https://github.com/jeremysalwen/ORB_SLAM_COMMUNITY).
* Ensure Pangolin, OpenCV (requires GUI support and above v4.4), Eigen, and ROS are properly configured beforehand, these 4 specific libraries require environment variable manipulation in either the .bashrc file (ROS), at the start of every terminal session (Pangolin/OpenCV for ORBSLAM3 + ROS2) or one-time symbolic file setup (Eigen for ROS2).

## 🤖 12. Install ROS + ROS Node

* Follow the ROS installation guide for **Jetpack 5.1.5 (Ubuntu 20.04)** [here](https://github.com/NAIRBS/ORBSLAM3-Ubuntu-20.04/tree/main/ROS2%20Node%20Setup)


