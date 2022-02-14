# AkulaControl

### Description

AkulaControl is a package for controlling skid steering robot, obtaining data from Velodyne VLP-16 lidar, Basler camera and encoders and publishing everything through ROS2 Foxy. The package is consisted of the main server (DesktopServer), stm32f407 board firmware (STM32), mobile control UI (AndroidApp) and ROS2 package (AkulaPackageROS2). It is quite easy to adapt this code to any sensors you want to use or even platforms due to modules separation and the fact that communication between them is packet-based.

The main server has three file descriptors dedicated to data transmisson:
1. serial (for communication with stm32)
2. UDP server (for communication with Android device)
3. UDP client (for passing encoder values to akula_package ROS node).
It is responsible for passing controls from Android joystick to stm32 board in PWM format, enabling/disabling sensors and data recording via systemd services calls, sending heartbeats to the Android app with information about connection to stm32 board, sensors and recording status.

### Requirements:
Start obtaining dependencies with

```
sudo apt update
```
1. libyaml-cpp-dev
2. dSocket (already included as a reference)
3. cmake (version >= 3.16)
4. gcc, g++ (version >= 9.3.0)
5. arm-none-eabi-g++, arm-none-eabi-gcc (version >= 9.2.1)
6. **(optional)** basler-ros2-driver (if you need this camera, otherwise exclude it from the Main.launch.py and from AkulaSensorsRecord.sh that currently obtains images from Basler while recording)

For some tools you can change CMakeLists.txt files to check whether building works for older versions.

### Cloning:
Recursive flag is required for obtaining dSocket library
```
git clone --recurse-submodules https://github.com/MobileRoboticsSkoltech/AkulaControl.git
```

### Building DesktopServer (main server):
1. Install dependencies:

```
sudo apt install build-essential cmake libyaml-cpp-dev
```
2. Build the project:

```
cd path/to/AkulaControl/DesktopServer
mkdir build
cd build
cmake ..
make -j
```
The project can also be easily opened with any IDE that supports cmake (Clion, for example).

### Building STM32 (stm32 board firmware):
1a. The easiest way to build the project is to use STM32CubeIDE with STM32CubeMX that can build and flash the hardware. <br>
1b. If you want to avoid using any of the IDEs, you need to setup a toolchain for cross-compilation. For that obtain ARM compilers:

```
sudo apt install gcc-arm-none-eabi
```
2b. Build the project:

```
cd path/to/AkulaControl/STM32
mkdir build
cd build
cmake ..
make -j
```
3b. Upload the firmware to a stm32 board with your tool of choice. <br>
?b. **Method b was tested after setting up STM32CubeMX that automatically installs all the tools, so, toolchain setup can possibly require more effort**

### Building AndroidApp (joystick and control buttons on Android):
1a. The easiest way to build the project is to use Android Studio. <br>
2a. To sign your app folow these instructions: https://developer.android.com/studio/publish/app-signing

1b. If you want to avoid using any of the IDEs, you need to install JDK and manually setup gradle:
```
sudo apt install openjdk-8-jdk
```
2b. Build the project:

```
cd path/to/AkulaControl/AndroidApp
./gradlew assembleRelease
```
3b. To sign your app folow these instructions: https://developer.android.com/studio/build/building-cmdline <br>
?b. **Method b was tested after setting up Android Studio that automatically configures gradle and all related stuff, so, the actual build can possibly require more effort**

### Building AkulaPackageROS2 (sensors and data recording)

1. Create a soft link of the AkulaPackageROS2 to your ROS2 environment (lower **snake_case** is due to some ROS restrictions):

```
ln -s /global/path/to/AkulaPackageROS2 /global/path/to/env/src/akula_package
```
2. **(optional)** If you want to use Basler camera, clone the repository (https://github.com/MobileRoboticsSkoltech/basler_ros2_driver) and add a soft link to it:

```
git clone https://github.com/MobileRoboticsSkoltech/basler_ros2_driver.git
ln -s /global/path/to/basler_ros2_driver /global/path/to/env/src/basler_ros2_driver
```
3. Build everything using colcon:

```
colcon build --symlink-install
```
