# AkulaControl

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

### Building AndroidApp (Joystick and control buttons on Android):
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
