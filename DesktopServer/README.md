# DesktopServer

The main server has three file descriptors dedicated to data transmisson:
1. serial (for communication with stm32)
2. UDP server (for communication with Android device)
3. UDP client (for passing encoder values to akula_package ROS node).
It is responsible for passing controls from Android joystick to stm32 board in PWM format, 
enabling/disabling sensors and data recording via systemd services calls, sending heartbeats to 
the Android app with information about connection to stm32 board, sensors and recording status.
