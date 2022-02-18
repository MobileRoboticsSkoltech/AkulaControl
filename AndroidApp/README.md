# AndroidApp

### Description

Android app is used as a control interface for moving the robot, enabling/disabling sensors and data recording. <br>
There are four virtual LEDs that show statuses
of processes and robot active connections:
1. Yellow - shows whether the stm32 board is connected to the server via serial port
2. Green - shows whether the smartphone is connected to the server
3. Blue - shows whether Akula ROS2 package is active (polls corresponding service status)
4. Red - shows whether ROS2 bag recording is enabled (polls corresponding service status)

Text views:
1. In the bottom-left corner there is a text view that shows current encoders counter values that are sent by the stm32 board. 
2. The text view in the bottom-right
corner is dedicated to show the quality of the wireless connection displaying ping in milliseconds (time which a packet takes to go from the smartphone to
the stm32 board and back)

Buttons:
1. START SENSORS starts the systemd service dedicated to launching ROS2 nodes of the sensors specified in the **Main.launch.py** file
2. LATENCY sends a packet to test wireless connection quality (explained in the **Text views** list)
3. RECORD starts ROS2 bag recording of the topics specified in the **AkulaSensorsRecord.sh** script by calling the corresponding systemd service

The joystick is linear (the circle is converted to a rhombus).

<p align="center">
  <img src="https://github.com/MobileRoboticsSkoltech/AkulaControl/blob/assets/AndroidUI.jpg" width="400">
</p>
