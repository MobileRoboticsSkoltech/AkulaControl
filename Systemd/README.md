# Setting Linux service to start/stop ROS2 sensor nodes and recording

### Setup:

1. In order to use services in the userspace it's necessary to put them in the proper directory:
```
mkdir -p ~/.config/systemd/user
cp AkulaControl/Systemd/akula_sensors_*.service ~/.config/systemd/user
```

2. Inside the both services change path (**global!**) to the scripts that run all the ROS related stuff:
```
[Service] 
Type=simple
ExecStart=path/to/AkulaSensorsLauncher.sh
```
or
```
ExecStart=path/to/AkulaSensorsRecord.sh
```
3. Inside the both scripts change **source** path to your ROS environment directory:
```
source path/to/env/install/setup.bash
```
4. Inside the **AkulaSensorsRecord.sh** script change the path to directory to save all the ROS bags to:
```
SAVE_PATH=path/to/save/rosbags
```
5. In the same **AkulaSensorsRecord.sh** script specify the topic you want to record:
```
TOPICS=(
    '/basler/image' 
    '/basler/camera_info'
    '/robot_description'
    '/velodyne_points'
)
```

### Usage:

1. Starting/stopping sensors:
```
systemctl --user start akula_sensors_launcher.service
systemctl --user stop akula_sensors_launcher.service
```

2. Starting/stopping recording:
```
systemctl --user start akula_sensors_record.service
systemctl --user stop akula_sensors_record.service
```

3. Checking service status:
```
systemctl --user status akula_sensors_launcher.service
systemctl --user status akula_sensors_record.service
```

### Warning

If you want to change the service, be sure to use **exec** for calling the scripts to ensure that the sevice shares PID with the script underneath, otherwise stopping the service will result in loosing control over the script execution.
