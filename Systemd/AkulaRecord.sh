#!/bin/bash

source /home/mrob/ws/install/setup.bash
exec ros2 bag record -o "$(date '+%Y-%m-%d-%H-%M-%S')" /basler/image /basler/camera_info /robot_description /velodyne_points
