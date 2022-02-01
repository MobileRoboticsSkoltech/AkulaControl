#!/bin/bash

SAVE_PATH=RosBags
TOPICS=(
    '/basler/image' 
    '/basler/camera_info'
    '/robot_description'
    '/velodyne_points'
)

source /home/mrob/ws/install/setup.bash
exec ros2 bag record -o "$SAVE_PATH/$(date '+%Y-%m-%d-%H-%M-%S')" "${TOPICS[@]}"
