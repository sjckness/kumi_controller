# kumi_controller
ros2 + gazebo model and control


## Issues
(+++) (2)joint_trajectory_controller not loaded


## Warnings

## Solved issues
(+++) (1)controller_manager not working 15/07

## About
ubuntu version: 22.04.5 LTS  
ros2 version: ros2 humble  
gazebo version: 11.14

## launch
in /dev_ws  
-> colcon build --symlink-install 
-> source install/setup.bash  
-> ros2 launch kumi_controller kumi_gz.launch.py

to kill gazebo
-> killall -9 gazebo gzserver gzclient
