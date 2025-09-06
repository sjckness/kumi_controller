# kumi_controller
ros2 + gazebo model and control


## Issues / ToDo
(+++) effort controller for better friction  
(++) resolve phisics problems in the urdf   

 

## Warnings
(+) no real time kernel

## Solved issues
(+++) (1)controller_manager not working             15/07    
(+++) (2)joint_trajectory_controller not loaded     21/07  
(+++) (3)control the joint via python script        22/07  
(+++) (4)robot goes around without permission!      27/08  
(++) configure IMU plugin                           29/08  

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


## Descrizione delle giunzioni e link

| Link         | Joint Parent     | Tipo Joint | Note                    |
|--------------|------------------|------------|-------------------------|
| base_link    | —                | —          | Base fissa              |
| body         | base_link        | fixed      | Connessione fissa       |
| front_leg    | body             | revolute   | Spalla anteriore        |
| front_leg_d  | front_leg        | revolute   | Ginocchio anteriore     |
| front_foot   | front_leg_d      | revolute   | Caviglia anteriore      |
| back_leg     | body             | revolute   | Spalla posteriore       |
| back_leg_d   | back_leg         | revolute   | Ginocchio posteriore    |
| back_foot    | back_leg_d       | revolute   | Caviglia posteriore     |  