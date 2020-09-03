# Autonomous Navigation

To run autonomous navigation correctly, follow given steps.

1- Run Husky or Rover in Gazebo (if available, use husky_velodyne package)
2- rosrun rover_21_control rover_sim_odom.py
3- roslaunch rover_21_localization localization.launch
4- rosrun rover_21_control autonomous_21.py
