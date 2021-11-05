# Autonomous Navigation

To run autonomous navigation in Gazebo correctly, follow given steps.

1- Run Husky or Rover in Gazebo (if available, use husky_velodyne package)

2- rosrun rover_21_control rover_sim_odom.py

3- roslaunch rover_21_localization localization.launch

4- rosrun rover_21_control autonomous_21.py

# Notes

Check the link below for video explanation of the implemented autonomous navigation algorithm.

https://www.youtube.com/playlist?list=PL2jykFOD1AWYvdLW6Alr55IydU_qFVe31
