Localization

Paketin/algoritmanın amacı nedir?

Lokalizasyon, kısaca otonom aracın kendi konumunu bilmesidir. Roverın konumu; odometri, LIDAR, IMU ve GPS sensörlerinden gelen veriler sayesinde bilinir. Sensör verilerinin EKF (Extended Kalman Filter)'ye sokulması ile hata payı oldukça düşürülür.

Paket/kod nasıl kullanılır?

Paketin kullanımı sırasında kullanılacak komut sırası şu şekilde:
1- roslaunch rover_20_serial serial.launch
2- rosrun rover_20_control rover_cmd_sub_main.py
3- rosrun rover_21_control rover_odom.py
4- roslaunch rover_20_imu imu.launch
**5- roslaunch rover_21_localization localization.launch**
6- roslaunch velodyne_pointcloud VLP16_points.launch
7- roslaunch rover_21_navigation move_base.launch
8- rviz
