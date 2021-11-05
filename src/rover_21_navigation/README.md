# rover_21_autonomous_drive


move_base 

Paketin/algoritmanın amacı nedir?

	move_base, otonom hareket için kullanılan hazır bir pakettir. Etrafındaki engelleri belirli parametrelere bağlı olarak şişirir(costmap) ve bu costmapler ile birlikte etrafını haritalandırır. Bu harita üzerinde costmaplerden kaçarak verilen hedefe hareket gerçekleştirilir.
 
Paket/kod nasıl kullanılır?

	Paketin kullanımı sırasında kullanılacak komut sırası şu şekilde:
1- roslaunch rover_20_serial serial.launch
2- rosrun rover_20_control rover_cmd_sub_main.py
3- rosrun rover_21_control rover_odom.py
4- roslaunch rover_20_imu imu.launch
5- roslaunch rover_21_localization localization.launch
6- roslaunch velodyne_pointcloud VLP16_points.launch
**7- roslaunch rover_21_navigation move_base.launch**
8- rviz

1-7 komutları ssh üzerinden araçta çalıştırılır.
8. komut üzerinden ssh kurulan bilgisayarda çalıştırılır.



