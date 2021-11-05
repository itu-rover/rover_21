#!/usr/bin/env python
# 2018 URC and 2018 ERC subs joy mobile_base,  pubs to serial node
# 1-3 are the left side of the mobile_base, 2-4 are the right side of the mobile_base.
# "S + motor_1 + motor_2 + motor_3  + motor_4 + F"
# If you dont use your joy, it will send 0000 to serial node
# ITU mobile_base Team
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import sqrt



twist_cmd = Twist()
twist_nav = Twist()
twist = Twist()

pub=rospy.Publisher("/rover_serial_topic", String, queue_size=10)
multiplier = 120


def callbackcmd(data):  
	twist_cmd.linear.x = data.linear.x * multiplier  
	twist_cmd.angular.z = data.angular.z * multiplier
	
def callbacknav(data):
	twist_nav.linear.x = data.linear.x * multiplier
	twist_nav.angular.z = data.angular.z * multiplier

def main():
	rospy.init_node('rover_19_cmd_sub_serial')
	rospy.Subscriber("/cmd_vel", Twist, callbacknav)
	rospy.Subscriber("/rover_joy/cmd_vel", Twist, callbackcmd)

	
	
	way_left="0"
	left_wheelString="000"
	way_right="000"
	right_wheelString="000"
	way_left = 0
	way_right = 0
	all_wheels_msg = ""
	robotic_arm_msg = "1000656515656552011665320002"
	science_msg = "3212321526541653"

	left_wheel = 0 #left wheel
	right_wheel = 0 #right wheel
	
	rate = rospy.Rate(20)

	while not rospy.is_shutdown():

		if(twist_cmd.linear.x != 0.0 or twist_cmd.angular.z != 0.0):
			twist = twist_cmd


		else:
			twist = twist_nav
		

		if twist.linear.x >= 0:
				
			left_wheel = twist.linear.x -  twist.angular.z
			#left_wheel = twist.linear.x -  (twist.linear.x/2) 
			right_wheel = twist.linear.x + twist.angular.z
			#right_wheel = twist.linear.x + (twist.linear.x/2)


		elif twist.linear.x < 0: 

			left_wheel = twist.linear.x + twist.angular.z
			#left_wheel = twist.linear.x + (twist.linear.x/2)
			right_wheel = twist.linear.x - twist.angular.z
			#right_wheel = twist.linear.x - (twist.linear.x/2) 

		#eps = 1e-8
		#omer_hayyam = sqrt(left_wheel**2 + right_wheel**2) + eps
		#left_wheel = int(left_wheel * multiplier / omer_hayyam)
		#right_wheel = int(right_wheel * multiplier / omer_hayyam)

		if(left_wheel<0):
			way_left =1
			left_wheel *= -1
		else:
			way_left =0
		if(right_wheel<0):
			way_right =1
			right_wheel *= -1
		else:
			way_right=0


		print("left: "+str(left_wheel)+ " right: "+ str(right_wheel))
		
		if abs(left_wheel) < 10:
			left_wheelString = "00" + str(int(left_wheel))
			
		elif abs(left_wheel) < 100:
			left_wheelString = "0" + str(int(left_wheel))

		elif abs(left_wheel) > 100:
			left_wheelString = str(int(left_wheel))
			

		if abs(right_wheel) < 10:
			right_wheelString = "00" + str(int(right_wheel))
			
		elif abs(right_wheel) < 100:
			right_wheelString = "0" + str(int(right_wheel))

		elif abs(right_wheel) > 100:
			right_wheelString = str(int(right_wheel))
		
		if(left_wheel < 90 and left_wheel != 0):
			left_wheelString = "090"

		if(right_wheel < 90 and right_wheel != 0):
			right_wheelString = "090"

		if(left_wheel > 160):
			left_wheelString = "160"

		if(right_wheel > 160):
			right_wheelString = "160"
		


		all_wheels_msg = str(way_left) + str(left_wheelString) + str(way_right) + str(right_wheelString)
		print("S"+ all_wheels_msg+ robotic_arm_msg + science_msg +"100"+"F")
		pub.publish("S"+ all_wheels_msg + robotic_arm_msg + science_msg +"100"+"F")
		rate.sleep()

if __name__ == '__main__':
	main()
