#!/usr/bin/env python

### STATE MACHINE for European Rover Challenge'21 Navigation Task + Probing Task + Science Task (MEGA TASK!)

import rospy
from rospy.core import logerr
import smach
import smach_ros
import math
import actionlib
from std_msgs.msg import String, Int16
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from deneme.msg import Coordinate
from darknet_ros_msgs.srv import srvCoordinate

client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
nav_counter = 0
navigation_mode = 'Forward'
goal = []
for i in range(3):
    goal.append([])
for i in range(3):
    goal[i].append(float(input('Enter {itr}. X:'.format(itr=i+1))))
    goal[i].append(float(input('Enter {itr}. Y:'.format(itr=i+1))))
print(goal)

class SensorCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['FAIL','REPEAT'])
        
        
        self.imu_working = False
        self.encoder_working = False

        self.imu_topic = '/imu/data'
        self.encoder_topic = 'odometry/wheel'
        
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        rospy.Subscriber(self.encoder_topic, Odometry, self.encoder_callback)

    def imu_callback(self,data):
        self.ImuOrientation = [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
        if self.ImuOrientation[0] != '' and self.ImuOrientation[1] != '':
            self.imu_working = True
        else:
            self.imu_working = False

    def encoder_callback(self,data):
        self.EncoderPose = [data.pose.pose.position.x, data.pose.pose.position.y,data.pose.pose.position.z]
    	if self.EncoderPose[0] != '' and self.EncoderPose[1] != '':
            self.encoderWorking = True
    	else:
            self.encoderWorking = False    

    def execute(self,userdata):

        rospy.sleep(1)
        rospy.loginfo('Checking sensors...')

        if self.imu_working == True and self.encoder_working == True:
            return 'REPEAT'
        else:
            return 'FAIL'

class Initialise(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['SUCCESS'])
	rospy.sleep(5)        
	rospy.loginfo('on Initialise state')
        
        
    def execute(self,userdata):
        
        return 'SUCCESS'

class SystemControl(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['REPEAT','SUCCESS'])
        self.move_base_flag = False
        self.localization_flag = False
        self.image_processing_flag = True
        self.odom_flag = False


    def execute(self, userdata):
        rospy.loginfo('on System Control state')

        X = rospy.get_published_topics()

        for i in range(len(X) - 1):
            
            if '/move_base/goal' in X[i]:
                self.move_base_flag = True
            
            if '/odometry/filtered' in X[i]:
                self.localization_flag = True

            if '/imu/data' in X[i]:
                self.imu_flag = True

            if '/odometry/wheel' in X[i]:
                self.odom_flag = True

            if '/visualization_marker' in X[i]:
                self.image_processing_flag = True
                
        if self.odom_flag == True and self.move_base_flag == True and self.imu_flag == True and self.localization_flag == True:
            self.localization_flag = True
        print(self.localization_flag)
        print(self.move_base_flag)

        if self.localization_flag == True and self.move_base_flag == True: 
        #and self.image_processing_flag == True:
            return 'SUCCESS'
        else:
            return 'REPEAT'

        
class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['DEPLOY','APPROACH','FAIL'])
        

    def execute(self,userdata):
        
        rospy.loginfo('on Navigation state')
        global nav_counter    
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'odom'
        nav_goal.target_pose.header.stamp = rospy.Time.now()
        global navigation_mode


        if navigation_mode == 'Forward':
            if nav_counter < 3 and nav_counter > -1:
                nav_goal.target_pose.pose.position.x = goal[nav_counter][0]
                nav_goal.target_pose.pose.position.y = goal[nav_counter][1]
                nav_goal.target_pose.pose.orientation.x = 0.0
                nav_goal.target_pose.pose.orientation.y = 0.0
                nav_goal.target_pose.pose.orientation.z = 0.0
                nav_goal.target_pose.pose.orientation.w = 1.0

                nav_counter = nav_counter + 1
                print('nav_counter:',nav_counter)
                client.send_goal(nav_goal)
                self.wait = client.wait_for_result()
                return 'DEPLOY'

            else:
                logerr('Wrong Nav Goal Index {nav_ct} !!!'.format(nav_ct = nav_counter))
                return 'FAIL'

        elif navigation_mode == 'Backward':
            if nav_counter < 6 and nav_counter > 2:
                nav_goal.target_pose.pose.position.x = goal[5 - nav_counter][0]
                nav_goal.target_pose.pose.position.y = goal[5 - nav_counter][1]
                nav_goal.target_pose.pose.orientation.x = 0.0
                nav_goal.target_pose.pose.orientation.y = 0.0
                nav_goal.target_pose.pose.orientation.z = 0.0
                nav_goal.target_pose.pose.orientation.w = 1.0

                nav_counter = nav_counter + 1
                print('nav_counter:',nav_counter)
                client.send_goal(nav_goal)
                self.wait = client.wait_for_result()
                return 'APPROACH'
            else:
                logerr('Wrong Nav Goal Index {nav_ct} !!!'.format(nav_ct = nav_counter))
                return 'FAIL'
    

class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['PICKUP','FAIL'])
        self.pickup_flag = False
        self.approach_goal = MoveBaseGoal()
        self.detection_flag = False
        self.frame_flag = 0
        
        camera_angle = 120
        self.camera_angle_rad = math.pi*camera_angle/180
        self.vehicle_yaw = 0
        self.counter = 0

        rospy.Subscriber('/odometry/filtered',Odometry,self.odom_cb)
        rospy.Subscriber('/probe/detection',String,self.detection_cb)
        rospy.Subscriber('/probe/frame',String,self.frame_check_cb)
        rospy.wait_for_service('basan_xyz_coordinates')
        self.xyz = rospy.ServiceProxy('basan_xyz',srvCoordinate)
        

        
    def frame_check_cb(self,data):
        self.frame_flag = int(data.data)

    def detection_cb(self,data):
        if data.data  == 'No Probe':
                self.detection_flag = False
        elif data.data == 'probe detected':
                self.detection_flag = True
        else:
                print('helomelo')
    
    def coordinate_assign(self):
        self.approach_goal.target_pose.header.frame_id = 'odom'
        self.approach_goal.target_pose.pose.position.x = self.coordinates.x
        self.approach_goal.target_pose.pose.position.y = self.coordinates.y
        self.approach_goal.target_pose.pose.orientation.x = 0.0
        self.approach_goal.target_pose.pose.orientation.y = 0.0
        self.approach_goal.target_pose.pose.orientation.z = 0.0
        self.approach_goal.target_pose.pose.orientation.w = 1.0
        self.detection_flag = True

    def odom_cb(self,data):
        vehicle_orientation = [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        (vehicle_roll,vehicle_pitch,self.vehicle_yaw) = euler_from_quaternion(vehicle_orientation)
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        

    def turn_ccw(self):
        print('turn_ccw')
        self.vehicle_yaw = self.vehicle_yaw - 1.25663706144
        goal = MoveBaseGoal()
        quar = quaternion_from_euler(0,0,self.vehicle_yaw) 
        print(quar[0],quar[1],quar[2],quar[3])
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.orientation.x = quar[0]
        goal.target_pose.pose.orientation.y = quar[1]
        goal.target_pose.pose.orientation.z = quar[2]
        goal.target_pose.pose.orientation.w = quar[3]
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.position.z = self.z
        print(rospy.Time.now())
        client.send_goal(goal)
        self.wait = client.wait_for_result()

    ############################### emir parmagi ############################

    def align_probe(self,direction,angle):
        print('align_probe')
        angle = angle*math.pi*direction
        self.vehicle_yaw = self.vehicle_yaw + angle
        goal = MoveBaseGoal()
        quar = quaternion_from_euler(0,0,self.vehicle_yaw) 
        print(quar[0],quar[1],quar[2],quar[3])
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.orientation.x = quar[0]
        goal.target_pose.pose.orientation.y = quar[1]
        goal.target_pose.pose.orientation.z = quar[2]
        goal.target_pose.pose.orientation.w = quar[3]
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.position.z = self.z
        print(rospy.Time.now())
        client.send_goal(goal)
        self.wait = client.wait_for_result()




    #########################################################################
        

    def execute(self,userdata):
        rospy.loginfo('Entered Approach Mode')
        rospy.sleep(1)
        
        while (not self.detection_flag):
            rospy.sleep(1)
            self.turn_ccw()

        ###################################################
        '''while (self.frame_flag != 0):
            rospy.sleep(1)
            self.align_probe(self.frame_flag,10)'''
        self.coordinates = self.xyz()
        self.coordinate_assign()
        ###################################################
        client.send_goal(self.approach_goal)
        self.detection_flag = False
        rospy.loginfo('Pickup')
        return 'PICKUP'
        
        

class DeployProbe(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['FAIL', 'SUCCESS'])
        rospy.Subscriber('/probe/deploy',String,self.deploy_cb)
        self.deploy_check = False

    def execute(self,userdata):
        rospy.loginfo('on DeployProbe state')
        while(not self.deploy_check):
            rospy.sleep(1)
        self.deploy_check = False
        return 'SUCCESS'
        

    def deploy_cb(self,data):
        if data.data == 'F':
            self.deploy_check = True 
        else:
            rospy.logerr("Deploy Check Flag returned the invalid value: %s", data.data)     

    

class PickupProbe(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['FAIL', 'SUCCESS'])
        self.pickup_check = False
        rospy.Subscriber('/probe/pickup',String,self.pickup_cb)

    def execute(self,userdata):
        rospy.loginfo('on PickupProbe state')
        while(not self.pickup_check):
            rospy.sleep(1)
        self.pickup_check = False
        return 'SUCCESS'
        

    def pickup_cb(self,data):
        if data.data == 'F':
            self.pickup_check = True 
        else:
            rospy.logerr("PickUp Check Flag returned the invalid value: %s", data.data) 

class Success(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['REINITIALISE'])
        self.over = False
        

    def execute(self,userdata):
        rospy.loginfo('Success')
        global navigation_mode
        if nav_counter == 3 and navigation_mode == 'Forward':
            navigation_mode = 'Backward'
        if nav_counter == 6:
            self.over = True
        if self.over:
            rospy.sleep(10)
        
        return 'REINITIALISE'

class Error(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['REINITIALISE'])
    
    def execute(self,userdata):
        rospy.loginfo('ERROR FOUND')
        return 'REINITIALISE'

def CreateStateMachine():

    rospy.init_node("state_machine")
    

    sm_rover = smach.StateMachine(outcomes = ['DEAD'])

    with sm_rover:

        smach.StateMachine.add('INITIALISE',Initialise(),transitions={'SUCCESS': 'SYSTEM CONTROL'})
        
        smach.StateMachine.add('SENSOR CHECK',SensorCheck(),transitions={'REPEAT': 'SENSOR CHECK','FAIL': 'INITIALISE'})

        smach.StateMachine.add('SYSTEM CONTROL',SystemControl(),transitions={'SUCCESS': 'NAVIGATION','REPEAT': 'INITIALISE'})

        smach.StateMachine.add('NAVIGATION',Navigation(),transitions={'APPROACH':'APPROACH','DEPLOY':'DEPLOY','FAIL':'ERROR'})

        smach.StateMachine.add('APPROACH',Approach(),transitions={'PICKUP':'PICKUP','FAIL':'ERROR'})

        smach.StateMachine.add('DEPLOY',DeployProbe(),transitions={'SUCCESS':'SUCCESS','FAIL':'ERROR'})

        smach.StateMachine.add('PICKUP',PickupProbe(),transitions={'SUCCESS':'SUCCESS','FAIL':'ERROR'})

        smach.StateMachine.add('SUCCESS',Success(),transitions={'REINITIALISE':'INITIALISE'})

        smach.StateMachine.add('ERROR',Error(),transitions={'REINITIALISE':'INITIALISE'})


    sis = smach_ros.IntrospectionServer('rover_21_state_machine', sm_rover, '/ROVER_SM_ROOT')
    sis.start()

    outcome = sm_rover.execute()
    sis.stop()

def main():
    #Init,pubs and subs
    

    while not rospy.is_shutdown():
        CreateStateMachine()
        rospy.loginfo( "Created State Machine..")



if __name__ == '__main__':
    main()
    rospy.spin()    
