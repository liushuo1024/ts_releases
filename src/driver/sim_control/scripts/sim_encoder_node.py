#!/usr/bin/env python3
# -*- coding:utf-8 -*-

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros
import tf 
from control_msgs.msg import JointControllerState
from common.msg import servo_encoder
import random
 
def quaternion_multiply(quaternion1, quaternion0): 
    x0, y0, z0 ,w0= quaternion0 
    x1, y1, z1 ,w1= quaternion1 
    return np.array([
        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0, 
        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0, 
        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0, 
        -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0], dtype=np.float64) 


class OdometryNode:
    # Set publishers
    

    def __init__(self):
 
        self.servo_encode_pub = rospy.Publisher('/servo_encoder', servo_encoder,queue_size=1)

        # Set the update rate
        self.update_timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback) # 20hz


        # Set subscribers
        rospy.Subscriber('/sim_robot/servo_angle_controller/state', JointControllerState, self.sub_servo_angle)
        rospy.Subscriber('/sim_robot/servo_velocity_controller/state', JointControllerState, self.sub_servo_velocity)
        self.servo_angle = 0
        self.servo_velocity = 0
        self.last_recieved_stamp =  rospy.Time.now()
        self.k = 1.0
        self.d = 0.0
        self.theta_gauss = 0.1
        self.v_gauss = 0.01

    def sub_servo_angle(self, msg):
        self.last_recieved_stamp = msg.header.stamp
        self.servo_angle = msg.process_value


    def sub_servo_velocity(self, msg):
        self.servo_velocity = msg.process_value 
       

    def timer_callback(self, event):  
        msg = servo_encoder()
        msg.header.stamp = self.last_recieved_stamp
        msg.se_theta = self.servo_angle*180/3.1415926*self.k  # + self.d + random.gauss(0,self.theta_gauss)

        msg.se_vel = self.servo_velocity/2/3.1415926*60 * 21.96 # + random.gauss(0,self.v_gauss)
        self.servo_encode_pub.publish(msg)     

      

# Start the node
if __name__ == '__main__':
    rospy.init_node("sim_encoder_node")
    node = OdometryNode()
    rospy.spin()
