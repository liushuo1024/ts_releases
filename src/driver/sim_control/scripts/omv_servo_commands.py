#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
import math
from common.msg import servo_omv_cmd

pub_servo_angle_cmd = rospy.Publisher(
    '/sim_robot/servo_angle_controller/command', Float64, queue_size=1)
pub_servo_velocity_cmd = rospy.Publisher(
    '/sim_robot/servo_velocity_controller/command', Float64, queue_size=1)

pub_right_servo_angle_cmd = rospy.Publisher(
    '/sim_robot/right_servo_angle_controller/command', Float64, queue_size=1)

pub_left_servo_angle_cmd = rospy.Publisher(
    '/sim_robot/left_servo_angle_controller/command', Float64, queue_size=1)

time_recv_last = []


def set_throttle_steer(data):
    global pub_servo_angle_cmd, pub_servo_velocity_cmd, time_recv_last, pub_right_servo_angle_cmd, pub_left_servo_angle_cmd

    servo_angle_cmd = Float64()
    servo_angle_cmd.data = data.sc_theta / 180 * 3.1415926

    servo_velocity_cmd = Float64()
    servo_velocity_cmd.data = data.sc_vel / 60 * 2*3.1415926

    left_servo_angle_cmd = Float64()
    left_servo_angle_cmd.data = data.sc_left_theta / 180 * 3.1415926

    right_servo_angle_cmd = Float64()
    right_servo_angle_cmd.data = data.sc_right_theta / 180 * 3.1415926

    pub_servo_angle_cmd.publish(servo_angle_cmd)
    pub_servo_velocity_cmd.publish(servo_velocity_cmd)
    pub_right_servo_angle_cmd.publish(right_servo_angle_cmd)
    pub_left_servo_angle_cmd.publish(left_servo_angle_cmd)
    time_recv_last = rospy.Time.now()


def pub_zero_vel():

    global pub_servo_angle_cmd, pub_servo_velocity_cmd, pub_right_servo_angle_cmd, pub_left_servo_angle_cmd
    servo_angle_cmd = Float64()
    servo_angle_cmd.data = 0

    servo_velocity_cmd = Float64()
    servo_velocity_cmd.data = 0

    pub_servo_angle_cmd.publish(servo_angle_cmd)
    pub_servo_velocity_cmd.publish(servo_velocity_cmd)

    pub_right_servo_angle_cmd.publish(servo_angle_cmd)
    pub_left_servo_angle_cmd.publish(servo_angle_cmd)


def time_callback(event):
    global time_recv_last
    # print("time:")
    # print(rospy.Time.now())
    # print(time_recv_last)
    # print(rospy.Time.now() - time_recv_last)
    if(rospy.Time.now() - time_recv_last > rospy.Duration(0.5)):
        pub_zero_vel()


def servo_commands():

    global time_recv_last

    rospy.init_node('servo_commands', anonymous=True)

    time_recv_last = rospy.Time.now()
    print("time:")
    print(rospy.Time.now())

    time.sleep(1.0)
    rospy.Subscriber("/servo_cmd", servo_omv_cmd, set_throttle_steer)

    rospy.Timer(rospy.Duration(0.1), time_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
