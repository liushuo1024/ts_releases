#!/usr/bin/env python3
import rospy
import time
import math
from common.msg import battery

time_recv_last = []
servo_encoder_angle = 0

def set_throttle_steer(data):
    global pub_servo_angle_cmd, pub_servo_velocity_cmd, time_recv_last

    servo_angle_cmd = Float64()
    servo_angle_cmd.data = data.sc_theta / 180 * 3.1415926

    servo_velocity_cmd = Float64()
    # 除了减速比
    servo_velocity_cmd.data = data.sc_vel / 60 * 2*3.1415926 / 21.96 
    # print(f"servo_angle_cmd:{servo_angle_cmd.data}")
    # print(f"servo_encoder_angle2:{servo_encoder_angle}")
    if math.fabs(servo_angle_cmd.data - servo_encoder_angle) > 0.1:
        print("wait servo")
        pub_servo_angle_cmd.publish(servo_angle_cmd)
        return
    pub_servo_angle_cmd.publish(servo_angle_cmd)
    pub_servo_velocity_cmd.publish(servo_velocity_cmd)
    time_recv_last = rospy.Time.now()


def pub_zero_vel():

    global pub_servo_angle_cmd, pub_servo_velocity_cmd
    servo_angle_cmd = Float64()
    servo_angle_cmd.data = 0

    servo_velocity_cmd = Float64()
    servo_velocity_cmd.data = 0

    pub_servo_angle_cmd.publish(servo_angle_cmd)
    pub_servo_velocity_cmd.publish(servo_velocity_cmd)


def time_callback(event):
    global time_recv_last
    # print("time:")
    # print(rospy.Time.now())
    # print(time_recv_last)
    # print(rospy.Time.now() - time_recv_last)
    if(rospy.Time.now() - time_recv_last > rospy.Duration(0.5)):
        pub_zero_vel()

def sub_servo_angle(msg):
    global servo_encoder_angle
    servo_encoder_angle = msg.process_value
    # print(f"servo_encoder_angle:{servo_encoder_angle}")
def servo_commands():

    global time_recv_last

    rospy.init_node('sim_bms', anonymous=True)
    pub_servo_angle_cmd = rospy.Publisher(
        '/battery_data', battery, queue_size=1)
    while not rospy.is_shutdown():
        data = battery()
        data.electricity = 100
        pub_servo_angle_cmd.publish(data)
        time.sleep(0.1)


if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
