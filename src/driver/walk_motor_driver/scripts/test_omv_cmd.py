#!/usr/bin/env python
"""
测试omv_servo_cmd消息发送
"""

import rospy
from common.msg import OmvServoCmd
from std_msgs.msg import Empty
import math


def test_omv_cmd():
    rospy.init_node('omv_cmd_test')

    # 发布者
    omv_cmd_pub = rospy.Publisher('/omv_servo_cmd', OmvServoCmd, queue_size=10)
    enable_pub = rospy.Publisher('/motors/enable', Empty, queue_size=10)

    rospy.loginfo('OMV命令测试节点已启动')
    rospy.sleep(1.0)

    # 使能电机
    rospy.loginfo('使能所有电机...')
    enable_pub.publish(Empty())
    rospy.sleep(1.0)

    # 测试1: 直线前进
    rospy.loginfo('=== 测试1: 直线前进 ===')
    msg = OmvServoCmd()
    msg.header.stamp = rospy.Time.now()
    msg.sc_vel = 1.0
    msg.sc_main_theta = 0.0
    msg.sc_left_vel = 1.0
    msg.sc_right_vel = 1.0
    msg.sc_left_theta = 0.0
    msg.sc_right_theta = 0.0
    omv_cmd_pub.publish(msg)
    rospy.sleep(3.0)

    # 测试2: 左转
    rospy.loginfo('=== 测试2: 左转 (偏转30度) ===')
    msg.header.stamp = rospy.Time.now()
    msg.sc_vel = 0.5
    msg.sc_main_theta = math.radians(30)
    msg.sc_left_vel = 0.5
    msg.sc_right_vel = 0.5
    msg.sc_left_theta = math.radians(30)
    msg.sc_right_theta = math.radians(30)
    omv_cmd_pub.publish(msg)
    rospy.sleep(3.0)

    # 测试3: 右转
    rospy.loginfo('=== 测试3: 右转 (偏转-30度) ===')
    msg.header.stamp = rospy.Time.now()
    msg.sc_vel = 0.5
    msg.sc_main_theta = math.radians(-30)
    msg.sc_left_vel = 0.5
    msg.sc_right_vel = 0.5
    msg.sc_left_theta = math.radians(-30)
    msg.sc_right_theta = math.radians(-30)
    omv_cmd_pub.publish(msg)
    rospy.sleep(3.0)

    # 测试4: 停止
    rospy.loginfo('=== 测试4: 停止 ===')
    msg.header.stamp = rospy.Time.now()
    msg.sc_vel = 0.0
    msg.sc_main_theta = 0.0
    msg.sc_left_vel = 0.0
    msg.sc_right_vel = 0.0
    msg.sc_left_theta = 0.0
    msg.sc_right_theta = 0.0
    omv_cmd_pub.publish(msg)
    rospy.sleep(1.0)

    rospy.loginfo('测试完成！')


if __name__ == '__main__':
    try:
        test_omv_cmd()
    except rospy.ROSInterruptException:
        rospy.loginfo('用户中断测试')
