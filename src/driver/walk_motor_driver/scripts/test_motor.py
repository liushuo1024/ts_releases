#!/usr/bin/env python
"""
行走电机测试脚本
测试使能、速度控制、失能等功能
"""

import rospy
from std_msgs.msg import Empty, Int32
import time


def test_motors():
    rospy.init_node('motor_tester')

    # 左电机发布者
    left_enable_pub = rospy.Publisher('/left_motor/motor/enable', Empty, queue_size=10)
    left_disable_pub = rospy.Publisher('/left_motor/motor/disable', Empty, queue_size=10)
    left_vel_pub = rospy.Publisher('/left_motor/motor/velocity_cmd', Int32, queue_size=10)

    # 右电机发布者
    right_enable_pub = rospy.Publisher('/right_motor/motor/enable', Empty, queue_size=10)
    right_disable_pub = rospy.Publisher('/right_motor/motor/disable', Empty, queue_size=10)
    right_vel_pub = rospy.Publisher('/right_motor/motor/velocity_cmd', Int32, queue_size=10)

    rospy.loginfo('电机测试节点已启动')
    rospy.sleep(1.0)  # 等待发布者连接

    def enable_motors():
        """使能所有电机"""
        rospy.loginfo('使能电机...')
        left_enable_pub.publish(Empty())
        right_enable_pub.publish(Empty())
        time.sleep(0.5)

    def disable_motors():
        """失能所有电机"""
        rospy.loginfo('失能电机...')
        left_disable_pub.publish(Empty())
        right_disable_pub.publish(Empty())
        time.sleep(0.5)

    def set_velocity(left_vel, right_vel):
        """设置电机速度"""
        msg_left = Int32()
        msg_left.data = left_vel
        left_vel_pub.publish(msg_left)

        msg_right = Int32()
        msg_right.data = right_vel
        right_vel_pub.publish(msg_right)

        rospy.loginfo('设置速度: 左轮=%d, 右轮=%d', left_vel, right_vel)

    try:
        enable_motors()
        time.sleep(1)

        # 测试前进
        rospy.loginfo('--- 测试前进 ---')
        set_velocity(3000, 3000)
        time.sleep(2)

        # 停止
        rospy.loginfo('--- 测试停止 ---')
        set_velocity(0, 0)
        time.sleep(1)

        # 测试后退
        rospy.loginfo('--- 测试后退 ---')
        set_velocity(-3000, -3000)
        time.sleep(2)

        # 停止
        set_velocity(0, 0)
        time.sleep(1)

        # 测试左转
        rospy.loginfo('--- 测试左转 ---')
        set_velocity(2000, 4000)
        time.sleep(2)

        # 停止
        set_velocity(0, 0)
        time.sleep(1)

        # 测试右转
        rospy.loginfo('--- 测试右转 ---')
        set_velocity(4000, 2000)
        time.sleep(2)

        # 停止
        set_velocity(0, 0)
        time.sleep(1)

        disable_motors()

        rospy.loginfo('测试完成！')

    except Exception as e:
        rospy.logerr('测试过程中出错: %s', e)
        disable_motors()


if __name__ == '__main__':
    try:
        test_motors()
    except rospy.ROSInterruptException:
        rospy.loginfo('用户中断测试')
