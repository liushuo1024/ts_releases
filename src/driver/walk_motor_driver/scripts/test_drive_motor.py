#!/usr/bin/env python
"""
行走电机测试脚本
测试行走电机的速度控制
"""

import rospy
from std_msgs.msg import Empty, Int32
import time


def test_drive():
    rospy.init_node('drive_motor_test')

    # 订阅者
    enable_pub = rospy.Publisher('/motors/enable', Empty, queue_size=10)
    disable_pub = rospy.Publisher('/motors/disable', Empty, queue_size=10)
    velocity_pub = rospy.Publisher('/motors/velocity_cmd', Int32, queue_size=10)

    rospy.loginfo('行走电机测试节点已启动')
    rospy.sleep(1.0)  # 等待发布者连接

    def enable_motors():
        """使能所有电机"""
        rospy.loginfo('使能所有电机...')
        enable_pub.publish(Empty())
        time.sleep(0.5)

    def disable_motors():
        """失能所有电机"""
        rospy.loginfo('失能所有电机...')
        disable_pub.publish(Empty())
        time.sleep(0.5)

    def set_velocity(velocity):
        """设置速度"""
        msg = Int32()
        msg.data = velocity
        velocity_pub.publish(msg)
        rospy.loginfo('设置速度: %d', velocity)
        time.sleep(0.1)

    try:
        enable_motors()

        # 测试前进
        rospy.loginfo('=== 测试前进 ===')
        set_velocity(3000)
        time.sleep(2.0)

        # 停止
        rospy.loginfo('停止')
        set_velocity(0)
        time.sleep(1.0)

        # 测试后退
        rospy.loginfo('=== 测试后退 ===')
        set_velocity(-3000)
        time.sleep(2.0)

        # 停止
        set_velocity(0)
        time.sleep(1.0)

        # 测试左转 (左轮慢，右轮快)
        rospy.loginfo('=== 测试左转 ===')
        # 注意: 这里的简化实现，实际应用中需要分别控制左右轮速度
        set_velocity(2000)
        time.sleep(2.0)

        # 停止
        set_velocity(0)
        time.sleep(1.0)

        # 测试右转
        rospy.loginfo('=== 测试右转 ===')
        set_velocity(4000)
        time.sleep(2.0)

        # 停止
        set_velocity(0)
        time.sleep(1.0)

        disable_motors()

        rospy.loginfo('测试完成！')

    except Exception as e:
        rospy.logerr('测试过程中出错: %s', e)
        disable_motors()


if __name__ == '__main__':
    try:
        test_drive()
    except rospy.ROSInterruptException:
        rospy.loginfo('用户中断测试')
