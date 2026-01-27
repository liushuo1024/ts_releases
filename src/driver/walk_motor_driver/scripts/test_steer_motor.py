#!/usr/bin/env python
"""
转向电机测试脚本
示例：命令转向电机以500转/分钟的速度逆时针旋转72度
72度 = 72 * (10000/360) = 2000 = 0x7D0
"""

import rospy
from std_msgs.msg import Empty, Int32, Int64
import time


def test_steer():
    rospy.init_node('steer_motor_test')

    # 订阅者
    enable_pub = rospy.Publisher('/motors/enable', Empty, queue_size=10)
    disable_pub = rospy.Publisher('/motors/disable', Empty, queue_size=10)
    profile_vel_pub = rospy.Publisher('/motors/profile_velocity_cmd', Int32, queue_size=10)
    position_pub = rospy.Publisher('/motors/position_cmd', Int64, queue_size=10)

    rospy.loginfo('转向电机测试节点已启动')
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

    def set_profile_velocity(velocity):
        """设置速度模式 (用于位置控制时的速度)"""
        msg = Int32()
        msg.data = velocity
        profile_vel_pub.publish(msg)
        rospy.loginfo('设置速度模式: %d rpm', velocity)
        time.sleep(0.1)

    def set_position(angle_deg):
        """
        设置转向角度
        angle_deg: 角度 (度)
        10000/圈 = 10000/360度
        """
        # 将角度转换为位置值
        position = int(angle_deg * 10000.0 / 360.0)
        msg = Int64()
        msg.data = position
        position_pub.publish(msg)
        rospy.loginfo('设置位置: %.1f度 -> %d (0x%04X)', angle_deg, position, position)
        time.sleep(0.1)

    try:
        # 使能电机
        enable_motors()

        # 示例：命令转向电机以500转/分钟的速度逆时针旋转72度
        # (a) 使能: 23 0D 20 01 00 00 00 00
        # (b) 运行速度: 23 03 20 01 00 00 01 F4 (500 rpm = 0x01F4)
        # (c) 位置控制: 23 02 20 01 00 00 07 D0 (72度 = 2000 = 0x07D0)

        rospy.loginfo('=== 测试: 逆时针旋转72度，速度500 rpm ===')

        # 设置速度模式: 500 rpm
        set_profile_velocity(500)
        time.sleep(0.2)

        # 位置控制: 72度
        set_position(72.0)
        time.sleep(3.0)

        # 返回到0度
        rospy.loginfo('返回到0度')
        set_profile_velocity(500)
        set_position(0.0)
        time.sleep(3.0)

        # 测试顺时针旋转
        rospy.loginfo('=== 测试: 顺时针旋转45度 ===')
        set_profile_velocity(500)
        set_position(-45.0)
        time.sleep(3.0)

        # 返回到0度
        rospy.loginfo('返回到0度')
        set_profile_velocity(500)
        set_position(0.0)
        time.sleep(3.0)

        disable_motors()

        rospy.loginfo('测试完成！')

    except Exception as e:
        rospy.logerr('测试过程中出错: %s', e)
        disable_motors()


if __name__ == '__main__':
    try:
        test_steer()
    except rospy.ROSInterruptException:
        rospy.loginfo('用户中断测试')
