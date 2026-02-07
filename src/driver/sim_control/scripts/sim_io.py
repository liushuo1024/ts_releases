#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from common.msg import peripheral_uart
from std_msgs.msg import Bool
# 设置一个阈值距离
threshold_distance = 0.08  # 单位：米
drop_distance = 0.2
peripheral_uart_pub = None
pub_msg = peripheral_uart()
def laser_scan_callback(msg):
    global threshold_distance
    global pub_msg
    """
    当收到新的LaserScan消息时，这个函数会被调用。
    它会遍历每一条扫描线，如果某条线的距离超过预设的阈值，
    则输出相关信息。
    """
    limit_switch = 0
    for i, distance in enumerate(msg.ranges):
        if distance < threshold_distance:
            limit_switch = 3
            threshold_distance = 0.12
            break
        if distance > drop_distance:
            threshold_distance = 0.08
    # if limit_switch != 0:
    #     print("limit switch triggered")
    #     pub_msg = peripheral_uart()
    #     pub_msg.limit_switch = limit_switch
    #     peripheral_uart_pub.publish(pub_msg)
    print("limit switch triggered")

    pub_msg.limit_switch = limit_switch
    peripheral_uart_pub.publish(pub_msg)


def io_callback(msg):
    global pub_msg
    if msg.data:
        pub_msg.m_iManualModeFlag = 1
    else:
        pub_msg.m_iManualModeFlag = 0
def main():
    global peripheral_uart_pub
    # 初始化ROS节点
    rospy.init_node('laser_subscriber', anonymous=True)

    peripheral_uart_pub = rospy.Publisher('/peripheral_devs_state', peripheral_uart,queue_size=10)

    # 订阅激光雷达的话题
    rospy.Subscriber("/inplace", LaserScan, laser_scan_callback)
    rospy.Subscriber("/manual", Bool, io_callback)
    # 阻塞调用，让节点持续运行直到节点关闭
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass