#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <common/servo_cmd.h>
#include <common/servo_encoder.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_broadcaster.h"
#include "vehicle_kinematics/tinyxml2.h"
#include <string.h>
#include <math.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <vector>
#include "common/omv_servo_cmd.h"
#include "common/omv_servo_encoder.h"
#include <common/MovingTaskFeedback.h>
#include <sensor_msgs/Imu.h>
#include "amr_msgs/move_cmd.h"
class SingleServo
{
public:
    SingleServo(ros::NodeHandle &n, ros::NodeHandle &pn);
    ~SingleServo();
    int get_param(ros::NodeHandle& nh_priv);
    void recv_vehicle_cmd(const geometry_msgs::Twist::ConstPtr &msg);
    void recv_servo_encoder(const common::servo_encoder::ConstPtr &msg);
    void recv_omv_servo_encoder(const common::omv_servo_encoder::ConstPtr &msg);
    void MovingTaskFeedback_callback(const common::MovingTaskFeedback::ConstPtr &msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void recv_move_cmd(const amr_msgs::move_cmd::ConstPtr &msg);
    //舵轮模型相关参数
    double k_red_ratio;
    double radius_steering_wheel;
    double offset_distance_wheel_x;
    double offset_distance_wheel_y;

    //差分模型相关参数
    double wheel_track;
    double radius_virtual_left_wheel;
    double radius_virtual_right_wheel;

    // 差分模型固定参数
    double wheel_track_theory_const;
    double radius_virtual_left_wheel_const;
    double radius_virtual_right_wheel_const;

    //
    bool pub_lidar_tf;
    bool odom_first_flg;
    bool omv_equivalent;
    bool pub_odom;
    bool use_imu = false;
    std::string config_file_path;

    double lidar_global_x;
    double lidar_global_y;

    double sum_theta;
    double sum_x;
    double sum_y;
    double imu_angular_velocity_z_;

    double base_to_lidar_x;
    double base_to_lidar_y;
    double base_to_lidar_yaw;
    int omv_moving_state;
    bool omv_calibrate;

    tf::TransformBroadcaster odom_bd;
    ros::Time current_time;
    ros::Time last_time;
    ros::Publisher ac_cmd_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher auto_calibrate_odom_pub_;
    ros::Publisher move_feedback_pub_;
    
    ros::Subscriber servo_encoder_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber vehicle_cmd_sub_;
    ros::Subscriber move_cmd_sub_;
    ros::Subscriber MovingTaskFeedback_sub_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
};
