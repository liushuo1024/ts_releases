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
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "common/remote_ctrl_mode.h"
#include "amr_msgs/move_cmd.h"
#include "amr_msgs/move_feedback.h"
using namespace Eigen;
template <typename T>
T clamp(const T& value, const T& min_val, const T& max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
class Dual
{
public:
    Dual(ros::NodeHandle &n, ros::NodeHandle &pn);
    ~Dual();

    double regular_theta(double theta);
    void recv_omv_servo_encoder(const common::omv_servo_encoder::ConstPtr &msg);
    void recv_vehicle_cmd(const geometry_msgs::Twist::ConstPtr &msg);
    void MovingTaskFeedback_callback(const common::MovingTaskFeedback::ConstPtr &msg);
    int cal_theta(Vector2d start_p, Vector2d end_p, double &theta);
    int recover_start_y(const Vector2d& end_p, double theta, double& start_y);
    void inverse_kinematics(double vx, double vy, double omega, 
                                const Eigen::Vector2d& wheel_pos,
                                double& steer_angle, double& wheel_speed);
    void cal_odom(double offset_distance_wheel_x, 
    double offset_distance_wheel_y, double v_T,
    double theta_2odom,
    double &v,
    double &omega);
    void cal_cmd_vel(double offset_distance_wheel_x, 
        double offset_distance_wheel_y, 
        double v,double omega,
        double theta_2odom,
        double &v_T);
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
    std::string config_file_path;

    double lidar_global_x;
    double lidar_global_y;

    double sum_theta;
    double sum_x;
    double sum_y;

    double base_to_lidar_x;
    double base_to_lidar_y;
    double base_to_lidar_yaw;
    int omv_moving_state;
    int moving_mode_ = 0;
    bool omv_calibrate;



    tf::TransformBroadcaster odom_bd;
    ros::Time current_time;
    ros::Time last_time;
    ros::Publisher ac_cmd_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher auto_calibrate_odom_pub_;
    ros::Publisher remote_ctrl_pub;
    ros::Publisher move_feedback_pub_;
    ros::Subscriber servo_encoder_sub_;
    ros::Subscriber vehicle_cmd_sub_;
    ros::Subscriber MovingTaskFeedback_sub_;
    ros::Subscriber move_cmd_sub_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;


    Vector2d left_wheel_p;
    Vector2d right_wheel_p;
    Vector2d steering_wheel_p;
    double max_steer_angle_, max_wheel_speed_;
    // Eigen::Vector2d left_wheel_pos_;   // 左舵轮坐标 (x, y)
    // Eigen::Vector2d right_wheel_pos_;  // 右舵轮坐标 (x, y)
    bool protection_ = false;
};
