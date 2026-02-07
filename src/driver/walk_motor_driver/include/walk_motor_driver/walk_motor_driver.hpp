#ifndef WALK_MOTOR_DRIVER_HPP_
#define WALK_MOTOR_DRIVER_HPP_

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <common/omv_servo_cmd.h>
#include <common/omv_servo_encoder.h>
#include <map>
#include <vector>

namespace walk_motor_driver
{

// 电机类型
enum MotorType
{
  MOTOR_TYPE_DRIVE = 0,  // 行走电机
  MOTOR_TYPE_STEER = 1   // 转向舵轮
};

// 电机控制模式
enum ControlMode
{
  MODE_VELOCITY = 0x00,   // 速度控制模式
  MODE_TORQUE = 0x01,     // 转矩控制模式
  MODE_POSITION = 0x02,   // 位置控制模式
  MODE_PROFILE_VELOCITY = 0x03  // 速度模式（用于位置控制时的速度设置）
};

// 电机配置
struct MotorConfig
{
  int motor_id;           // 电机ID
  MotorType type;         // 电机类型
  uint32_t can_address;   // CAN地址
  std::string topic_prefix; // 话题前缀
};

class WalkMotorDriver
{
public:
  WalkMotorDriver(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~WalkMotorDriver();

  void run();

private:
  // 初始化电机
  void initMotors();
  void enableAllMotors();
  // 控制指令回调函数
  void enableCallback(const std_msgs::Empty::ConstPtr& msg);
  void disableCallback(const std_msgs::Empty::ConstPtr& msg);
  void omvServoCmdCallback(const common::omv_servo_cmd::ConstPtr& msg);

  // 发送速度命令到行走电机
  void sendDriveVelocity(double left_vel, double right_vel);

  // 发送位置命令到转向电机
  void sendSteerPosition(double left_theta, double right_theta);

  // 发送CAN帧
  void sendCanFrame(uint32_t can_address, uint8_t cmd_type, const uint8_t* data, uint8_t dlc);

  // 将数值转换为4字节数据
  void int32ToBytes(int32_t value, uint8_t* bytes);
  void uint32ToBytes(uint32_t value, uint8_t* bytes);

  void canCallback(const can_msgs::Frame::ConstPtr& msg);

  // 定时查询回调
  void queryCallback(const ros::TimerEvent& event);

  // 发送查询命令
  void queryMotorVelocity();
  void querySteerPosition();

  // 处理查询响应
  void processVelocityResponse(uint32_t can_address, int32_t velocity);
  void processPositionResponse(uint32_t can_address, uint16_t position);

  // 发布编码器反馈
  void publishEncoderFeedback();


  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber enable_sub_;
  ros::Subscriber disable_sub_;
  ros::Subscriber velocity_sub_;
  ros::Subscriber profile_velocity_sub_;
  ros::Subscriber torque_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber omv_servo_cmd_sub_;

  ros::Subscriber can_sub_;

  ros::Publisher encoder_pub_;
  ros::Publisher can_pub_;

  ros::Timer query_timer_;

  // 电机配置列表
  std::vector<MotorConfig> motors_;

  // 编码器反馈数据缓存
  struct EncoderData {
    int32_t left_drive_vel;   // 左行走电机转速 (RPM)
    int32_t right_drive_vel;  // 右行走电机转速 (RPM)
    uint16_t left_steer_pos;  // 左转向电机位置 (0-9999)
    uint16_t right_steer_pos; // 右转向电机位置 (0-9999)
    bool left_drive_valid;
    bool right_drive_valid;
    bool left_steer_valid;
    bool right_steer_valid;
  } encoder_data_;

  // 控制参数
  int max_motor_rpm_;              // 最大速度 (+/-10000)
  double position_scale_;         // 位置转换比例 (rad -> CAN值)
  double velocity_rated_speed_;
  double steer_rated_speed_;
  double steer_ratio_;

  double left_zero_point_ = 0,right_zero_point_ = 0;
  bool left_driver_neg_ = false,right_driver_neg_ = false;
  bool left_steer_neg_ = false,right_steer_neg_ = false;
};

}  // namespace walk_motor_driver

#endif  // WALK_MOTOR_DRIVER_HPP_
