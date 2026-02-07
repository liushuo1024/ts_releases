#include "walk_motor_driver/walk_motor_driver.hpp"

namespace walk_motor_driver
{

WalkMotorDriver::WalkMotorDriver(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh), private_nh_(private_nh)
{
  // 获取参数
  private_nh_.param("max_motor_rpm", max_motor_rpm_, 10000);
  private_nh_.param("velocity_rated_speed", velocity_rated_speed_, 3000.0);
  private_nh_.param("steer_rated_speed", steer_rated_speed_, 3000.0);
  private_nh_.param("steer_ratio", steer_ratio_, 40.0);
  private_nh_.param("position_scale", position_scale_, 10000.0 / (2.0 * M_PI));





  std::cout << "scale1: " << (180.0/M_PI)*(10000/360.0) << std::endl;
  std::cout << "scale2: " << 10000.0 / (2.0 * M_PI) << std::endl;

  private_nh_.param("left_zero_point",left_zero_point_,0.0);
  private_nh_.param("right_zero_point",right_zero_point_,0.0);

  private_nh_.param("left_driver_neg",left_driver_neg_,false);
  private_nh_.param("right_driver_neg",right_driver_neg_,false);
  private_nh_.param("left_steer_neg",left_steer_neg_,false);
  private_nh_.param("right_steer_neg",right_steer_neg_,false);

  // 初始化电机
  initMotors();
  ROS_INFO("已配置 %d 个电机", (int)motors_.size());

  for (const auto& motor : motors_) {
    ROS_INFO("  电机ID: %d, 类型: %s, CAN地址: 0x%08X, 话题前缀: %s",
      motor.motor_id,
      motor.type == MOTOR_TYPE_DRIVE ? "行走" : "转向",
      motor.can_address,
      motor.topic_prefix.c_str());
  }

  // 创建订阅者
  enable_sub_ = nh_.subscribe("motors/enable", 10, &WalkMotorDriver::enableCallback, this);
  disable_sub_ = nh_.subscribe("motors/disable", 10, &WalkMotorDriver::disableCallback, this);
  omv_servo_cmd_sub_ = nh_.subscribe("omv_servo_cmd", 10, &WalkMotorDriver::omvServoCmdCallback, this);

  can_pub_ = nh_.advertise<can_msgs::Frame>("sent_messages", 10);

  can_sub_ = nh_.subscribe("received_messages", 10, &WalkMotorDriver::canCallback, this);

  // 创建编码器反馈发布者
  encoder_pub_ = nh_.advertise<common::omv_servo_encoder>("omv_servo_encoder", 10);

  // 初始化编码器数据
  encoder_data_.left_drive_vel = 0;
  encoder_data_.right_drive_vel = 0;
  encoder_data_.left_steer_pos = 0;
  encoder_data_.right_steer_pos = 0;
  encoder_data_.left_drive_valid = false;
  encoder_data_.right_drive_valid = false;
  encoder_data_.left_steer_valid = false;
  encoder_data_.right_steer_valid = false;

  // 创建查询定时器 (50ms = 20Hz)
  query_timer_ = nh_.createTimer(ros::Duration(0.05), &WalkMotorDriver::queryCallback, this);
}

WalkMotorDriver::~WalkMotorDriver()
{
  ROS_INFO("行走电机驱动节点关闭");
}

void WalkMotorDriver::initMotors()
{
  // 从参数服务器读取电机配置
  XmlRpc::XmlRpcValue motor_configs;
  if (private_nh_.getParam("motors", motor_configs)) {
    ROS_ASSERT(motor_configs.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int i = 0; i < motor_configs.size(); i++) {
      XmlRpc::XmlRpcValue motor_config = motor_configs[i];

      MotorConfig motor;
      motor.motor_id = static_cast<int>(motor_config["motor_id"]);

      std::string type_str = static_cast<std::string>(motor_config["type"]);
      if (type_str == "drive") {
        motor.type = MOTOR_TYPE_DRIVE;
        motor.can_address = 0x05800000 + motor.motor_id;
      } else if (type_str == "steer") {
        motor.type = MOTOR_TYPE_STEER;
        motor.can_address = 0x06000000 + motor.motor_id;
      } else {
        ROS_WARN("未知电机类型: %s, 默认为行走电机", type_str.c_str());
        motor.type = MOTOR_TYPE_DRIVE;
      }
      motor.topic_prefix = static_cast<std::string>(motor_config["topic_prefix"]);
      motors_.push_back(motor);
    }
  } else {
    // 默认配置：对角舵轮
    ROS_WARN("未找到电机配置，使用默认配置");

    // 左前行走电机
    motors_.push_back({1, MOTOR_TYPE_DRIVE, 0x05800001, "lf_drive"});
    // 右后行走电机
    motors_.push_back({2, MOTOR_TYPE_DRIVE, 0x05800002, "rr_drive"});
    // 左前转向舵轮
    motors_.push_back({3, MOTOR_TYPE_STEER, 0x05800003, "lf_steer"});
    // 右后转向舵轮
    motors_.push_back({4, MOTOR_TYPE_STEER, 0x05800004, "rr_steer"});
  }
}

void WalkMotorDriver::run()
{
  ROS_INFO("run");
  ros::spin();
}
void WalkMotorDriver::enableAllMotors()
{
  // 23 0D 20 01 00 00 00 00
  for (const auto& motor : motors_) {
    if(motor.type == MOTOR_TYPE_STEER){
      uint8_t data[8] = {0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
      sendCanFrame(motor.can_address, 0x0D, data, 8);
      usleep(1000);
      uint8_t speed[8] = {0x23, 0x03, 0x20, 0x01, 0x00, 0x00, 0x01, 0xF4};
      sendCanFrame(motor.can_address, 0x0D, data, 8);
      usleep(1000);
      ROS_INFO("电机 %d (%s) 使能", motor.motor_id, motor.topic_prefix.c_str());
    }
    else if(motor.type == MOTOR_TYPE_DRIVE){
      ROS_INFO("todo");
    }
    
  }
}
void WalkMotorDriver::canCallback(const can_msgs::Frame::ConstPtr& msg)
{
  // 返回数据格式: 60 XX 21 01 DATA...
  if (msg->dlc >= 4) {
    uint8_t cmd_type = msg->data[1];
    uint8_t sub_cmd = msg->data[2];
    uint8_t sub_param = msg->data[3];

    ROS_DEBUG("收到响应: ID=0x%08X, 命令类型=0x%02X", msg->id, cmd_type);

    // 行走电机转速查询响应: 60 03 21 01 DATA_H(h) DATA_H(l) DATA_L(h) DATA_L(l)
    if (cmd_type == 0x03 && sub_cmd == 0x21 && sub_param == 0x01 && msg->dlc >= 8) {
      int32_t velocity = 0;
      velocity = (static_cast<int32_t>(msg->data[4]) << 24) |
                  (static_cast<int32_t>(msg->data[5]) << 16) |
                  (static_cast<int32_t>(msg->data[6]) << 8) |
                  static_cast<int32_t>(msg->data[7]);
      processVelocityResponse(msg->id, velocity);
    }

    // 转向电机位置查询响应: 60 04 21 01 00 00 DATA-H DATA-L
    if (cmd_type == 0x04 && sub_cmd == 0x21 && sub_param == 0x02 && msg->dlc >= 8) {
      uint16_t position = (static_cast<uint16_t>(msg->data[6]) << 8) |
                          static_cast<uint16_t>(msg->data[7]);
      processPositionResponse(msg->id, position);
    }
  }
}

void WalkMotorDriver::enableCallback(const std_msgs::Empty::ConstPtr& msg)
{
  (void)msg;
  enableAllMotors();
}

void WalkMotorDriver::disableCallback(const std_msgs::Empty::ConstPtr& msg)
{
  (void)msg;

  // 失能命令: 23 0C 20 01 00 00 00 00
  for (const auto& motor : motors_) {
    if(motor.type == MOTOR_TYPE_STEER){
      uint8_t data[8] = {0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
      sendCanFrame(motor.can_address, 0x0C, data, 8);
      ROS_INFO("电机 %d (%s) 使能", motor.motor_id, motor.topic_prefix.c_str());
    }
    else if(motor.type == MOTOR_TYPE_DRIVE){
      ROS_INFO("todo");
    }
  }
}

void WalkMotorDriver::omvServoCmdCallback(const common::omv_servo_cmd::ConstPtr& msg)
{
  // OMV伺服命令消息格式:
  // std_msgs/Header header
  // float64 sc_vel              - 线速度 (m/s)
  // float64 sc_main_theta        - 主偏转角 (rad)
  // float64 sc_left_vel         - 左轮速度 (m/s)
  // float64 sc_right_vel        - 右轮速度 (m/s)
  // float64 sc_left_theta       - 左轮偏转角 (rad)
  // float64 sc_right_theta      - 右轮偏转角 (rad)

  ROS_DEBUG("收到OMV伺服命令:");
  ROS_DEBUG("  sc_vel: %.3f m/s", msg->sc_vel);
  ROS_DEBUG("  sc_main_theta: %.3f rad", msg->sc_main_theta);
  ROS_INFO("  sc_left_vel: %.3f m/s", msg->sc_left_vel);
  ROS_INFO("  sc_right_vel: %.3f m/s", msg->sc_right_vel);
  ROS_INFO("  sc_left_theta: %.3f rad", msg->sc_left_theta);
  ROS_INFO("  sc_right_theta: %.3f rad", msg->sc_right_theta);

  double left_vel = msg->sc_left_vel;
  double right_vel = msg->sc_right_vel;
  double left_theta = msg->sc_left_theta;
  double right_theta = msg->sc_right_theta;
  left_theta += left_zero_point_;
  right_theta += right_zero_point_;
  
  left_vel = left_driver_neg_?-left_vel:left_vel;
  right_vel = right_driver_neg_?-right_vel:right_vel;
  left_theta = left_steer_neg_?-left_theta:left_theta;
  right_theta = right_steer_neg_?-right_theta:right_theta;

  enableAllMotors();
  sendDriveVelocity(left_vel, right_vel);
  sendSteerPosition(left_theta, right_theta);
}

void WalkMotorDriver::sendDriveVelocity(const double left_vel,const double right_vel)
{
  // 将浮点速度转换为CAN整数 (乘以比例因子)
  double left_radio = left_vel/velocity_rated_speed_;
  double right_radio = right_vel/velocity_rated_speed_;
  int32_t left_can_vel = static_cast<int32_t>(left_radio * 10000);
  int32_t right_can_vel = static_cast<int32_t>(right_radio * 10000);

  // 限制速度范围
  if (left_can_vel > max_motor_rpm_) left_can_vel = max_motor_rpm_;
  if (left_can_vel < -max_motor_rpm_) left_can_vel = -max_motor_rpm_;
  if (right_can_vel > max_motor_rpm_) right_can_vel = max_motor_rpm_;
  if (right_can_vel < -max_motor_rpm_) right_can_vel = -max_motor_rpm_;
  //   使能指令： 23 0D 20 01 00 00 00 00
  //   速度指令：23 00 20 01 00 00 13 88 （0x1388 = 5000）
  uint8_t data[8] = {0x23, 0x00, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};

  // 对所有行走电机发送速度控制
  for (const auto& motor : motors_) {
    if (motor.type == MOTOR_TYPE_DRIVE) {
      int32_t vel = (motor.topic_prefix == "lf_drive") ? left_can_vel : right_can_vel;
      int32ToBytes(vel, &data[4]);
      sendCanFrame(motor.can_address, 0x00, data, 8);
      ROS_DEBUG("行走电机 %d 速度: %.3f m/s -> %d", motor.motor_id,
        (motor.topic_prefix == "lf_drive") ? left_vel : right_vel, vel);
    }
  }
}

void WalkMotorDriver::sendSteerPosition(const double left_theta, const double right_theta)
{
  // 将弧度转换为位置值 (10000/圈 = 10000/(2π) /rad)
  double left_motor_theta = steer_ratio_*left_theta;
  double right_motor_theta = steer_ratio_*right_theta;
  int32_t left_pos = static_cast<int32_t>(left_motor_theta * position_scale_);
  int32_t right_pos = static_cast<int32_t>(right_motor_theta * position_scale_);

  // 位置控制命令: 23 02 20 01 DATA_H(h) DATA_H(l) DATA_L(h) DATA_L(l)
  uint8_t data[8] = {0x23, 0x02, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};

  // 对所有转向电机发送位置控制
  for (const auto& motor : motors_) {
    if (motor.type == MOTOR_TYPE_STEER) {
      int32_t pos = (motor.topic_prefix == "lf_steer") ? left_pos : right_pos;
      uint32ToBytes(static_cast<uint32_t>(pos), &data[4]);
      sendCanFrame(motor.can_address, 0x02, data, 8);
      usleep(1000);
      ROS_DEBUG("转向电机 %d 位置: %.3f rad -> %d", motor.motor_id,
        (motor.topic_prefix == "lf_steer") ? left_motor_theta : right_motor_theta, pos);
    }
  }
}

void WalkMotorDriver::sendCanFrame(uint32_t can_address, uint8_t cmd_type, const uint8_t* data, uint8_t dlc)
{
  can_msgs::Frame can_msg;

  can_msg.id = can_address;
  can_msg.is_extended = true;  // 扩展帧
  can_msg.dlc = dlc;

  for (uint8_t i = 0; i < dlc && i < 8; i++) {
    can_msg.data[i] = data[i];
  }

  can_pub_.publish(can_msg);
}

void WalkMotorDriver::int32ToBytes(int32_t value, uint8_t* bytes)
{
  // 大端序: 高位在前
  bytes[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
  bytes[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
  bytes[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  bytes[3] = static_cast<uint8_t>(value & 0xFF);
}

void WalkMotorDriver::uint32ToBytes(uint32_t value, uint8_t* bytes)
{
  // 大端序: 高位在前
  bytes[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
  bytes[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
  bytes[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  bytes[3] = static_cast<uint8_t>(value & 0xFF);
}

void WalkMotorDriver::queryCallback(const ros::TimerEvent& event)
{
  (void)event;  // 避免未使用警告
  // 定时发送查询命令
  queryMotorVelocity();
  querySteerPosition();
}

void WalkMotorDriver::queryMotorVelocity()
{
  // 电机转速查询: 40 03 21 01 00 00 00 00
  uint8_t data[8] = {0x40, 0x03, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00};

  for (const auto& motor : motors_) {
    if (motor.type == MOTOR_TYPE_DRIVE) {
      sendCanFrame(motor.can_address, 0x03, data, 8);
    }
  }
}

void WalkMotorDriver::querySteerPosition()
{
  // 转子机械位置查询: 40 04 21 01 00 00 00 00
  uint8_t data[8] = {0x40, 0x04, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00};

  for (const auto& motor : motors_) {
    if (motor.type == MOTOR_TYPE_STEER) {
      sendCanFrame(motor.can_address, 0x04, data, 8);
    }
  }
}

void WalkMotorDriver::processVelocityResponse(uint32_t can_address, int32_t velocity)
{
  // 根据CAN地址找到对应的电机
  for (const auto& motor : motors_) {
    if (motor.can_address == can_address && motor.type == MOTOR_TYPE_DRIVE) {
      if (motor.topic_prefix == "lf_drive") {
        encoder_data_.left_drive_vel = velocity;
        encoder_data_.left_drive_valid = true;
        ROS_DEBUG("左行走电机转速: %d RPM", velocity);
      } else if (motor.topic_prefix == "rr_drive") {
        encoder_data_.right_drive_vel = velocity;
        encoder_data_.right_drive_valid = true;
        ROS_DEBUG("右行走电机转速: %d RPM", velocity);
      }
      break;
    }
  }
  publishEncoderFeedback();
}

void WalkMotorDriver::processPositionResponse(uint32_t can_address, uint16_t position)
{
  // 根据CAN地址找到对应的电机
  for (const auto& motor : motors_) {
    if (motor.can_address == can_address && motor.type == MOTOR_TYPE_STEER) {
      if (motor.topic_prefix == "lf_steer") {
        encoder_data_.left_steer_pos = position;
        encoder_data_.left_steer_valid = true;
        ROS_DEBUG("左转向电机位置: %u", position);
      } else if (motor.topic_prefix == "rr_steer") {
        encoder_data_.right_steer_pos = position;
        encoder_data_.right_steer_valid = true;
        ROS_DEBUG("右转向电机位置: %u", position);
      }
      break;
    }
  }
  publishEncoderFeedback();
}

void WalkMotorDriver::publishEncoderFeedback()
{
  common::omv_servo_encoder encoder_msg;
  encoder_msg.header.stamp = ros::Time::now();

  // 将位置值转换为弧度 (0-9999 对应 0-2π)
  double left_theta = encoder_data_.left_steer_valid ?
                      static_cast<double>(encoder_data_.left_steer_pos) /position_scale_/steer_ratio_ : 0.0;
  double right_theta = encoder_data_.right_steer_valid ?
                       static_cast<double>(encoder_data_.right_steer_pos) /position_scale_/steer_ratio_ : 0.0;

  // 将RPM转换为线速度 (m/s)
  double left_vel = encoder_data_.left_drive_valid ?
                   encoder_data_.left_drive_vel : 0.0;
  double right_vel = encoder_data_.right_drive_valid ?
                    encoder_data_.right_drive_vel : 0.0;

  // encoder_msg.se_vel = (left_vel + right_vel) / 2.0;  // 平均速度
  // encoder_msg.se_main_theta = (left_theta + right_theta) / 2.0;  // 平均偏转角
  encoder_msg.se_left_theta = left_theta;
  encoder_msg.se_right_theta = right_theta;
  encoder_msg.se_left_vel = left_vel;
  encoder_msg.se_right_vel = right_vel;

  encoder_pub_.publish(encoder_msg);

  ROS_DEBUG("发布编码器反馈: vel=%.3f, left_theta=%.3f, right_theta=%.3f",
            encoder_msg.se_vel, encoder_msg.se_left_theta, encoder_msg.se_right_theta);
}

}

// 主函数
int main(int argc, char** argv)
{
  setlocale(LC_ALL, "zh_CN.UTF-8");
  ros::init(argc, argv, "walk_motor_driver");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  walk_motor_driver::WalkMotorDriver driver(nh, private_nh);
  driver.run();
  ROS_INFO("walk_motor_driver 退出");
  return 0;
}

