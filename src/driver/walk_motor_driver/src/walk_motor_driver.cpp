#include "walk_motor_driver/walk_motor_driver.hpp"

namespace walk_motor_driver
{

WalkMotorDriver::WalkMotorDriver(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh), private_nh_(private_nh)
{
  // 获取参数
  private_nh_.param("max_velocity", max_velocity_, 10000);
  private_nh_.param("max_torque", max_torque_, 10000);
  private_nh_.param("velocity_scale", velocity_scale_, 1000.0);
  private_nh_.param("position_scale", position_scale_, 10000.0 / (2.0 * M_PI));
  std::cout << "scale1: " << (180.0/M_PI)*(10000/360.0) << std::endl;
  std::cout << "scale2: " << 10000.0 / (2.0 * M_PI) << std::endl;

  // 初始化电机
  initMotors();

  ROS_INFO("行走电机驱动节点启动");
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

  // 创建CAN发布者 (发布标准ROS CAN格式话题)
  can_pub_ = nh_.advertise<can_msgs::Frame>("sent_messages", 10);

  // 创建CAN订阅者 (假设收到消息)
  can_sub_ = nh_.subscribe("received_messages", 10, &WalkMotorDriver::canCallback, this);

  ROS_INFO("订阅话题:");
  ROS_INFO("  - motors/enable (使能所有电机)");
  ROS_INFO("  - motors/disable (失能所有电机)");
  ROS_INFO("  - omv_servo_cmd (OMV伺服命令)");
  ROS_INFO("发布话题:");
  ROS_INFO("  - sent_messages (CAN发送)");
  ROS_INFO("  - received_messages (CAN接收)");
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
  // 假设收到消息,可以在这里处理返回的数据
  // 返回数据格式: 60 XX 20 00 00 00 00 00
  uint8_t cmd_type = msg->data[1];
  ROS_DEBUG("收到响应: ID=0x%08X, 命令类型=0x%02X", msg->id, cmd_type);
}

void WalkMotorDriver::enableCallback(const std_msgs::Empty::ConstPtr& msg)
{
  (void)msg;  // 避免未使用警告
  enableAllMotors();
}

void WalkMotorDriver::disableCallback(const std_msgs::Empty::ConstPtr& msg)
{
  (void)msg;  // 避免未使用警告

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

void WalkMotorDriver::velocityCallback(const std_msgs::Int32::ConstPtr& msg)
{
  // 假设消息格式为：[左前行走, 右后行走, 左前转向, 右后转向]
  // 实际应用中需要根据具体消息格式调整

  // 这里只示例行走电机的速度控制
  if (motors_.size() >= 2) {
    int32_t left_vel = msg->data;
    int32_t right_vel = msg->data;  // 简化示例，实际应从消息中分别获取

    // 限制速度范围
    if (left_vel > max_velocity_) left_vel = max_velocity_;
    if (left_vel < -max_velocity_) left_vel = -max_velocity_;
    if (right_vel > max_velocity_) right_vel = max_velocity_;
    if (right_vel < -max_velocity_) right_vel = -max_velocity_;

    // 发送速度控制命令
    uint8_t data[8] = {0x23, 0x00, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};

    // 左前行走电机
    int32ToBytes(left_vel, &data[4]);
    sendCanFrame(motors_[0].can_address, 0x00, data, 8);

    // 右后行走电机
    int32ToBytes(right_vel, &data[4]);
    sendCanFrame(motors_[1].can_address, 0x00, data, 8);

    ROS_DEBUG("速度控制: 左轮=%d, 右轮=%d", left_vel, right_vel);
  }
}

void WalkMotorDriver::profileVelocityCallback(const std_msgs::Int32::ConstPtr& msg)
{
  // 速度模式命令: 23 03 20 01 DATA_H(h) DATA_H(l) DATA_L(h) DATA_L(l)
  // 用于设置位置控制时的运行速度
  int32_t velocity = msg->data;

  // 限制速度范围
  if (velocity > max_velocity_) {
    velocity = max_velocity_;
  } else if (velocity < -max_velocity_) {
    velocity = -max_velocity_;
  }

  uint8_t data[8] = {0x23, 0x03, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
  int32ToBytes(velocity, &data[4]);

  // 对所有电机发送速度模式命令
  for (const auto& motor : motors_) {
    sendCanFrame(motor.can_address, 0x03, data, 8);
  }

  ROS_INFO("设置速度模式: %d", velocity);
}

void WalkMotorDriver::torqueCallback(const std_msgs::Int32::ConstPtr& msg)
{
  int32_t torque = msg->data;

  // 限制转矩范围
  if (torque > max_torque_) {
    torque = max_torque_;
  } else if (torque < -max_torque_) {
    torque = -max_torque_;
  }

  // 转矩控制命令: 23 01 20 01 DATA_H(h) DATA_H(l) DATA_L(h) DATA_L(l)
  uint8_t data[8] = {0x23, 0x01, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
  int32ToBytes(torque, &data[4]);

  // 对所有电机发送转矩控制命令
  for (const auto& motor : motors_) {
    sendCanFrame(motor.can_address, 0x01, data, 8);
  }

  ROS_DEBUG("转矩控制: %d", torque);
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
  ROS_DEBUG("  sc_left_vel: %.3f m/s", msg->sc_left_vel);
  ROS_DEBUG("  sc_right_vel: %.3f m/s", msg->sc_right_vel);
  ROS_INFO("  sc_left_theta: %.3f rad", msg->sc_left_theta);
  ROS_INFO("  sc_right_theta: %.3f rad", msg->sc_right_theta);

  // 发送行走电机速度控制
  // sendDriveVelocity(msg->sc_left_vel, msg->sc_right_vel);

  // 发送转向电机位置控制
  sendSteerPosition(msg->sc_left_theta, msg->sc_right_theta);
}

void WalkMotorDriver::sendDriveVelocity(double left_vel, double right_vel)
{
  // 将浮点速度转换为CAN整数 (乘以比例因子)
  int32_t left_can_vel = static_cast<int32_t>(left_vel * velocity_scale_);
  int32_t right_can_vel = static_cast<int32_t>(right_vel * velocity_scale_);

  // 限制速度范围
  if (left_can_vel > max_velocity_) left_can_vel = max_velocity_;
  if (left_can_vel < -max_velocity_) left_can_vel = -max_velocity_;
  if (right_can_vel > max_velocity_) right_can_vel = max_velocity_;
  if (right_can_vel < -max_velocity_) right_can_vel = -max_velocity_;

  // 速度控制命令: 23 00 20 01 DATA_H(h) DATA_H(l) DATA_L(h) DATA_L(l)
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

void WalkMotorDriver::sendSteerPosition(double left_theta, double right_theta)
{
  // 将弧度转换为位置值 (10000/圈 = 10000/(2π) /rad)
  int32_t left_pos = static_cast<int32_t>(left_theta * position_scale_);
  int32_t right_pos = static_cast<int32_t>(right_theta * position_scale_);

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
        (motor.topic_prefix == "lf_steer") ? left_theta : right_theta, pos);
    }
  }
}

void WalkMotorDriver::sendCanFrame(uint32_t can_address, uint8_t cmd_type, const uint8_t* data, uint8_t dlc)
{
  can_msgs::Frame can_msg;

  // 标准ROS CAN帧格式
  can_msg.id = can_address;
  can_msg.is_extended = true;  // 扩展帧
  can_msg.dlc = dlc;

  for (uint8_t i = 0; i < dlc && i < 8; i++) {
    can_msg.data[i] = data[i];
  }

  can_pub_.publish(can_msg);
}

// void WalkMotorDriver::int32ToBytes(int32_t value, uint8_t* bytes)
// {
//   // 小端序: 低位在前
//   bytes[0] = static_cast<uint8_t>(value & 0xFF);
//   bytes[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
//   bytes[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
//   bytes[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
// }

// void WalkMotorDriver::uint32ToBytes(uint32_t value, uint8_t* bytes)
// {
//   // 小端序: 低位在前
//   bytes[0] = static_cast<uint8_t>(value & 0xFF);
//   bytes[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
//   bytes[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
//   bytes[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
// }
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
}  // namespace walk_motor_driver

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
