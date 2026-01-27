# 行走电机驱动功能包 (ROS1)

## 功能说明
此功能包用于驱动对角舵轮底盘，包含多个电机（行走电机和转向舵轮），支持以下控制模式：

- **使能控制**: 激活所有电机
- **失能控制**: 停用所有电机
- **速度控制**: 控制电机转速 (-10000 ~ +10000)
- **速度模式**: 设置位置控制时的运行速度
- **转矩控制**: 控制电机转矩 (-10000 ~ +10000)
- **位置控制**: 控制电机位置 (10000/圈)
- **OMV伺服命令**: 接收 `common::OmvServoCmd` 消息进行综合控制

## CAN通信协议

### 命令格式
所有命令均为8字节CAN帧：

| 功能 | CAN帧数据 | 说明 |
|------|-----------|------|
| 使能 | 23 0D 20 01 00 00 00 00 | 激活电机 |
| 失能 | 23 0C 20 01 00 00 00 00 | 停用电机 |
| 速度控制 | 23 00 20 01 [DATA] | 速度值(小端序) |
| 速度模式 | 23 03 20 01 [DATA] | 位置控制时的速度 |
| 转矩控制 | 23 01 20 01 [DATA] | 转矩值(小端序) |
| 位置控制 | 23 02 20 01 [DATA] | 位置值(小端序) |

### 返回数据
- 返回地址: 0x0580000 + 电机ID
- 返回数据格式: 60 XX 20 00 00 00 00 00

## OMV伺服命令消息格式

### common::OmvServoCmd
```msg
std_msgs/Header header
float64 sc_vel              # 线速度 (m/s)
float64 sc_main_theta        # 主偏转角 (rad)
float64 sc_left_vel         # 左轮速度 (m/s)
float64 sc_right_vel        # 右轮速度 (m/s)
float64 sc_left_theta       # 左轮偏转角 (rad)
float64 sc_right_theta      # 右轮偏转角 (rad)
```

## 控制值范围

### 速度控制
- 范围: -10000 ~ +10000
- -10000: 负额定转速
- +10000: 正额定转速

### 速度模式 (用于位置控制)
- 单位: 转/分钟 (rpm)
- 示例: 500 rpm = 0x01F4

### 转矩控制
- 范围: -10000 ~ +10000
- -10000: 负额定转矩
- +10000: 正额定转矩

### 位置控制
- 范围: -4294967295 ~ +4294967295
- 分辨率: 10000/圈
- 转换: 角度(度) × 10000/360

## 编译

```bash
cd ~/ts_amr_ws
catkin_make
source devel/setup.bash
```

## 启动驱动

```bash
# 启动驱动(默认配置: 对角舵轮)
roslaunch walk_motor_driver walk_motor_driver.launch
```

## ROS话题

### 控制话题
| 话题 | 类型 | 说明 |
|------|------|------|
| /motors/enable | std_msgs/Empty | 使能所有电机 |
| /motors/disable | std_msgs/Empty | 失能所有电机 |
| /motors/velocity_cmd | std_msgs/Int32 | 速度控制命令 |
| /motors/profile_velocity_cmd | std_msgs/Int32 | 速度模式(位置控制用) |
| /motors/torque_cmd | std_msgs/Int32 | 转矩控制命令 |
| /motors/position_cmd | std_msgs/Int64 | 位置控制命令 |
| /omv_servo_cmd | common/OmvServoCmd | OMV伺服命令 |
| /to_can_bus | can_msgs/Frame | 发送到CAN总线 |
| /from_can_bus | can_msgs/Frame | 从CAN总线接收 |

## 使用示例

### 示例1: 转向电机控制
**命令: 以500转/分钟的速度逆时针旋转72度**

```bash
# (a) 使能
rostopic pub /motors/enable std_msgs/Empty "{}"

# (b) 运行速度: 23 03 20 01 00 00 01 F4 (500 rpm)
rostopic pub /motors/profile_velocity_cmd std_msgs/Int32 "data: 500"

# (c) 位置控制: 23 02 20 01 00 00 07 D0 (72度 = 2000)
rostopic pub /motors/position_cmd std_msgs/Int64 "data: 2000"
```

### 示例2: 使用OMV伺服命令
**命令: 前进速度1m/s，左轮和右轮偏转30度**

```bash
# 使能
rostopic pub /motors/enable std_msgs/Empty "{}"

# 发送OMV命令
rostopic pub /omv_servo_cmd common/OmvServoCmd \
  "{header: {stamp: now}, \
    sc_vel: 1.0, \
    sc_main_theta: 0.52, \
    sc_left_vel: 1.0, \
    sc_right_vel: 1.0, \
    sc_left_theta: 0.52, \
    sc_right_theta: 0.52}"
```

## 使用测试脚本

### 测试OMV伺服命令
```bash
# 添加执行权限
chmod +x ~/ts_amr_ws/src/driver/walk_motor_driver/scripts/test_omv_cmd.py

# 运行测试
python ~/ts_amr_ws/src/driver/walk_motor_driver/scripts/test_omv_cmd.py
```

### 测试转向电机
```bash
# 添加执行权限
chmod +x ~/ts_amr_ws/src/driver/walk_motor_driver/scripts/test_steer_motor.py

# 运行测试
python ~/ts_amr_ws/src/driver/walk_motor_driver/scripts/test_steer_motor.py
```

### 测试行走电机
```bash
# 添加执行权限
chmod +x ~/ts_amr_ws/src/driver/walk_motor_driver/scripts/test_drive_motor.py

# 运行测试
python ~/ts_amr_ws/src/driver/walk_motor_driver/scripts/test_drive_motor.py
```

## 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| can_channel | string | can0 | CAN通道名称 |
| max_velocity | int | 10000 | 最大速度限制 |
| max_torque | int | 10000 | 最大转矩限制 |
| velocity_scale | double | 1000.0 | 速度转换比例 (m/s -> CAN值) |
| position_scale | double | 1591.55 | 位置转换比例 (rad -> CAN值, 10000/2π) |
| motors | list | 见下文 | 电机配置列表 |

### 电机配置 (motors参数)
默认配置为对角舵轮：

```yaml
motors:
  - motor_id: 1
    type: "drive"
    topic_prefix: "lf_drive"
  - motor_id: 2
    type: "drive"
    topic_prefix: "rr_drive"
  - motor_id: 3
    type: "steer"
    topic_prefix: "lf_steer"
  - motor_id: 4
    type: "steer"
    topic_prefix: "rr_steer"
```

- `motor_id`: 电机ID (1-4)
- `type`: "drive" (行走) 或 "steer" (转向)
- `topic_prefix`: 话题前缀
- CAN地址自动计算: 0x0580000 + motor_id

## 注意事项

1. 电机地址计算公式: 0x0580000 + motor_id
2. 速度和转矩值超出范围时会被自动限制
3. 位置控制时需要先设置速度模式(0x03命令)
4. 位置控制使用32位无符号整数，实际值为10000/圈
5. 发送的CAN帧为扩展帧格式
6. 假设已经配置好CAN接口驱动
7. 此为ROS1版本，使用catkin编译
8. OMV命令会同时控制行走电机速度和转向电机位置
