/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_IO_INDEX_H_
#define amr_COMMON_INCLUDE_amr_COMMON_IO_INDEX_H_
#include <iostream>
#include <string>

// IO设备集编号: 所有的IO设备及其编号

namespace io {
// 可添加设备与设备编号, 不可修改!!!
// 共有设备
constexpr uint8_t kNoneIoDevice = 0;
// 防撞条
constexpr uint8_t kBumper = 1;
constexpr uint8_t kEmergencyStop = 2;

// 输入 1~100
// 托盘叉车上下限位 (诺力)
constexpr uint8_t kForkUpperLimitSwitch = 11;
constexpr uint8_t kForkDownerLimitSwitch = 12;
// 托盘叉车左右叉腿防撞 (诺力)
constexpr uint8_t kLeftForkLegSwitch = 13;
constexpr uint8_t kRightForkLegSwitch = 14;
// 托盘叉车货架位置左右检测 (诺力)
constexpr uint8_t kLeftPositionSwitch = 15;
constexpr uint8_t kRightPositionSwitch = 16;
// 叉车避障等级输入(诺力 / 中力)
constexpr uint8_t kLaserObstacleLevelI = 17;
constexpr uint8_t kLaserObstacleLevelII = 18;
constexpr uint8_t kLaserObstacleLevelIII = 19;

// 叉车挡板检测开关(中力)
constexpr uint8_t kPalletPositionSwitch = 20;

// 门架高度开关 2个 (三向车)
constexpr uint8_t kHeightSwitch_1 = 21;
constexpr uint8_t kHeightSwitch_2 = 22;

// 侧移到位开关 2个 (三向车)
constexpr uint8_t kLateralLeftSwitch = 23;
constexpr uint8_t kLateralRightSwitch = 24;

// 旋转到位开关 3个 左到位,右到位,旋转减速 (三向车)
constexpr uint8_t kRotateLeftSwitch = 25;
constexpr uint8_t kRotateRightSwitch = 26;
constexpr uint8_t kRotateDecelSwtich = 27;

constexpr uint8_t kManualSwtich = 28;

// 防撞条复位开关
constexpr uint8_t kBumpReset = 29;

// 叉车io超声波避障 前 左 右 三个方向
constexpr uint8_t kForwardUltrasonic = 30;
constexpr uint8_t kLeftUltrasonic = 31;
constexpr uint8_t kRightUltrasonic = 32;

// 叉车避障激光2等级输入(诺力 / 中力)
constexpr uint8_t kLaser2ObstacleLevelI = 33;
constexpr uint8_t kLaser2ObstacleLevelII = 34;
constexpr uint8_t kLaser2ObstacleLevelIII = 35;

// 叉车输入 主电源(断电状态下，工控机继续保持带点状态)
constexpr uint8_t kPowerSupply = 60;

// 叉车输出 71~90
// 柯蒂斯复位继电器 (诺力托盘车)
constexpr uint8_t kCurtisReset = 71;
// 避障传感器选择地图4输出通道 (诺力托盘车 / 堆高车 / 三向车)
constexpr uint8_t kLaserObstacleChannelI = 72;
constexpr uint8_t kLaserObstacleChannelII = 73;
constexpr uint8_t kLaserObstacleChannelIII = 74;
constexpr uint8_t kLaserObstacleChannelIV = 75;
constexpr uint8_t kLaserObstacleChannelV = 76;
// 叉车转向灯
constexpr uint8_t kCorningLedConstantOn = 77;
constexpr uint8_t kCorningLedLeftBlink = 78;
constexpr uint8_t kCorningLedRightBlink = 79;
// v1.1.5新增叉车状态灯
constexpr uint8_t kALLGreenConstantOn = 61;
constexpr uint8_t kALLRedConstantOn = 62;
constexpr uint8_t kLeftYellowConstantOn = 63;
constexpr uint8_t kRightYellowConstantOn = 64;
constexpr uint8_t kALLBlueConstantOn = 65;
constexpr uint8_t kALLYellowConstantOn = 66;

// 叉车自动充电继电器
constexpr uint8_t kRecvChargeArrivedSignal = 78;
constexpr uint8_t kSendChargeArrivedSignal = 79;
constexpr uint8_t kChargeRelay = 80;

// 避障传感器2选择地图4输出通道 (诺力托盘车 / 堆高车 / 三向车)
constexpr uint8_t kLaser2ObstacleChannelI = 81;
constexpr uint8_t kLaser2ObstacleChannelII = 82;
constexpr uint8_t kLaser2ObstacleChannelIII = 83;
constexpr uint8_t kLaser2ObstacleChannelIV = 84;
constexpr uint8_t kLaser2ObstacleChannelV = 85;

// 顶升
// 顶升输入 91~150
// 顶升车上下限位
constexpr uint8_t kJackUpUpperLimitSwitchI = 91;
constexpr uint8_t kJackUpUpperLimitSwitchII = 92;
constexpr uint8_t kJackUpDownerLimitSwitchI = 93;
constexpr uint8_t kJackUpDownerLimitSwitchII = 94;
//滚筒车左右限位
constexpr uint8_t kRollerOutLimitSwitchI = 91;
constexpr uint8_t kRollerOutLimitSwitchII = 92;
constexpr uint8_t kRollerInLimitSwitchI = 93;
constexpr uint8_t kRollerInLimitSwitchII = 94;

constexpr uint8_t kRollerInLimitSwitch = 91;
constexpr uint8_t kRollerMiddleLimitSwitch = 92;
constexpr uint8_t kRollerOutLimitSwitch = 93;

// 顶升托盘 归零光电开关
constexpr uint8_t kJackUpPalletZeroSwitchI = 95;
constexpr uint8_t kJackUpPalletZeroSwitchII = 96;

// 顶升车设备对接信号
constexpr uint8_t kJRollerJointI = 95;
constexpr uint8_t kJRollerJointII = 96;

// 无线手柄DI信号量
constexpr uint8_t kRemoteControlModeSwitchOpen = 100;
constexpr uint8_t kRemoteControlModeSwitchClose = 150;
constexpr uint8_t kRemoteControlMoveForward = 101;
constexpr uint8_t kRemoteControlMoveBackward = 102;
constexpr uint8_t kRemoteControlMoveLeft = 103;
constexpr uint8_t kRemoteControlMoveRight = 104;
constexpr uint8_t kRemoteControlUp = 105;
constexpr uint8_t kRemoteControlDown = 106;

// 放射源车遥控信号：二进制捕获
// 1-上升 2-下降 3-前进 4-后退
// 7-进入遥控模式、8-退出遥控模式
constexpr uint8_t kRemoteControl0 = 100;
constexpr uint8_t kRemoteControl1 = 101;
constexpr uint8_t kRemoteControl2 = 102;
constexpr uint8_t kRemoteControl3 = 103;

// 前进激光避障输入 (磁条顶升/三合一顶升)
constexpr uint8_t kLaserObstacleForwardLevelI = 107;
constexpr uint8_t kLaserObstacleForwardLevelII = 108;
constexpr uint8_t kLaserObstacleForwardLevelIII = 109;
// 后退激光避障输入 (磁条顶升／三合一顶升)
constexpr uint8_t kLaserObstacleBackwardLevelI = 110;
constexpr uint8_t kLaserObstacleBackwardLevelII = 111;
constexpr uint8_t kLaserObstacleBackwardLevelIII = 112;
// 前后防撞 (磁条顶升)
constexpr uint8_t kForwardBumper = 113;
constexpr uint8_t kBackwardBumper = 114;

// 顶升输出 151~180
constexpr uint8_t kJackUpLedRed = 151;
constexpr uint8_t kJackUpLedYellow = 152;
constexpr uint8_t kJackUpLedGreen = 153;
// 顶升自动充电继电器
constexpr uint8_t kJackUpCharge = 154;
constexpr uint8_t kJackUpLiftMotorBreak = 155;
constexpr uint8_t kJackUpRotateMotorBreak = 156;
constexpr uint8_t kJackUp24vLowpower = 157;
constexpr uint8_t kJackUp48vLowpower = 158;
constexpr uint8_t kJackUpMoveMotorBreak = 159;

// 顶升前进激光避障等级输入
constexpr uint8_t kFwdObstacleLI = 72;
constexpr uint8_t kFwdObstacleLII = 73;
constexpr uint8_t kFwdObstacleLIII = 74;

// 顶升前进激光避障设置
constexpr uint8_t kSetFwdObstacleChannel1 = 121;
constexpr uint8_t kSetFwdObstacleChannel2 = 122;
constexpr uint8_t kSetFwdObstacleChannel3 = 123;
constexpr uint8_t kSetFwdObstacleChannel4 = 124;
constexpr uint8_t kSetFwdObstacleChannel5 = 125;

// 前进激光避障输出 (磁条顶升/三合一顶升)
constexpr uint8_t kLaserObstacleForwardChannelI = 159;
constexpr uint8_t kLaserObstacleForwardChannelII = 160;
constexpr uint8_t kLaserObstacleForwardChannelIII = 161;
constexpr uint8_t kLaserObstacleForwardChannelIV = 162;
constexpr uint8_t kLaserObstacleForwardChannelV = 163;
// 后退激光避障输出 (磁条顶升/三合一顶升)
constexpr uint8_t kLaserObstacleBackwardChannelI = 164;
constexpr uint8_t kLaserObstacleBackwardChannelII = 165;
constexpr uint8_t kLaserObstacleBackwardChannelIII = 166;
constexpr uint8_t kLaserObstacleBackwardChannelIV = 167;
constexpr uint8_t kLaserObstacleBackwardChannelV = 168;
// Io控制抬降(中力液压顶升)/放射源车
constexpr uint8_t kJackUpIoLift = 169;
constexpr uint8_t kJackUpIoDown = 170;

// Io控制滚筒,正反转
constexpr uint8_t kRoller1IoIn = 169;  // 1代表内侧电机
constexpr uint8_t kRoller1IoOut = 170;
constexpr uint8_t kRoller2IoIn = 190;  // 2代表外侧电机
constexpr uint8_t kRoller2IoOut = 191;

constexpr uint8_t kRollerIoFMQ = 171;

//　备用按钮
constexpr uint8_t kBackupKey1 = 171;  //故障复位--手自动复位信号
constexpr uint8_t kBackupKey2 = 172;  //电梯门触发信号
// 电梯门控制信号
constexpr uint8_t kOpenDoor = 173;
//夹抱装置输入
constexpr uint8_t kClampLeft = 181;
constexpr uint8_t kClampRight = 182;
constexpr uint8_t kClampDetect = 183;
constexpr uint8_t kClampLeftReach1 = 184;
constexpr uint8_t kClampLeftReach2 = 185;
constexpr uint8_t kClampLeftReach3 = 186;
constexpr uint8_t kClampRightReach1 = 187;
constexpr uint8_t kClampRightReach2 = 188;
constexpr uint8_t kClampRightReach3 = 189;

}  // namespace io

#endif  // amr_COMMON_INCLUDE_amr_COMMON_IO_INDEX_H_
