int32 ERROR_CODE
int32 ERROR_CODE_RESUME_LAST = 0                # 错误恢复最终状态,恢复需要三秒后才赋值
int32 ERROR_CODE_20  = 20                       # 调度与机器人同步异常
int32 ERROR_CODE_30  = 30                       # 调度与机器人同步异常
int32 ERROR_CODE_40  = 40                       # 调度与机器人同步异常
int32 ERROR_CODE_41  = 41                       # 调度与机器人同步异常
int32 ERROR_CODE_42  = 42                       # 调度与机器人同步异常
int32 ERROR_CODE_50  = 50                       # 调度与机器人同步异常
int32 ERROR_CODE_60  = 60                       # 调度与机器人同步异常
int32 ERROR_CODE_70  = 70                       # 调度与机器人同步异常
int32 ERROR_CODE_71  = 71                       # 调度与机器人同步异常
int32 ERROR_CODE_72  = 72                       # 调度与机器人同步异常
int32 ERROR_CODE_73  = 73                       # 调度与机器人同步异常
int32 ERROR_CODE_74  = 74                       # 调度与机器人同步异常
int32 ERROR_CODE_80  = 80                       # 调度与机器人同步异常
int32 ERROR_CODE_90  = 90                       # 调度与机器人同步异常
int32 ERROR_CODE_91  = 91                       # 调度与机器人同步异常
int32 ERROR_CODE_100 = 100                      # 调度与机器人同步异常
int32 ERROR_CODE_110 = 110                      # 故障-压力传感器丢失
int32 ERROR_CODE_111 = 111                      # 调度与机器人同步异常
int32 ERROR_CODE_122 = 122                      # 调度与机器人同步异常
int32 ERROR_CODE_130 = 130                      # 调度与机器人同步异常
int32 ERROR_CODE_200 = 200                      # 调度与机器人同步异常
int32 ERROR_CODE_201 = 201                      # 调度与机器人同步异常
int32 ERROR_CODE_202 = 202                      # 调度与机器人同步异常
int32 ERROR_CODE_203 = 203                      # 调度与机器人同步异常
int32 ERROR_CODE_205 = 205                      # 调度与机器人同步异常
int32 ERROR_CODE_220 = 220                      # 故障-导航雷达丢失 
int32 ERROR_CODE_265 = 265                      # 托盘位空缺
int32 ERROR_CODE_300 = 300                      # 调度与机器人同步异常
int32 ERROR_CODE_301 = 301                      # 调度与机器人同步异常
int32 ERROR_CODE_302 = 302                      # 调度与机器人同步异常
int32 ERROR_CODE_303 = 303                      # 调度与机器人同步异常
int32 ERROR_CODE_304 = 304                      # 调度与机器人同步异常
int32 ERROR_CODE_305 = 305                      # 调度与机器人同步异常
int32 ERROR_CODE_351 = 351                      # 调度与机器人同步异常
int32 ERROR_CODE_390 = 390                      # 调度与机器人同步异常
int32 ERROR_CODE_455 = 455                      # 调度与机器人同步异常
int32 ERROR_CODE_545 = 545                      # 取货时，叉齿红外触发
int32 ERROR_CODE_BASE = 55000                   # 错误中间态
int32 ERROR_CODE_55001 = 55001                  # 打滑
int32 ERROR_CODE_55002 = 55002                  # 安全激光驱动故障
int32 ERROR_CODE_55020 = 55020                  # 电池电量极低
int32 ERROR_CODE_BREAK_IN = 55040               # 保护区遭闯入 
int32 ERROR_CODE_55060    = 55060               # 压力传感器连接丢失
int32 ERROR_CODE_55062    = 55062               # 压力传感器读数超过量程
int32 ERROR_CODE_55063    = 55063               # 高度测量超过限制

int32 ERROR_CODE_55070    = 55070               # 拉线传感器信号丢失
int32 ERROR_CODE_55072    = 55072               # 拉线传感器读数超过限值/高度超限
int32 ERROR_CODE_55073    = 55073               # 叉齿控制超时，一直没到达目标点
int32 ERROR_CODE_55074    = 55074               # 高度重复读数三次
int32 ERROR_CODE_55075    = 55075               # 反馈的高度比目标高度差超过可容忍误差值
int32 ERROR_CODE_55076    = 55076               # 任务下发高度超过车体叉齿运动范围，比如托盘摆放太偏了会引起这个问题
int32 ERROR_CODE_55078    = 55078               # 深度相机未识别出托盘
int32 ERROR_CODE_55079    = 55079               # 托盘摆放角度过大

int32 ERROR_CODE_55080    = 55080               # 与主控通讯超时
int32 ERROR_CODE_55090    = 55090               # 主动轮编码器读数异常
int32 ERROR_CODE_55091    = 55091               # 转向轮编码器读数异常
int32 ERROR_CODE_55110    = 55110               # 导航雷达和主控连接丢失
int32 ERROR_CODE_55122    = 55122               # 叉齿倾斜无法调整到小于预定的值
int32 ERROR_CODE_55130    = 55130               # AC控制器和接口板连接丢失
int32 ERROR_CODE_OUTSIDE_WORKZONE = 55204       # 车辆位于工作区外
int32 ERROR_CODE_ROBOT_POS_ERR    = 55205       # 车辆位置超差 根据任务状态。车辆实际到达的位置与系统指定的位置之间的偏差超过偏差限值，速度公差曲线
int32 ERROR_CODE_TURN_OVER        = 55206       # 转过了 
int32 ERROR_CODE_PATH_INVALID        = 55207       # 路径无效 
int32 ERROR_CODE_WIFI_SPOTTY      = 55302       # WIFI信号不稳定导致无法和上位机通讯
int32 ERROR_CODE_55330            = 55330       # 货物掉落 
int32 ERROR_CODE_55350            = 55350       # 目标点附近震荡
int32 ERROR_CODE_55352            = 55352       # 车辆实际运行速度低于设定的最低限值
int32 ERROR_CODE_55353            = 55353       # 车辆速度超过上限
int32 ERROR_CODE_EMERGENCY_SWITCH_TRIGGERED = 55380 # 紧急开关被按下
int32 ERROR_CODE_EMERGENCY_SWITCH_RELEASE   = 55381 # 恢复紧急开关
int32 ERROR_CODE_EMERGENCY_EDGE_CRASH       = 55382 # 触边
int32 ERROR_CODE_ANGLE_REPEAT               = 55383 # 反馈的角度连续为同一个值报错
int32 ERROR_CODE_KINCO_RESPONSE_ERROR       = 55384 # 电机返回值错误
int32 ERROR_CODE_KINCO_STATUS_ERROR         = 55385 # 电机状态异常 错误
int32 ERROR_CODE_KINCO_WIDTH_ERROR          = 55386 # 下发的电机宽度超过范围
int32 ERROR_CODE_RESUME = 55555                 # 错误恢复初始值,从其他线程传递给错误线程,不作为获取最终参考
int32 ERROR_CODE_MOVE_CONTROL_FAULT_ERROR_BASE = 55600 # 车体报错, 行走控制器故障
#/* 51  CAN通讯故障      52 转向系统故障          53 互锁开关操作顺序故障            54提升下降操作顺序故障            55手柄操作顺序故障
# 56泵控制器故障           57 AGV急停故障   踏板未踩报0x3A          59 CAN加速器CAN通讯故障               61 BMS通讯故障  
# 62BMS系统故障       63PLC CAN通讯故障       64 PLC 心跳故障         65  BMS保护故障(无心跳指令会报0x40/0x41)                66BMS低温故障 */
int32 ERROR_CODE_MOVE_CONTROL_FAULT_ERROR_END = 55753
int32 ERROR_CODE_END              = 55754

int32 ERROR_CODE_FORK       = 55754  #插齿升降错误 2022.5.18 suntao


int32 BMS_ERROR1_Low_Charging_Temperature       = 56001  #单体电池充电温度低故障
int32 BMS_ERROR1_High_Charging_Temperature       = 56002  #单体电池充电温度高故障
int32 BMS_ERROR1_Low_Discharge_Temperature       = 56003  #单体电池放电温度低故障
int32 BMS_ERROR1_High_Discharge_Temperature       = 56004  #单体电池放电温度高故障

int32 BMS_ERROR2_Low_Voltage       = 56005  #单体电压过低故障
int32 BMS_ERROR2_High_Voltage       = 56006  #单体电压过高故障
int32 BMS_ERROR2_Voltage_Difference       = 56007  #单体压差故障
int32 BMS_ERROR2_Temperature_Difference       = 56008  #单体温度差故障

int32 BMS_ERROR3_Low_Soc       = 56009  #soc过低故障
int32 BMS_ERROR3_High_Fast_Charging_Current       = 56010  #快充电流过大故障
int32 BMS_ERROR3_High_Feedback_Current       = 56011  #回馈电流过大故障
int32 BMS_ERROR3_High_Discharge_Current       = 56012  #放电电流过大故障

int32 BMS_ERROR4_Module_Loss_Of_Connection       = 56013  #模块失联故障
int32 BMS_ERROR4_Temperature_Sensor       = 56014  #单体温度传感器故障
int32 BMS_ERROR4_Low_Total_Voltage       = 56015  #总压过低故障
int32 BMS_ERROR4_High_Total_Voltage       = 56016  #总压过高故障

int32 BMS_ERROR5_Discharge_Relay       = 56017  #放电继电器故障
int32 BMS_ERROR5_High_Charging_Stand_Temperature       = 56018  #充电座温度过高故障
int32 BMS_ERROR5_Charging_Stand_Temperature_Sensor       = 56019  #充电座温度传感器故障


int32 Monomer_overvoltage       = 56020  #单体过压
int32  Unit_undervoltage   =56021 #单体欠压
int32  Total_voltage_overvoltage =56022  #总点压过压
int32  Total_voltage_undervoltage =56023#总电压欠压
int32  Large_monomer_pressure_difference =56024#单体压差大
int32  Discharge_overcurrent =56025#放电过流
int32  Charging_overcurrent  =56026#充电过流
int32  Temperature_too_high  =56027#温度过高
int32  Temperature_too_low =56028#温度过低
int32  Excessive_temperature_difference  =56029#温差过大
int32  SOC_too_low  =56030#soc过低
int32  Insulation_too_low  =56031#绝缘过低
int32  High_voltage_interlock_fault  =56032#高压互锁故障
int32  External_communication_failure  =56033#外部通讯故障
int32  Internal_communication_failure  =56034#内部通讯故障
int32  Contactor_1_adhesion =56035#接触器 1 黏连
int32 Contactor_2_adhesion =56036#接触器 2黏连
int32 Contactor_3_adhesion =56037#接触器 3黏连
int32 Contactor_4_adhesion =56038#接触器 3黏连

int32 safe_lidar_error =57001#安全激光错误
int32 car_tilting_error =57002#车体倾斜错误
int32 SAFE_LIDAR_LOW_POWER_ERROR =57003#安全激光LOW_POWER_ERROR
int32 SAFE_LIDAR_MOTOR_STALL_ERROR =57004#安全激光MOTOR_STALL_ERROR
int32 SAFE_LIDAR_TEMPERATURE_HIGH_ERROR =57005#安全激光TEMPERATURE_HIGH_ERROR
int32 SAFE_LIDAR_NETWORK_ERROR =57006#安全激光NETWORK_ERROR
int32 SAFE_LIDAR_NO_OUTPUT_ERROR =57007#安全激光NO_OUTPUT_ERROR
int32  Imu_data_timeout_error  =57008#imu数据超时
int32  lidar_data_timeout_error  =57009#lidar数据超时



int32  tray_fell_off_error  =58001 #托盘掉落
int32  Scan_failed   = 59001               # 扫码失败