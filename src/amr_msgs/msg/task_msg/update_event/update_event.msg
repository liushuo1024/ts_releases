# 状态更新事件消息包  -- 心跳包  taskType: ForkUpdataEvent
string time     # 时间戳
string agv_id  # agv 设备的序列号
string node_id  # 空间间实体对象的ID，表示agv当前所处的位置
int32 point_id  # 目标点id
int32 plate_id  # 托盘编号
int32 plate_angle  # 托盘角度  -180 ~ 180
uint32 plate_weight # 托盘承重
float64 speed  # 当前速度 (单位)米为单位,精确到小数点后三位
float64 x  # agv 在空间中所处的x坐标 (单位)米为单位,精确到小数点后三位
float64 y  # agv 在空间中所处的y坐标 (单位)米为单位,精确到小数点后三位
float64 yaw  # agv 在坐标系中的偏航角 -180 ~ 180
float64 lift_height  # agv 当前托盘抬升高度，会随抬升过程而变化，单位米
int32 agv_status  # agv 状态, 双方约定枚举类型
int32 current_section_id  # 当前所在区域
int32 switch_section_id  # 需要切换的目标区域
amr_msgs/error_state[] error_code  # 错误码表: 错误编号 + 错误码
int32 battery  # 电池电量 0 ~ 100 %
int32 power_supply  # 电源 1：断电 2：有电
string task_type  # 当前正在处理的任务类型，与消息类型与消息体对象对照表对应
string order_id # 工单号
int32 task_sequence  # 当前正在处理的任务指令的序列号
int32 load_state     # 载货状态 0: 未知 1: 载货状态 2: 无货状态
int32 pallet_state   # 托盘上下限位状态 
int32 pallet_zero_state #托盘归零状态
float64 remain_distance # 当前任务剩余距离
bool request_task # 请求任务
int32 avoid_state # 避障状态 0：无避障 1:1级避障 2:2级避障 3:3级避障 4:避障超时

int32 end_point_sequence_id  # 当前任务串终点的sequence_id
int32 end_point_sequence_num # 当前任务串终点的sequence_num
int32 end_point_id # 当前任务串终点的point_id
float64 end_point_x  # 当前任务串终点x坐标 (单位)米为单位,精确到小数点后三位
float64 end_point_y  # 当前任务串终点y坐标 (单位)米为单位,精确到小数点后三位
float64 end_point_yaw  #当前任务串终点偏航角 -180 ~ 180 
