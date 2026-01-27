# 任务请求   taskType: ForkTaskRequest
string agv_id  # agv 设备的序列号
string order_id # 任务订单号
string task_id # 任务号
int32 task_type# 任务类型
int32 sequence_num
int32 step_size
bool  end_point_vaild  # 新老协议兼容 判断是否有taskEndpoint
float64 end_poinx_x   # 任务终点坐标x
float64 end_poinx_y   # 任务终点坐标y
float64 end_poinx_yaw # 任务终点坐标yaw
int32 end_point_operation_type  # 任务终点动作类型
float64 end_point_operation_value  # 任务终点动作值
amr_msgs/step[] step_list