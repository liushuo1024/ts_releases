# 获取标定地图信息包  1~10 为地图信息标定, 11~20 为车体机械传感器安装等参数标定,  21~30 为运动参数标定
int32 map_id  # 标定地图编号
int32 section_id  # 分区编号
int32 calibration_mode  # 标定模式, 0:无  1:反光板标定, 2:slam地图标定 ...
int32 calibration_operation_type  # 标定操作 1:开启, 2:关闭, 3:本地保存
