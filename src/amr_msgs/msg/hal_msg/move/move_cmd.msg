# 发送 (v, w) 给各模型
int8  cmd_velocity_direction   # Target speed direction  0: NONE  1:FORWARD  -1:BACKWARD
int8 mode # 0: 正常模式 1： 横移模式
float32 cmd_velocity
float32 cmd_omega
float32 curvature              # 路径曲率补偿