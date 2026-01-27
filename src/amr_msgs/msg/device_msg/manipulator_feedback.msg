  time stamp   # 时间戳
  uint8 state  # 机械手运行状态 0: 空闲  1：正在执行任务
  uint8 error_code  # 报警码
  uint16 doing_task # 正在执行的任务唯一码
  uint16 done_task  # 已经完成的任务唯一码
  uint8 layer       # 机械手所在层级
  uint8 arm_1 # 伸出臂的位置  0:未伸出  1:伸出  2:中间位置
  uint8 arm_2 # 移动伸出臂的位置  0:未伸出  1:伸出  2:中间位置
  uint8 arm_3 # 移动抱臂的位置  0:缩回  1:放餐盒位置  2:中间位置