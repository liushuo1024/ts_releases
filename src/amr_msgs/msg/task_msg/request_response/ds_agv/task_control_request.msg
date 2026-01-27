int32 control_type

# 1 ~ 10停车相关  11 ~ 20对任务列表非正常更新操作
# enum class TaskControlType : int32_t {
#   NONE = 0,
#   STOP_RECOVER = 1,  // 恢复
#   PAUSE = 2,         // 暂停
#   EMERGENCY_CUT_OFF = 3,  // 紧急停车 
#   
#   TASK_CANCEL = 11,  // 取消任务
#   (reserve)
# } 