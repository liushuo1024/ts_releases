uint8 avoid_io_state

# 上报优先级 不可恢复(防撞条) > 可恢复光电(防撞检测开关) > 可恢复避障区域(北洋避障区域)
# enum class Avoid_io_state : uint8_t {
#   NONE = 0,            // 无避障 
#   LEVEL_I = 1,         // 避障区域 避障等级1 - slowlevel1 
#   LEVEL_II = 2,        // 避障区域 避障等级2 - slowlevel2 
#   LEVEL_III = 3,       // 避障区域 避障等级3 - stop 
#   BUMP = 4,            // 防撞条 
#   FORK_LEFT_LEG = 5,   // 叉车左叉腿防撞 
#   FORK_RIGHT_LEG = 6,  // 叉车右叉腿防撞 
#   FORK_LEG_BOTH = 7    // 叉车左右叉腿都检测到 
#   }


