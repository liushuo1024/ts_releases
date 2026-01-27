bool left_motor_enable
bool right_motor_enable
bool lift_motor_enable
bool rotate_motor_enable
float64 lift_v
float64 rotate_w
uint8 corning_led_type
uint8 three_color_led_type
int32 audio_control_type
int32 audio_control_level
int32 atomic_action_type
float64 atomic_action_value

# 三色灯控制类型
# enum class ThreeColorLedType : uint32_t {
#     GREEN_ON                 = 0x00000100,
#     YELLOW_ON                = 0x00000010,
#     RED_ON                   = 0x00000001,
    
#     YELLOW_GREEN_ON          = 0x00000110,
#     RED_YELLOW_ON            = 0x00000011,
#     RED_GREEN_ON             = 0x00000101,
#     RED_YELLOW_GREEN_ON      = 0x00000111,
    
#     RED_TWINKLE              = 0x10000001,
#     YELLOW_TWINKLE           = 0x10000010,
#     GREEN_TWINKLE            = 0x10000100,
    
#     YELLOW_GREEN_TWINKLE     = 0x10000110,
#     RED_YELLOW_TWINKLE       = 0x10000011,
#     RED_GREEN_TWINKLE        = 0x10000101,
#     RED_YELLOW_GREEN_TWINKLE = 0x10000111,
    
#     YELLOW_GREEN_SWITCH      = 0x20000110,
#     RED_YELLOW_SWITCH        = 0x20000011,
#     RED_GREEN_SWITCH         = 0x20000101,
    
#     LIGHT_ALL_ON             = 0x00000111,
#     LIGHT_OFF                = 0x00000000,
# }

# enum class AudioControlType : uint32_t {
#   STOP = 0,
#   BUMP_FAULT = 1,
#   FORWARD = 2,
#   BACKWARD = 3,
#   OBSTACLE_WARN = 4,
#   OBSTACLE_STOP = 5,
#   LOW_BATTERY = 6
# }

# enum class AtomicActionType : uint32_t {
#   REST = 0,
#   LOW_POWER = 2,         // 低功耗模式  (operation_value 填0(关), 1(开))
#   CHARGE = 3,            // 自动充电继电器 (operation_value 填0(关), 1(开))
#   ROATE_MOTOR_BREAK = 4, // 旋转电机报闸 (operation_value 填0(关), 1(开))
# };