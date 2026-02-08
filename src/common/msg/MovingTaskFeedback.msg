std_msgs/Header header

int64 running_id
int64 future_id
int64 last_done_id
int64 task_num

uint8 moving_mode  #运行模式
uint8 moving_mode_longitudinal=1  #运行模式:纵向
uint8 moving_mode_transverse=2  #运行模式:横向
uint8 moving_mode_rotation=3  #运行模式:原地旋转


uint8 direction
uint8 forward = 0      #前进
uint8 backward = 1     #后退

uint8 cw = 0      #顺时针
uint8 ccw = 1     #逆时针
float64 max_vel

uint8 safety_boundary
uint8 oblique_laser_mode
float64 curvature