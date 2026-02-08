1. 使能can  
./can_enable.sh  
2. 启动can驱动  
roslaunch socketcan_bridge socketcan_bridge.launch  
3. 启动驱动节点  
roslaunch walk_motor_driver walk_motor_driver.launch  
4. 启动速度解算节点  
roslaunch vehicle_kinematics dual_kinematics.launch 
5. 启动键盘驱动  
roslaunch sim_control key_cmd.launch  