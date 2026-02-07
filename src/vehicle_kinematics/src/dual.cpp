#include "vehicle_kinematics/dual.h"

Dual::Dual(ros::NodeHandle &n, ros::NodeHandle &pn)
    : odom_first_flg(true), sum_theta(0.0), sum_x(0.0), sum_y(0.0), omv_calibrate(false)
{

    nh_ = n;
    nh_priv_ = pn;
    //舵轮模型相关参数
    // 减速比
    ros::param::get("~k_red_ratio", k_red_ratio);
    // 舵轮的初始半径
    ros::param::get("~radius_steering_wheel", radius_steering_wheel);
    // 单位:米, 舵轮y方向偏距
    ros::param::get("~L_base", offset_distance_wheel_x);
    // 单位:米, 舵轮x方向偏距
    ros::param::get("~L_base_y", offset_distance_wheel_y);

    //差分模型相关参数
    // 单位:米, 轮距
    ros::param::get("~wheel_track", wheel_track);
    // 左轮半径
    ros::param::get("~radius_virtual_left_wheel", radius_virtual_left_wheel);
    // 右轮半径
    ros::param::get("~radius_virtual_right_wheel", radius_virtual_right_wheel);

    // 差分模型固定参数
    // 单位:米, 轮距
    ros::param::get("~wheel_track_theory_const", wheel_track_theory_const);
    // 左轮半径
    ros::param::get("~radius_virtual_left_wheel_const", radius_virtual_left_wheel_const);
    // 右轮半径
    ros::param::get("~radius_virtual_right_wheel_const", radius_virtual_right_wheel_const);

    ros::param::get("~pub_lidar_tf", pub_lidar_tf);

    ros::param::get("~omv_calibrate", omv_calibrate);

    double L_passive_base;
    double L_passive_steer;
    double L_passive_tread;

    ros::param::get("~L_passive_base", L_passive_base);
    ros::param::get("~L_base", L_passive_steer);
    ros::param::get("~L_passive_tread", L_passive_tread);
    ros::param::get("~max_wheel_speed", max_wheel_speed_);
    ros::param::get("~max_steer_angle", max_steer_angle_);
    if (!omv_calibrate)
        offset_distance_wheel_x = offset_distance_wheel_x - L_passive_base;

    left_wheel_p << L_passive_base, L_passive_tread / 2.0;
    right_wheel_p << -L_passive_base, -L_passive_tread / 2.0;
    steering_wheel_p << L_passive_steer - L_passive_base, 0;
    std::cout << "left_wheel_pose:" << left_wheel_p << std::endl;
    std::cout << "right_wheel_ppse:" << right_wheel_p << std::endl;
    std::cout << "steering_wheel_p:" << steering_wheel_p << std::endl;

    if (!omv_calibrate)
        MovingTaskFeedback_sub_ = nh_.subscribe("/moving_task_feedback", 1, &Dual::MovingTaskFeedback_callback, this);

    if (pub_lidar_tf)
    {
        ros::param::get("~config_file", config_file_path);
        std::string config_file = config_file_path + "robot_tf.launch.xml";
        std::cout << "config_file" << config_file << std::endl;

        tinyxml2::XMLDocument xml;
        const char *filename = config_file.c_str();
        xml.LoadFile(filename);

        tinyxml2::XMLElement *launch_file = xml.RootElement();
        if (launch_file == NULL)
        {
            std::cout << "Failed to load file: No root element." << std::endl;
            // 清理内存
            xml.Clear();
            return;
        }

        for (auto *elem = launch_file->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
        {
            std::string elemName = elem->Value();
            const char *attr;
            attr = elem->Attribute("name");
            if (strcmp(attr, "base_link_to_laser") == 0)
            {
                const char *attr;
                attr = elem->Attribute("args");
                std::string attr_s = attr;
                std::cout << "args:" << attr_s << std::endl;
                std::vector<std::string> sub_s;
                boost::trim_if(attr_s, boost::is_any_of("\t "));
                boost::split(sub_s, attr_s, boost::is_any_of("\t "), boost::token_compress_on);

                if (sub_s.size() == 8)
                {
                    base_to_lidar_x = stod(sub_s[0]);
                    base_to_lidar_y = stod(sub_s[1]);
                    base_to_lidar_yaw = stod(sub_s[3]);
                    std::cout << "base_to_lidar_x" << base_to_lidar_x << std::endl;
                    std::cout << "base_to_lidar_y" << base_to_lidar_y << std::endl;
                    std::cout << "base_to_lidar_yaw" << base_to_lidar_yaw << std::endl;
                }
            }
        }
    }

    ac_cmd_pub_ = nh_.advertise<common::omv_servo_cmd>("/omv_servo_cmd", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
    auto_calibrate_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/auto_calibrate_odom", 10);
    move_feedback_pub_ = nh_.advertise<amr_msgs::move_feedback>(
        "/move_feedback", 10);
    servo_encoder_sub_ = nh_.subscribe("/omv_servo_encoder", 1, &Dual::recv_omv_servo_encoder, this, ros::TransportHints().tcpNoDelay(true));

    if (omv_calibrate)
    {
        remote_ctrl_pub = nh_.advertise<common::remote_ctrl_mode>("/remote_ctrl_mode", 2);
        vehicle_cmd_sub_ = nh_.subscribe("/vehicle_cmd", 1, &Dual::recv_vehicle_cmd, this);
    }
    move_cmd_sub_ = nh_.subscribe("/move_cmd", 1, &Dual::recv_move_cmd, this);
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    omv_moving_state = 1;

    if (omv_calibrate)
        omv_moving_state = 1;
    
    // while (ros::ok())
    // {
    //     geometry_msgs::Twist twist;
    //     twist.linear.x = 0.0;
    //     twist.linear.y = 0.1;
    //     twist.angular.z = 0.01;
    //     auto twist_msg = boost::make_shared<geometry_msgs::Twist>(twist);
    //     recv_vehicle_cmd(twist_msg);
    //     sleep(0.1);
    // }
    

    ros::spin();
}

Dual::~Dual()
{
}

void Dual::MovingTaskFeedback_callback(const common::MovingTaskFeedback::ConstPtr &msg)
{
    omv_moving_state = msg->moving_mode;
}
int Dual::cal_theta(Vector2d start_p, Vector2d end_p, double &theta)
{
    if (start_p == end_p)
        return -1;
    else
        theta = atan2(end_p(1) - start_p(1), end_p(0) - start_p(0));
    return 0;
}
int Dual::recover_start_y(const Vector2d& end_p, double theta, double& start_y) {
    const double eps = 1e-9;
    if (std::abs(end_p(0)) < eps) {
        // 无法唯一确定 start_y，因为方向是垂直的
        // 可根据实际需求处理：比如设为 end_p(1) - 1，或报错
        return -1;
    }
    start_y = end_p(1) - end_p(0) * std::tan(theta);
    return 0;
}
int sign(double x)
{
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    else
        return 0;
}
void Dual::recv_move_cmd(const amr_msgs::move_cmd::ConstPtr &msg)
{
    // std::cout <<"recv_vehicle_cmd" <<std::endl;
    double v = msg->cmd_velocity;
    double omega = msg->cmd_omega;
    moving_mode_ = msg->mode;
    geometry_msgs::Twist twist;
    twist.linear.x = v;
    twist.angular.z = omega;
    if(moving_mode_ == 1){
        twist.linear.x = 0;
        twist.linear.y = msg->cmd_velocity;
        twist.angular.z = omega;
    }
    auto twist_msg = boost::make_shared<geometry_msgs::Twist>(twist);
    recv_vehicle_cmd(twist_msg);
}
/**
 * @brief 逆运动学解算：车体速度 -> 舵轮角度+速度
 * @param vx 车体x方向速度（前向，m/s）
 * @param vy 车体y方向速度（横向，m/s）
 * @param omega 车体角速度（rad/s）
 * @param wheel_pos 舵轮安装位置（车体坐标系）
 * @param steer_angle 输出：舵轮转向角（rad）
 * @param wheel_speed 输出：舵轮线速度（m/s）
 */
void Dual::inverse_kinematics(double vx, double vy, double omega,
                              const Eigen::Vector2d& wheel_pos,
                              double& steer_angle, double& wheel_speed)
{
    // 1. 轮子中心期望速度（车体坐标系）
    double v_wheel_x = vx - omega * wheel_pos.y();
    double v_wheel_y = vy + omega * wheel_pos.x();

    // 2. 期望角度 & 速度
    steer_angle = std::atan2(v_wheel_y, v_wheel_x);
    wheel_speed   = std::hypot(v_wheel_x, v_wheel_y);

    // 3. 根据“模式”决定允许转角区间
    bool longitudinal_mode = (std::fabs(vy) < 1e-3);   // 纯前后运动
    double lo, hi;                                       // 允许区间 [lo, hi]
    if (longitudinal_mode)
    {
        lo = -M_PI_2;   // -90°
        hi =  M_PI_2;   // +90°
    }
    else
    {
        lo = -M_PI;     // -180°
        hi =  0.0;      //   0°
    }

    // 4. 角度落到允许区间里
    if (steer_angle < lo || steer_angle > hi)
    {
        steer_angle += M_PI;        // 转180°
        wheel_speed   = -wheel_speed; // 速度反向
    }

    // 5. 归一化到 [-π,π]
    steer_angle = regular_theta(steer_angle);

    // 6. 机械限幅（最后保险）
    steer_angle = clamp(steer_angle, -max_steer_angle_, max_steer_angle_);
    wheel_speed = clamp(wheel_speed,  -max_wheel_speed_, max_wheel_speed_);

    // 7. 原地旋转特殊处理（保持你原来的逻辑）
    if (std::fabs(vx) < 1e-5 && std::fabs(vy) < 1e-5 && std::fabs(omega) > 1e-5)
    {
        steer_angle = std::atan2(-wheel_pos.x(), wheel_pos.y());
        wheel_speed = -omega * std::hypot(wheel_pos.x(), wheel_pos.y());
    }
}
void Dual::recv_vehicle_cmd(const geometry_msgs::Twist::ConstPtr &msg)
{
    ROS_INFO("inpute vx=%.6f m/s, vy=%.6f m/s omega=%.6f", msg->linear.x, msg->linear.y, msg->angular.z);
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double omega = msg->angular.z;

    ROS_INFO("vx=%.6f m/s, vy=%.6f m/s omega=%.6f", vx, vy, omega);

    // 解算左舵轮角度和速度
    double left_steer, left_speed;
    inverse_kinematics(vx, vy, omega, left_wheel_p, left_steer, left_speed);

    // 解算右舵轮角度和速度
    double right_steer, right_speed;
    inverse_kinematics(vx, vy, omega, right_wheel_p, right_steer, right_speed);

    // 转换为舵轮控制指令（rpm + 角度）
    double rpm_left = (30 * left_speed * k_red_ratio) / (M_PI * radius_steering_wheel);
    double rpm_right = (30 * right_speed * k_red_ratio) / (M_PI * radius_steering_wheel);
    double left_steer_deg = left_steer * 180 / M_PI;
    double right_steer_deg = right_steer * 180 / M_PI;

    // 发布控制指令
    common::omv_servo_cmd osc;
    osc.header.stamp = ros::Time::now();
    osc.header.frame_id = "odom";
    osc.sc_left_theta = left_steer_deg;
    osc.sc_right_theta = right_steer_deg;
    osc.sc_left_vel = rpm_left;
    osc.sc_right_vel = rpm_right;
    if(protection_){
        osc.sc_left_theta = 0;
        osc.sc_right_theta = 0;
        osc.sc_left_vel = 0;
        osc.sc_right_vel = 0;
        std::cout << "Trigger protection, please contact the developer" << std::endl;
    }
    ac_cmd_pub_.publish(osc);

    // 调试输出
    ROS_INFO("Left wheel: steer=%.2f deg, speed=%.2f m/s (rpm=%.1f)",
              left_steer_deg, left_speed, rpm_left);
    ROS_INFO("Right wheel: steer=%.2f deg, speed=%.2f m/s (rpm=%.1f)",
              right_steer_deg, right_speed, rpm_right);

}

double Dual::regular_theta(double theta)
{
    while (theta > M_PI)
        theta -= 2 * M_PI;
    while (theta < -M_PI)
        theta += 2 * M_PI;
    return theta;
}
void Dual::cal_odom(double offset_distance_wheel_x, 
    double offset_distance_wheel_y, 
    double v_T,double theta_2odom,
    double &v,
    double &omega)
{
    v = v_T * cos(theta_2odom) + v_T * offset_distance_wheel_y * sin(theta_2odom) / (offset_distance_wheel_x);
    omega = v_T * sin(theta_2odom) / offset_distance_wheel_x;       
}
void Dual::cal_cmd_vel(double offset_distance_wheel_x, 
    double offset_distance_wheel_y, 
    double v,double omega,
    double theta_2odom,
    double &v_T)
{
    v_T = v/(cos(theta_2odom) + offset_distance_wheel_y * sin(theta_2odom) / (offset_distance_wheel_x));       
}
void Dual::recv_omv_servo_encoder(const common::omv_servo_encoder::ConstPtr &msg)
{
    double L = 1.3 * 0.5;
    double L_y = 0.5;

    // left
    double theta_2odom_left = (msg->se_left_theta * M_PI) / 180.0f;
    double rpm_2odom_left = msg->se_left_vel;
    // 将接收到的角度映射到(-pi,pi)之间
    while (theta_2odom_left > M_PI)
        theta_2odom_left -= 2 * M_PI;
    while (theta_2odom_left <= -M_PI)
        theta_2odom_left += 2 * M_PI;
    double v_T_left = (2 * M_PI * rpm_2odom_left * radius_steering_wheel) / (60 * k_red_ratio);

    // right
    double theta_2odom_right = (msg->se_right_theta * M_PI) / 180.0f;
    double rpm_2odom_right = msg->se_right_vel;
    // 将接收到的角度映射到(-pi,pi)之间
    while (theta_2odom_right > M_PI)
        theta_2odom_right -= 2 * M_PI;
    while (theta_2odom_right <= -M_PI)
        theta_2odom_right += 2 * M_PI;
    double v_T_right = (2 * M_PI * rpm_2odom_right * radius_steering_wheel) / (60 * k_red_ratio);
    
    std::cout << "rpm_2odom_left: " << rpm_2odom_left << std::endl;
    std::cout << "rpm_2odom_right: " << rpm_2odom_right << std::endl;
    std::cout << "se v_T_left:" << v_T_left << std::endl;
    std::cout << "se v_T_right:" << v_T_right << std::endl;
    std::cout << "theta_2odom_left:" << theta_2odom_left << std::endl;
    std::cout << "theta_2odom_right:" << theta_2odom_right << std::endl;
    double v_2odom = 0;
    double omega_2odom = 0;
    if (omv_moving_state == 1)
    {
        double v_from_left;
        double omega_from_left;
        cal_odom(L, L_y, v_T_left, theta_2odom_left, v_from_left, omega_from_left);

        double v_from_right;
        double omega_from_right;
        cal_odom(-L, -L_y, v_T_right, theta_2odom_right, v_from_right, omega_from_right);

        double v = (v_from_left + v_from_right) / 2;
        double omega = (omega_from_left + omega_from_right) / 2;
        std::cout << "se v:" << v << std::endl;
        std::cout << "se omega:" << omega << std::endl;
        v_2odom = v;
        omega_2odom = omega;
    }
    // else if (omv_moving_state == 2)
    // {
    //     double theta_left_2odom = (msg->se_left_theta * M_PI) / 180.0f;
    //     while (theta_left_2odom > M_PI)
    //         theta_left_2odom -= 2 * M_PI;
    //     while (theta_left_2odom <= -M_PI)
    //         theta_left_2odom += 2 * M_PI;
    //     if (fabs(regular_theta(theta_left_2odom + M_PI / 2)) < 0.05)
    //     {
    //         v_2odom = v_T;
    //         omega_2odom = 0;
    //     }
    //     else if (fabs(regular_theta(theta_left_2odom - M_PI / 2)) < 0.05)
    //     {
    //         v_2odom = v_T;
    //         omega_2odom = 0;
    //     }
    //     else
    //     {
    //         std::cout << "theta:" << theta_left_2odom << std::endl;
    //         double sin_ = sin(theta_left_2odom);
    //         double cos_ = cos(theta_left_2odom);
    //         double center_x = left_wheel_p(1) * sin_ / cos_ - fabs(left_wheel_p(0));
    //         std::cout << "center_x:" << center_x << std::endl;
    //         if (fabs(center_x - steering_wheel_p(0)) < 0.4)
    //         {
    //             v_2odom = 0;
    //             omega_2odom = 0;
    //         }
    //         else
    //         {
    //             omega_2odom = v_T / (center_x - steering_wheel_p(0));
    //             v_2odom = omega_2odom * center_x;
    //         }
    //     }
    // }
    // else if (omv_moving_state == 3)
    // {
    //     v_2odom = 0;
    //     omega_2odom = v_T / steering_wheel_p(0);
    // }

    std::cout << "moving type:" << omv_moving_state << std::endl;
    std::cout << "v:" << v_2odom << std::endl;
    std::cout << "omega:" << omega_2odom << std::endl;

    if (odom_first_flg)
    {
        current_time = msg->header.stamp;
        odom_first_flg = false;
    }
    else
    {
        current_time = msg->header.stamp;
        double delta_time = (current_time - last_time).toSec();

        // 舵轮x,y方向速度分量为
        double v_x = 0;
        double v_y = 0;
        if (omv_moving_state == 1)
        {
            v_x = v_2odom * cos(sum_theta);
            v_y = v_2odom * sin(sum_theta);
        }
        else if (omv_moving_state == 2)
        {
            v_x = v_2odom * sin(sum_theta);
            v_y = -v_2odom * cos(sum_theta);
        }

        // 根据速度计算累计的位置和方向角偏差

        double delta_theta = omega_2odom * delta_time;
        double delta_x = v_x * delta_time;
        double delta_y = v_y * delta_time;

        sum_x += delta_x;
        sum_y += delta_y;

        sum_theta += delta_theta;

        while (sum_theta > M_PI)
            sum_theta -= 2 * M_PI;
        while (sum_theta <= -M_PI)
            sum_theta += 2 * M_PI;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(sum_theta);
        if (pub_lidar_tf)
        {
            lidar_global_x = sum_x + base_to_lidar_x * cos(sum_theta) - base_to_lidar_y * sin(sum_theta) - base_to_lidar_x;
            lidar_global_y = sum_y + base_to_lidar_y * cos(sum_theta) + base_to_lidar_x * sin(sum_theta) - base_to_lidar_y;

            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_footprint";

            odom_trans.transform.translation.x = lidar_global_x;
            odom_trans.transform.translation.y = lidar_global_y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            // send the transform
            odom_bd.sendTransform(odom_trans);
        }
        else
        {
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_footprint";

            odom_trans.transform.translation.x = sum_x;
            odom_trans.transform.translation.y = sum_y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            // send the transform
            // odom_bd.sendTransform(odom_trans);
        }

        // 发布 Odometry Topic
        nav_msgs::Odometry odom;
        odom.header.stamp = msg->header.stamp;
        odom.header.frame_id = "odom";

        // 设定 Position
        if (pub_lidar_tf)
        {
            odom.pose.pose.position.x = lidar_global_x;
            odom.pose.pose.position.y = lidar_global_y;
        }
        else
        {
            odom.pose.pose.position.x = sum_x;
            odom.pose.pose.position.y = sum_y;
        }
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.pose.covariance[35] = 0.05;

        // 设定 Velocity
        odom.child_frame_id = "base_footprint";
        if (pub_lidar_tf)
        {
            odom.twist.twist.linear.x = v_x - omega_2odom * base_to_lidar_y;
            odom.twist.twist.linear.y = v_y + omega_2odom * base_to_lidar_x;
        }
        else
        {
            odom.twist.twist.linear.x = v_x;
            odom.twist.twist.linear.y = v_y;
        }

        odom.twist.twist.angular.z = omega_2odom;

        odom_pub_.publish(odom);

        // 发布 auto calibrate odometry topic

        nav_msgs::Odometry auto_calibrate_odom;
        auto_calibrate_odom.header.stamp = msg->header.stamp;
        auto_calibrate_odom.header.frame_id = "odom";

        // 设定 Position
        auto_calibrate_odom.pose.pose.position.x = sum_x;
        auto_calibrate_odom.pose.pose.position.y = sum_y;
        auto_calibrate_odom.pose.pose.position.z = 0.0;
        auto_calibrate_odom.pose.pose.orientation = odom_quat;
        auto_calibrate_odom.pose.covariance[35] = 0.05;

        // 设定 Velocity
        auto_calibrate_odom.child_frame_id = "base_footprint";
        auto_calibrate_odom.twist.twist.linear.x = v_2odom;
        auto_calibrate_odom.twist.twist.linear.y = 0;
        auto_calibrate_odom.twist.twist.angular.z = omega_2odom;

        auto_calibrate_odom_pub_.publish(auto_calibrate_odom);
        
        amr_msgs::move_feedback fb_msg;
        // fb_msg.header.stamp = msg->header.stamp;
        fb_msg.actual_omega = omega_2odom;
        fb_msg.actual_velocity = v_2odom;
        fb_msg.virtual_velocity = v_2odom;
        move_feedback_pub_.publish(fb_msg);
        last_time = current_time;
        last_time = current_time;
        if(std::fabs(odom.pose.pose.position.x) > 50 ||
        std::fabs(odom.pose.pose.position.y) > 50 ||
        std::fabs(odom.twist.twist.linear.x) > 1.0 ||
        std::fabs(odom.twist.twist.linear.y) > 1.0
        ){
            protection_ = true;
        }
        
    }
}
