#include "vehicle_kinematics/single_servo.h"
#include "amr_msgs/move_feedback.h"

int SingleServo::get_param(ros::NodeHandle& nh_priv){
    bool result;
    result = nh_priv.getParam("k_red_ratio", k_red_ratio);
    if(!result) {
        ROS_ERROR("Parameter %s not found","k_red_ratio");
        return -1;
    }
    result = nh_priv.getParam("radius_steering_wheel", radius_steering_wheel);
    if(!result) {
        ROS_ERROR("Parameter %s not found","radius_steering_wheel");
        return -1;
    }
    result = nh_priv.getParam("L_base", offset_distance_wheel_x);
    if(!result) {
        ROS_ERROR("Parameter %s not found","L_base");
        return -1;
    }
    result = nh_priv.getParam("L_base_y", offset_distance_wheel_y);
    if(!result) {
        ROS_ERROR("Parameter %s not found","L_base_y");
        return -1;
    }
    result = nh_priv.getParam("wheel_track", wheel_track);
    if(!result) {
        ROS_ERROR("Parameter %s not found","wheel_track");
        return -1;
    }
    result = nh_priv.getParam("radius_virtual_left_wheel", radius_virtual_left_wheel);
    if(!result) {
        ROS_ERROR("Parameter %s not found","fre");
        return -1;
    }
    result = nh_priv.getParam("radius_virtual_right_wheel", radius_virtual_right_wheel);
    if(!result) {
        ROS_ERROR("Parameter %s not found","radius_virtual_right_wheel");
        return -1;
    }
    result = nh_priv.getParam("wheel_track_theory_const", wheel_track_theory_const);
    if(!result) {
        ROS_ERROR("Parameter %s not found","wheel_track_theory_const");
        return -1;
    }
    result = nh_priv.getParam("radius_virtual_right_wheel_const", radius_virtual_right_wheel_const);
    if(!result) {
        ROS_ERROR("Parameter %s not found","radius_virtual_right_wheel_const");
        return -1;
    }
    result = nh_priv.getParam("radius_virtual_left_wheel_const", radius_virtual_left_wheel_const);
    if(!result) {
        ROS_ERROR("Parameter %s not found","k_passive_red_ratio");
        return -1;
    }
    result = nh_priv.getParam("pub_lidar_tf", pub_lidar_tf);
    if(!result) {
        ROS_ERROR("Parameter %s not found","pub_lidar_tf");
        return -1;
    }
    result = nh_priv.getParam("omv_equivalent", omv_equivalent);
    if(!result) {
        ROS_ERROR("Parameter %s not found","omv_equivalent");
        return -1;
    }
    result = nh_priv.getParam("pub_odom", pub_odom);
    if(!result) {
        ROS_ERROR("Parameter %s not found","pub_odom");
        return -1;
    }
    result = nh_priv.getParam("use_imu", use_imu);
    if(!result) {
        ROS_ERROR("Parameter %s not found","use_imu");
        return -1;
    }
}

SingleServo::SingleServo(ros::NodeHandle &n, ros::NodeHandle &pn)
    : odom_first_flg(true), sum_theta(0.0), sum_x(0.0), sum_y(0.0), omv_equivalent(false)
{
    nh_ = n;
    nh_priv_ = pn;
    get_param(nh_priv_);
    //舵轮模型相关参数
    // 减速比
    // ros::param::get("~k_red_ratio", k_red_ratio);
    // // 舵轮的初始半径
    // ros::param::get("~radius_steering_wheel", radius_steering_wheel);
    // // 单位:米, 舵轮y方向偏距
    // ros::param::get("~L_base", offset_distance_wheel_x);
    // // 单位:米, 舵轮x方向偏距
    // ros::param::get("~L_base_y", offset_distance_wheel_y);

    // //差分模型相关参数
    // // 单位:米, 轮距
    // ros::param::get("~wheel_track", wheel_track);
    // // 左轮半径
    // ros::param::get("~radius_virtual_left_wheel", radius_virtual_left_wheel);
    // // 右轮半径
    // ros::param::get("~radius_virtual_right_wheel", radius_virtual_right_wheel);
    // // 差分模型固定参数
    // // 单位:米, 轮距
    // ros::param::get("~wheel_track_theory_const", wheel_track_theory_const);
    // // 左轮半径
    // ros::param::get("~radius_virtual_left_wheel_const", radius_virtual_left_wheel_const);
    // // 右轮半径
    // ros::param::get("~radius_virtual_right_wheel_const", radius_virtual_right_wheel_const);
    // ros::param::get("~pub_lidar_tf", pub_lidar_tf);
    // ros::param::get("~omv_equivalent", omv_equivalent);
    // ros::param::get("~pub_odom", pub_odom);

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

    if (omv_equivalent)
        ac_cmd_pub_ = nh_.advertise<common::omv_servo_cmd>("/servo_cmd", 10);
    else
        ac_cmd_pub_ = nh_.advertise<common::servo_cmd>("/servo_cmd", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
    auto_calibrate_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/auto_calibrate_odom", 10);
    move_feedback_pub_ = nh_.advertise<amr_msgs::move_feedback>(
        "/move_feedback", 10);
    if (omv_equivalent)
        servo_encoder_sub_ = nh_.subscribe("/servo_encoder", 1, &SingleServo::recv_omv_servo_encoder, this, ros::TransportHints().tcpNoDelay(true));
    else
        servo_encoder_sub_ = nh_.subscribe("/servo_encoder", 1, &SingleServo::recv_servo_encoder, this, ros::TransportHints().tcpNoDelay(true));

    imu_sub_ = nh_.subscribe("/imu", 1, &SingleServo::imu_callback, this);
    vehicle_cmd_sub_ = nh_.subscribe("/vehicle_cmd", 1, &SingleServo::recv_vehicle_cmd, this);
    move_cmd_sub_ = nh_.subscribe("/move_cmd", 1, &SingleServo::recv_move_cmd, this);
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::spin();
}

SingleServo::~SingleServo()
{
}
void SingleServo::recv_move_cmd(const amr_msgs::move_cmd::ConstPtr &msg)
{
    // std::cout <<"recv_vehicle_cmd" <<std::endl;
    double v = msg->cmd_velocity;
    double omega = msg->cmd_omega;
    if(std::fabs(omega) > 0.2) omega = omega > 0 ? 0.2 : -0.2;
    // std::cout << "recv_vehicle_cmd v " << v << " omega: " << omega << std::endl;
    double v_r = v + omega * wheel_track / 2.0f;
    double v_l = v - omega * wheel_track / 2.0f;
    // std::cout << "recv_vehicle_cmd v_r: " << v_r << "  v_l: " << v_l << std::endl;
    double omega_r = v_r / radius_virtual_right_wheel;
    double omega_l = v_l / radius_virtual_left_wheel;
    // std::cout << "recv_vehicle_cmd omega_r: " << omega_r << "  omega_l: " << omega_l << std::endl;
    double v_r_0 = omega_r * radius_virtual_right_wheel_const;
    double v_l_0 = omega_l * radius_virtual_left_wheel_const;
//    std::cout << "recv_vehicle_cmd v_r_0: " << v_r_0 << "  v_l_0: " << v_l_0 << std::endl;
    v = 0.5f * (v_r_0 + v_l_0);
    omega = (v_r_0 - v_l_0) / wheel_track_theory_const;

//    std::cout << "recv_vehicle_cmd 2 v " << v << " omega: " << omega << std::endl;

    double v_t;
    double theta_2ac;
    if (fabs(omega) < 1e-5)
    {
        omega = 0;
        v_t = v;
        theta_2ac = 0;
    }
    else
    {
        double r_1 = v / omega;
        theta_2ac = atan2(offset_distance_wheel_x, r_1 - offset_distance_wheel_y);
        // std::cout << "recv_vehicle_cmd theta_2ac " << theta_2ac <<std::endl;
        if (theta_2ac > M_PI / 2)
            theta_2ac = theta_2ac - M_PI;
        // std::cout << "recv_vehicle_cmd r_1 " << r_1 <<std::endl;
        // std::cout << "recv_vehicle_cmd offset_distance_wheel_y " << offset_distance_wheel_y <<std::endl;
        // std::cout << "recv_vehicle_cmd offset_distance_wheel_x " << offset_distance_wheel_x <<std::endl;
        v_t = omega * sqrt(pow(r_1 - offset_distance_wheel_y, 2) + pow(offset_distance_wheel_x, 2));
        if (theta_2ac < 0)
            v_t = -v_t;
    }
    // std::cout << "recv_vehicle_cmd v_t " << v_t <<std::endl;
    // std::cout << "recv_vehicle_cmd k_red_ratio " << k_red_ratio << "radius_steering_wheel" << radius_steering_wheel <<std::endl;
    double rpm_2ac = (30 * v_t * k_red_ratio) / (M_PI * radius_steering_wheel);

    current_time = ros::Time::now();
    if (omv_equivalent)
    {
        common::omv_servo_cmd osc;
        osc.header.stamp = current_time;
        osc.header.frame_id = "odom";
        osc.sc_main_theta = (theta_2ac * 180) / M_PI;
        osc.sc_vel = rpm_2ac;
        osc.sc_left_theta = 0;
        osc.sc_right_theta = 0;

        ac_cmd_pub_.publish(osc);
    }
    else
    {
        common::servo_cmd ac;
        ac.header.stamp = current_time;
        ac.header.frame_id = "odom";
        ac.sc_theta = (theta_2ac * 180) / M_PI;
        ac.sc_vel = rpm_2ac;
        std::cout << "ac.sc_vel " << ac.sc_vel << "          ac.sc_theta" << ac.sc_theta <<std::endl;
        ac_cmd_pub_.publish(ac);
    }
}
void SingleServo::recv_vehicle_cmd(const geometry_msgs::Twist::ConstPtr &msg)
{
    // std::cout <<"recv_vehicle_cmd" <<std::endl;
    double v = msg->linear.x;
    double omega = msg->angular.z;
    // std::cout << "recv_vehicle_cmd v " << v << " omega: " << omega << std::endl;
    double v_r = v + omega * wheel_track / 2.0f;
    double v_l = v - omega * wheel_track / 2.0f;
    // std::cout << "recv_vehicle_cmd v_r: " << v_r << "  v_l: " << v_l << std::endl;
    double omega_r = v_r / radius_virtual_right_wheel;
    double omega_l = v_l / radius_virtual_left_wheel;
    // std::cout << "recv_vehicle_cmd omega_r: " << omega_r << "  omega_l: " << omega_l << std::endl;
    double v_r_0 = omega_r * radius_virtual_right_wheel_const;
    double v_l_0 = omega_l * radius_virtual_left_wheel_const;
//    std::cout << "recv_vehicle_cmd v_r_0: " << v_r_0 << "  v_l_0: " << v_l_0 << std::endl;
    v = 0.5f * (v_r_0 + v_l_0);
    omega = (v_r_0 - v_l_0) / wheel_track_theory_const;

//    std::cout << "recv_vehicle_cmd 2 v " << v << " omega: " << omega << std::endl;

    double v_t;
    double theta_2ac;
    if (fabs(omega) < 1e-5)
    {
        omega = 0;
        v_t = v;
        theta_2ac = 0;
    }
    else
    {
        double r_1 = v / omega;
        theta_2ac = atan2(offset_distance_wheel_x, r_1 - offset_distance_wheel_y);
        // std::cout << "recv_vehicle_cmd theta_2ac " << theta_2ac <<std::endl;
        if (theta_2ac > M_PI / 2)
            theta_2ac = theta_2ac - M_PI;
        // std::cout << "recv_vehicle_cmd r_1 " << r_1 <<std::endl;
        // std::cout << "recv_vehicle_cmd offset_distance_wheel_y " << offset_distance_wheel_y <<std::endl;
        // std::cout << "recv_vehicle_cmd offset_distance_wheel_x " << offset_distance_wheel_x <<std::endl;
        v_t = omega * sqrt(pow(r_1 - offset_distance_wheel_y, 2) + pow(offset_distance_wheel_x, 2));
        if (theta_2ac < 0)
            v_t = -v_t;
    }
    // std::cout << "recv_vehicle_cmd v_t " << v_t <<std::endl;
    // std::cout << "recv_vehicle_cmd k_red_ratio " << k_red_ratio << "radius_steering_wheel" << radius_steering_wheel <<std::endl;
    double rpm_2ac = (30 * v_t * k_red_ratio) / (M_PI * radius_steering_wheel);

    current_time = ros::Time::now();
    if (omv_equivalent)
    {
        common::omv_servo_cmd osc;
        osc.header.stamp = current_time;
        osc.header.frame_id = "odom";
        osc.sc_main_theta = (theta_2ac * 180) / M_PI;
        osc.sc_vel = rpm_2ac;
        osc.sc_left_theta = 0;
        osc.sc_right_theta = 0;

        ac_cmd_pub_.publish(osc);
    }
    else
    {
        common::servo_cmd ac;
        ac.header.stamp = current_time;
        ac.header.frame_id = "odom";
        ac.sc_theta = (theta_2ac * 180) / M_PI;
        ac.sc_vel = rpm_2ac;
        std::cout << "ac.sc_vel " << ac.sc_vel << "          ac.sc_theta" << ac.sc_theta <<std::endl;
        ac_cmd_pub_.publish(ac);
    }
}

void SingleServo::recv_omv_servo_encoder(const common::omv_servo_encoder::ConstPtr &msg)
{
    boost::shared_ptr<common::servo_encoder> se(new common::servo_encoder());
    // common::servo_encoder se;
    se->se_theta = msg->se_main_theta;
    se->se_vel = msg->se_vel;
    recv_servo_encoder(se);
}
void SingleServo::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_angular_velocity_z_ = msg->angular_velocity.z;
}
void SingleServo::recv_servo_encoder(const common::servo_encoder::ConstPtr &msg)
{
    // std::cout<<"recv encoder"<<std::endl;
    double theta_2odom = (msg->se_theta * M_PI) / 180.0f;
    double rpm_2odom = msg->se_vel;
    // 将接收到的角度映射到(-pi,pi)之间
    while (theta_2odom > M_PI)
        theta_2odom -= 2 * M_PI;
    while (theta_2odom <= -M_PI)
        theta_2odom += 2 * M_PI;
    // 舵轮rpm转化为速度,求出参数更新前的虚拟差分模型左右轮的角速度
    double v_T = (2 * M_PI * rpm_2odom * radius_steering_wheel) / (60 * k_red_ratio);
    double v = v_T * cos(theta_2odom) + v_T * offset_distance_wheel_y * sin(theta_2odom) / (offset_distance_wheel_x);
    double omega = v_T * sin(theta_2odom) / offset_distance_wheel_x;
    double v_L = v - omega * wheel_track_theory_const / 2.0f;
    double v_R = v + omega * wheel_track_theory_const / 2.0f;
    double omega_L = v_L / radius_virtual_left_wheel_const;
    double omega_R = v_R / radius_virtual_right_wheel_const;

    v_L = omega_L * radius_virtual_left_wheel;
    v_R = omega_R * radius_virtual_right_wheel;
    double v_2odom = (v_L + v_R) / 2.0f;

    double omega_2odom = (v_R - v_L) / wheel_track;

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
        double v_x = v_2odom * cos(sum_theta);
        double v_y = v_2odom * sin(sum_theta);

        // 根据速度计算累计的位置和方向角偏差

        // double delta_theta = omega_2odom * delta_time;
        double delta_theta ;
        if(use_imu)delta_theta =imu_angular_velocity_z_ * delta_time;
        else delta_theta = omega_2odom * delta_time;

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
            if (pub_odom)
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

            if (pub_odom)
                odom_bd.sendTransform(odom_trans);
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
    }
}
