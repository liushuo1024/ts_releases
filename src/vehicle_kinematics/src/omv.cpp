#include "vehicle_kinematics/omv.h"

Omv::Omv(ros::NodeHandle &n, ros::NodeHandle &pn)
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

    if (!omv_calibrate)
        offset_distance_wheel_x = offset_distance_wheel_x - L_passive_base;

    left_wheel_p << -L_passive_base, L_passive_tread / 2.0;
    right_wheel_p << -L_passive_base, -L_passive_tread / 2.0;
    steering_wheel_p << L_passive_steer - L_passive_base, 0;
    std::cout << "left_wheel_pose:" << left_wheel_p << std::endl;
    std::cout << "right_wheel_ppse:" << right_wheel_p << std::endl;
    std::cout << "steering_wheel_p:" << steering_wheel_p << std::endl;

    if (!omv_calibrate)
        MovingTaskFeedback_sub_ = nh_.subscribe("/moving_task_feedback", 1, &Omv::MovingTaskFeedback_callback, this);

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

    servo_encoder_sub_ = nh_.subscribe("/omv_servo_encoder", 1, &Omv::recv_omv_servo_encoder, this, ros::TransportHints().tcpNoDelay(true));

    if (omv_calibrate)
    {
        remote_ctrl_pub = nh_.advertise<common::remote_ctrl_mode>("/remote_ctrl_mode", 2);
        vehicle_cmd_sub_ = nh_.subscribe("/vehicle_cmd", 1, &Omv::recv_vehicle_cmd, this);
    }
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    omv_moving_state = 0;

    if (omv_calibrate)
        omv_moving_state = 1;
    ros::spin();
}

Omv::~Omv()
{
}

void Omv::MovingTaskFeedback_callback(const common::MovingTaskFeedback::ConstPtr &msg)
{
    omv_moving_state = msg->moving_mode;
}

void Omv::recv_vehicle_cmd(const geometry_msgs::Twist::ConstPtr &msg)
{
    double v = msg->linear.x;
    double omega = msg->angular.z;

    double v_r = v + omega * wheel_track / 2.0f;
    double v_l = v - omega * wheel_track / 2.0f;

    double omega_r = v_r / radius_virtual_right_wheel;
    double omega_l = v_l / radius_virtual_left_wheel;

    double v_r_0 = omega_r * radius_virtual_right_wheel_const;
    double v_l_0 = omega_l * radius_virtual_left_wheel_const;

    v = 0.5f * (v_r_0 + v_l_0);
    omega = (v_r_0 - v_l_0) / wheel_track_theory_const;
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

        if (theta_2ac > M_PI / 2)
            theta_2ac = theta_2ac - M_PI;

        v_t = omega * sqrt(pow(r_1 - offset_distance_wheel_y, 2) + pow(offset_distance_wheel_x, 2));
        if (theta_2ac < 0)
            v_t = -v_t;
    }

    double rpm_2ac = (30 * v_t * k_red_ratio) / (M_PI * radius_steering_wheel);
    current_time = ros::Time::now();

    common::omv_servo_cmd osc;
    osc.header.stamp = current_time;
    osc.header.frame_id = "odom";
    osc.sc_main_theta = (theta_2ac * 180) / M_PI;
    osc.sc_vel = rpm_2ac;
    osc.sc_left_theta = 0;
    osc.sc_right_theta = 0;

    common::remote_ctrl_mode remote_mode_cmd;
    remote_mode_cmd.type = 1;
    remote_ctrl_pub.publish(remote_mode_cmd);
    ac_cmd_pub_.publish(osc);
}

double Omv::regular_theta(double theta)
{
    while (theta > M_PI)
        theta -= 2 * M_PI;
    while (theta < -M_PI)
        theta += 2 * M_PI;
    return theta;
}

void Omv::recv_omv_servo_encoder(const common::omv_servo_encoder::ConstPtr &msg)
{
    double v_2odom = 0;

    double omega_2odom = 0;
    double theta_2odom = (msg->se_main_theta * M_PI) / 180.0f;
    while (theta_2odom > M_PI)
        theta_2odom -= 2 * M_PI;
    while (theta_2odom <= -M_PI)
        theta_2odom += 2 * M_PI;

    double rpm_2odom = msg->se_vel;
    double v_T = (2 * M_PI * rpm_2odom * radius_steering_wheel) / (60 * k_red_ratio);
    if (omv_moving_state == 1)
    {
        double v = v_T * cos(theta_2odom) + v_T * offset_distance_wheel_y * sin(theta_2odom) / (offset_distance_wheel_x);
        double omega = v_T * sin(theta_2odom) / offset_distance_wheel_x;
        v_2odom = v;
        omega_2odom = omega;
    }
    else if (omv_moving_state == 2)
    {
        double theta_left_2odom = (msg->se_left_theta * M_PI) / 180.0f;
        while (theta_left_2odom > M_PI)
            theta_left_2odom -= 2 * M_PI;
        while (theta_left_2odom <= -M_PI)
            theta_left_2odom += 2 * M_PI;
        if (fabs(regular_theta(theta_left_2odom + M_PI / 2)) < 0.05)
        {
            v_2odom = v_T;
            omega_2odom = 0;
        }
        else if (fabs(regular_theta(theta_left_2odom - M_PI / 2)) < 0.05)
        {
            v_2odom = v_T;
            omega_2odom = 0;
        }
        else
        {
            std::cout << "theta:" << theta_left_2odom << std::endl;
            double sin_ = sin(theta_left_2odom);
            double cos_ = cos(theta_left_2odom);
            double center_x = left_wheel_p(1) * sin_ / cos_ - fabs(left_wheel_p(0));
            std::cout << "center_x:" << center_x << std::endl;
            if (fabs(center_x - steering_wheel_p(0)) < 0.4)
            {
                v_2odom = 0;
                omega_2odom = 0;
            }
            else
            {
                omega_2odom = v_T / (center_x - steering_wheel_p(0));
                v_2odom = omega_2odom * center_x;
            }
        }
    }
    else if (omv_moving_state == 3)
    {
        v_2odom = 0;
        omega_2odom = v_T / steering_wheel_p(0);
    }

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

        last_time = current_time;
    }
}
