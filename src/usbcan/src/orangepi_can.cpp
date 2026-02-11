#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include "common/can.h"
#include "common/EHandle_set.h"
using namespace std;
double px_recv_time =0;
double ow_recv_time = 0;
double pf_recv_time = 0;
int ow_recv_num = 0;
double rack_recv_time;
int rack_recv_num = 0;
ros::Publisher pallet_pose_pub;
ros::Publisher rack_can_pub;
ros::Publisher pallet_pose_on_fork_pub;

ros::Subscriber can1_rx_sub;
ros::Subscriber pick_detection_enable_sub_;
ros::Subscriber unpick_detection_enable_sub_;
ros::Subscriber pallet_on_fork_enable_sub_;

geometry_msgs::Pose pmsg;
geometry_msgs::Pose rmsg;
geometry_msgs::Pose pfmsg;
ros::Publisher marker_pub;
ros::Publisher storage_rack_marker_pub;
ros::Publisher can1_tx_pub;
ros::ServiceClient client_eHandle_set_err_no;
int check_invalid(geometry_msgs::Pose &pose){
    if(pose.position.x == 10000){
        std::cout << "pos x" << std::endl; 
        return -1;
    }
    if(pose.position.y == 10000){
        std::cout << "pos y" << std::endl; 
        return -1;
    }
    if(pose.position.z == 10000){
        std::cout << "pos z" << std::endl;
        return -1;
    }
    if(pose.orientation.x == 10000){
        std::cout << "ori x" << std::endl;
        return -1;
    }
    if(pose.orientation.y == 10000){
        std::cout << "ori y" << std::endl;
        return -1;
    }
    if(pose.orientation.z == 10000){
        std::cout << "ori z" << std::endl;
        return -1;
    }
    if(pose.orientation.w == 10000){
        std::cout << "ori w" << std::endl;
        return -1;
    }
    return 0;

}
void report_err(int err_num)
{
    common::EHandle_set srv_set;
    srv_set.request.ErrorCode = err_num;
    std::cout << "report_err: " << err_num << std::endl;
    client_eHandle_set_err_no.call(srv_set);
}
int set_invalid(geometry_msgs::Pose &pose){
    pose.position.x = 10000;
    pose.position.y = 10000;
    pose.position.z = 10000;
    pose.orientation.x = 10000;
    pose.orientation.y = 10000;
    pose.orientation.z = 10000;
    pose.orientation.w = 10000;
}
static void handle_orangepi_candata(const common::canConstPtr &msg)
{
    double result;

    switch (msg->id)
    {
        case 0x1c0:
            px_recv_time = ros::Time::now().toSec();
            std::cout << "0x1c0" << std::endl;
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pmsg.position.x = result;
            break;
        case 0x1c1:
            std::cout << "0x1c1" << std::endl;
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pmsg.position.y = result;
            break;
        case 0x1c2:
            std::cout << "0x1c2" << std::endl;
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pmsg.position.z = result;
            break;
        case 0x1c3:
            std::cout << "0x1c3" << std::endl;
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pmsg.orientation.x = result;
            break;
        case 0x1c4:
            std::cout << "0x1c4" << std::endl;
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pmsg.orientation.y = result;
            std::cout << "pmsg.orientation.y " << pmsg.orientation.y  << std::endl;
            // ow_recv_time = ros::Time::now().toSec();
            break;
        case 0x1c5:
            std::cout << "0x1c5" << std::endl;
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pmsg.orientation.z = result;
            // ow_recv_time = ros::Time::now().toSec();
            break;
        case 0x1c6:
            std::cout << "0x1c6" << std::endl;
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pmsg.orientation.w = result;
            std::cout <<"rec 0x1c6" << std::endl;
            ow_recv_time = ros::Time::now().toSec();
            std::cout << "delay: " << ow_recv_time - px_recv_time << std::endl;

            break;
        // 货架
        case 0x1c7:
            px_recv_time = ros::Time::now().toSec();
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            rmsg.position.x = result;
            break;
        case 0x1c8:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            rmsg.position.y = result;
            break;
        case 0x1c9:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            rmsg.position.z = result;
            break;
        case 0x1ca:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            rmsg.orientation.x = result;
            break;
        // case 0x10b:
        //     std::memcpy(&result, msg->datas.data(), sizeof(double));
        //     rmsg.orientation.y = result;
        //     std::cout <<"0x10b" << std::endl;
        //     // 0x10b有别的程序在发
        //     // ow_recv_time = ros::Time::now().toSec();
        //     break;

        case 0x1cc:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            rmsg.orientation.z = result;
            break;
        case 0x1cd:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            rmsg.orientation.w = result;
            std::cout <<"0x1cd" << std::endl;
            rack_recv_time = ros::Time::now().toSec();
            break;
        case 0x1ce:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            rmsg.orientation.y = result;
            std::cout <<"0x1ce" << std::endl;
            // 0x10b有别的程序在发
            // ow_recv_time = ros::Time::now().toSec();
            break;
        case 0x1cf:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            if(result < 0.5){
                report_err(57014); // 深度相机连接丢失
            }
            std::cout <<"0x1cf: " << result << std::endl;
            // 0x10b有别的程序在发
            // ow_recv_time = ros::Time::now().toSec();
            break;
        // 托盘脚
        case 0x1e0:
            px_recv_time = ros::Time::now().toSec();
            std::cout << "0x1e0" << std::endl;
            std::memcpy(&result, msg->datas.data(), sizeof(double));

            pfmsg.position.x = result;
            std::cout << "pfmsg.position.x: " << pfmsg.position.x << std::endl; 
            break;
        case 0x1e1:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pfmsg.position.y = result;
            std::cout << "0x1e1" << std::endl;
            break;
        case 0x1e2:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pfmsg.position.z = result;
            std::cout << "0x1e2" << std::endl;
            break;
        case 0x1e3:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pfmsg.orientation.x = result;
            std::cout << "0x1e3" << std::endl;
            break;
        case 0x1e4:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pfmsg.orientation.y = result;
            std::cout << "0x1e4" << std::endl;
            break;
        case 0x1e5:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pfmsg.orientation.z = result;
            std::cout << "0x1e5" << std::endl;
            break;
        case 0x1e6:
            std::memcpy(&result, msg->datas.data(), sizeof(double));
            pfmsg.orientation.w = result;
            std::cout << "pfmsg.position.x: " << pfmsg.position.x << std::endl;
            std::cout << "pfmsg.orientation.w: " << pfmsg.orientation.w << std::endl;  
            std::cout << "0x1e6" << std::endl;
            pf_recv_time = ros::Time::now().toSec();
            break;
    }

    if (fabs(ow_recv_time - px_recv_time) < 0.2 && check_invalid(pmsg) >= 0)
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pmsg.position.x, pmsg.position.y, pmsg.position.z) );
        tf::Quaternion q(pmsg.orientation.x,pmsg.orientation.y,pmsg.orientation.z,pmsg.orientation.w);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "pallet_sensor_frame", "pallet_link"));
        visualization_msgs::Marker marker;
        marker.header.frame_id = "pallet_sensor_frame";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose.position.x = pmsg.position.x;
        marker.pose.position.y = pmsg.position.y;
        marker.pose.position.z = pmsg.position.z;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = 0.5; // 柄直径
        marker.scale.y = 0.05;  // 箭头直径
        marker.scale.z = 0.05;   // 长度
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);
        std::cout << "pallet_pose_pub" << std::endl;
        pallet_pose_pub.publish(pmsg);
        set_invalid(pmsg);
    }

    if (fabs(rack_recv_time - px_recv_time) < 0.2 && check_invalid(rmsg) >= 0)
    {

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(rmsg.position.x, rmsg.position.y, rmsg.position.z) );
        tf::Quaternion q(rmsg.orientation.x,rmsg.orientation.y,rmsg.orientation.z,rmsg.orientation.w);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "pallet_sensor_frame", "unpick_fork_reach_point"));

        visualization_msgs::Marker marker;
        marker.header.frame_id = "pallet_sensor_frame";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose.position.x = rmsg.position.x;
        marker.pose.position.y = rmsg.position.y;
        marker.pose.position.z = rmsg.position.z;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = 0.5; // 柄直径
        marker.scale.y = 0.05;  // 箭头直径
        marker.scale.z = 0.05;   // 长度
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        storage_rack_marker_pub.publish(marker);


        rack_can_pub.publish(rmsg);
        std::cout << "rack_can_pub" << std::endl;
        set_invalid(rmsg);
    }




    if (fabs(pf_recv_time - px_recv_time) < 0.2 && check_invalid(pfmsg) >= 0)
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        std::cout << "pfmsg.position.x: " << pfmsg.position.x << std::endl;
        std::cout << "pfmsg.orientation.w: " << pfmsg.orientation.w << std::endl;  
        transform.setOrigin(tf::Vector3(pfmsg.position.x, pfmsg.position.y, pfmsg.position.z) );
        tf::Quaternion q(pfmsg.orientation.x,pfmsg.orientation.y,pfmsg.orientation.z,pfmsg.orientation.w);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "pallet_sensor_frame", "pallet_foot"));

        std::cout << "pallet_pose_on_fork_pub" << std::endl;
        pallet_pose_on_fork_pub.publish(pfmsg);
        set_invalid(pfmsg);
    }

}

void pick_detection_enable_cbk(const std_msgs::Bool::ConstPtr &msg){
    common::can can_msg;
    can_msg.id = 0x1d5;
    can_msg.len = 8;
    double datas;
    datas = 8;
    unsigned char data_array[8];
    std::memcpy(data_array,reinterpret_cast<unsigned char*>(&datas),8);
    // printf("Data: %d %d %d %d %d %d %d %d\n", 
    // data_array.data[0], data_array.data[1],
    // data_array.data[2], data_array.data[3],
    // data_array.data[4], data_array.data[5],
    // data_array.data[6], data_array.data[7]);
    for(int i = 0;i< 8 ;i++){
        can_msg.datas.push_back(data_array[i]);
    }
    double result;
    std::memcpy(&result,data_array,8);
    can1_tx_pub.publish(can_msg);
}
void unpick_detection_enable_cbk(const std_msgs::Bool::ConstPtr &msg){
    common::can can_msg;
    can_msg.id = 0x1d6;
    can_msg.len = 8;
    double datas;
    datas = 8;
    unsigned char data_array[8];
    std::memcpy(data_array,reinterpret_cast<unsigned char*>(&datas),8);

    for(int i = 0;i< 8 ;i++){
        can_msg.datas.push_back(data_array[i]);
    }
    can1_tx_pub.publish(can_msg);
}
void pallet_on_fork_enable_cbk(const std_msgs::Bool::ConstPtr &msg){
    common::can can_msg;
    can_msg.id = 0x1d7;
    can_msg.len = 8;
    double datas;
    datas = 8;
    unsigned char data_array[8];
    std::memcpy(data_array,reinterpret_cast<unsigned char*>(&datas),8);

    for(int i = 0;i< 8 ;i++){
        can_msg.datas.push_back(data_array[i]);
    }
    can1_tx_pub.publish(can_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "orangepi_cannode");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    pallet_pose_pub = nh_priv.advertise<geometry_msgs::Pose>("/pallet_pose", 10);
    marker_pub = nh_priv.advertise<visualization_msgs::Marker>("/pallet_pose_marker", 1);
    storage_rack_marker_pub = nh_priv.advertise<visualization_msgs::Marker>("/storage_rack_pose_marker", 1);
    rack_can_pub = nh_priv.advertise<geometry_msgs::Pose>("/storage_rack_pose", 10);
    pallet_pose_on_fork_pub = nh_priv.advertise<geometry_msgs::Pose>("/pallet_pose_on_fork", 10);
    can1_rx_sub = nh_priv.subscribe("/can1_rx", 10, handle_orangepi_candata);
    client_eHandle_set_err_no = nh.serviceClient<common::EHandle_set>("/eHandle_set_err_no");
    can1_tx_pub = nh_priv.advertise<common::can>("/can1_tx", 10);
    pick_detection_enable_sub_ =
        nh_priv.subscribe("/pallet_detection_enable", 10, pick_detection_enable_cbk);
    unpick_detection_enable_sub_ =
        nh_priv.subscribe("/unpick_detection_enable", 10, unpick_detection_enable_cbk);
    pallet_on_fork_enable_sub_ =
        nh_priv.subscribe("/pallet_foot_detection_enable", 10, pallet_on_fork_enable_cbk);
    px_recv_time=ros::Time::now().toSec();
    set_invalid(rmsg);
    set_invalid(pmsg);
    set_invalid(pfmsg);
    std::cout << "orangepi_cannode start" << std::endl;
    ros::spin();
    return 0;
}
