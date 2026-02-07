
#include <ros/ros.h>
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "string"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Path.h>

using namespace std;

ros::Publisher trajectory_msg_pub_;
double max_vel = 1.0f;
double anlge_add  = 0.01f;
double max_rotation_R = 4.0f;
double min_rotation_R = 1.0f;
double delta_R = 0.01f;

int shell_call(std::string cmdstr) {
    enum { maxline=100 };
    char line[maxline];
    FILE *fpin;
    int ret;
    if((fpin = popen(cmdstr.c_str(), "r")) == NULL) {
        printf("popen error\n");
        exit(-1);
    }
    for(;;) {
        fputs("prompt> ", stdout);
        fflush(stdout);
        if(fgets(line, maxline, fpin) == NULL) /*read from pipe*/
            break;
        if(fputs(line, stdout) == EOF) {
            printf("fputs error\n");
            exit(-1);
        }
    }
    if((ret = pclose(fpin)) == -1) {
        printf("pclose error\n");
        exit(-1);
    }
    return ret;
}

int send_cmd(void)
{
    geometry_msgs::Twist cmd;
    static int mode = 0;
    static double angle = 0;
    static double R = max_rotation_R;
    static double vel = 0;
    int vel_direction = 1;
    
    switch(mode)
    {
        case 0:  //forward
            if(angle < M_PI)
            {
                angle = angle + anlge_add;
                R = 1000;
                vel_direction = 1;
            }
            else
            {
                mode = mode +1;
            }
            break;
        case 1:  //backward
            if(angle >0)
            {
                angle = angle - anlge_add;
                R = 1000;
                vel_direction = -1;
            }
            else
            {
                R = max_rotation_R;
                mode = mode +1;
            }
            break;
        case 2:  //forward left
            if(angle < M_PI)
            {
                angle = angle + anlge_add;
                R = R - delta_R;
                vel_direction = 1;
            }
            else
            {
                mode = mode +1;
            }
            break;
        case 3: //backward left
            if(angle > 0)
            {
                angle = angle - anlge_add;
                R = R + delta_R;
                vel_direction = -1;
            }
            else
            {
                R = - max_rotation_R;
                mode = mode +1;
            }
            break;
        case 4: //forward right
            if(angle < M_PI)
            {
                angle = angle + anlge_add;
                R = R + delta_R;
                vel_direction = 1;
            }
            else
            {
                mode = mode +1;
            }
            break;
        case 5: //backward right
            if(angle > 0)
            {
                angle = angle - anlge_add;
                R = R - delta_R;
                vel_direction = -1;
            }
            else
            {
                mode = mode +1;
            }
            break;
        default:
            return 1;
        break;

    }
    vel = max_vel*sin(angle)*vel_direction;
    cmd.linear.x  = vel;
    cmd.angular.z = vel/R;

    std::cout<<"vel"<<cmd.linear.x<<" "<<"omiga: "<<cmd.angular.z<<std::endl;
   
    trajectory_msg_pub_.publish(cmd);
    return 0;
}

int main(int argc, char* argv[]){

    ros::init(argc, argv, "tra_gen_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    double arg_time = 0 ;
    nh_priv.getParam("arg_movetime", arg_time );
    arg_time = arg_time/6.0;
    if(arg_time != 0)
    {
        anlge_add = M_PI / (arg_time / 0.1) ;
    }
    double delay_time = 1.5;
    nh_priv.getParam("arg_delay_time", delay_time );
    nh_priv.getParam("arg_max_rotation_R", max_rotation_R );
    nh_priv.getParam("arg_min_rotation_R", min_rotation_R );

    delta_R = (max_rotation_R - min_rotation_R) /  (arg_time / 0.1) ;
    ros::Rate loop_rate(10);
    ros::Duration(0.1).sleep();
    trajectory_msg_pub_ = nh_priv.advertise<geometry_msgs::Twist>("/vehicle_cmd",100);
    if(true)
        while(ros::ok()){
            if( send_cmd() ) 
                break;
            ros::spinOnce();
            loop_rate.sleep();
        }
    ros::Duration(2.0).sleep();
    return 0;
}


