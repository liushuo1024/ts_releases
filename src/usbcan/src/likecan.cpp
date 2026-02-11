// build command line: gcc -lCanCmd -lpthread -lrt -g -o testLikeCan testLikeCan.c

// cp libCanCmd.so /usr/lib/
// make
// sudo ./testLikeCan

#include <unistd.h>
#include <stdio.h>
#include <ros/ros.h>

#include "usbcan/ICANCmd.h"
#include "common/can.h"
#include <can_msgs/Frame.h>
#include <queue>
#include "geometry_msgs/Pose.h"
using namespace std;
queue<common::can> can0_sendqueue;
queue<common::can> can1_sendqueue;
#define __countof(a) (sizeof(a) / sizeof(a[0]))
common::can can0_send_cache;
common::can can0_move_send_cache;
common::can can0_fork_send_cache;
int arg_bitrate0;
int arg_ch0;
int arg_bitrate1;
int arg_ch1;

ros::Publisher can0_recv_pub_;
ros::Publisher can1_recv_pub_;
ros::Publisher can_tx_debug_;

ros::Publisher can0_frame_pub_;
ros::Publisher can1_frame_pub_;

ros::Subscriber can0_send_sub;
ros::Subscriber can1_send_sub;

ros::Subscriber can0_frame_sub;
ros::Subscriber can1_frame_sub;
double fork_recv_time_;
double agv_recv_time_;

ros::Subscriber agv_move_send_sub;
ros::Subscriber fork_floor_send_sub;
ros::Subscriber mima_agv_move_send_sub;
ros::Subscriber mima_fork_floor_send_sub;
ros::Subscriber YJL_agv_move_send_sub;
ros::Subscriber YJL_fork_floor_send_sub;
ros::Subscriber fork_normal_send_sub;

DWORD dwDeviceHandle;
typedef struct
{
    int Run;
    int ch;
    int bit_rate;
} thread_arg_t;

thread_arg_t rcv_thread_arg0 = {0};
thread_arg_t rcv_thread_arg1 = {0};



void receive_func(void *param, ros::Publisher &msg_pub_) // 接收线程的处理函数
{
    int reclen = 0;
    thread_arg_t *thread_arg = (thread_arg_t *)param;
    int ind = thread_arg->ch;
    int count = 0;
    int errcount = 0;
    CAN_DataFrame rec[30];
    CAN_ErrorInformation err;

    if ((reclen = CAN_ChannelReceive(dwDeviceHandle, ind, rec, __countof(rec), 10)) > 0)
    {
        for (int j = 0; j < reclen; j++)
        {
            common::can msg;
            msg.id = rec[j].uID;
            msg.len = rec[j].nDataLen;
            msg.header.stamp = ros::Time::now();
            // printf("CAN%d Receive: %08X", ind, rec[j].uID);
            for (int i = 0; i < rec[j].nDataLen; i++)
            {
                msg.datas.push_back(rec[j].arryData[i]);
                //      printf(" %02X", rec[j].arryData[i]);
            }
            // printf("\n");
            count += reclen;
            // if ((msg.id >= 0x100) && (msg.id <= 0x104))
            //     handle_orangepi_candata(msg);
            //   printf("CAN%d rcv count=%d\n", ind, count);
            msg_pub_.publish(msg);
        }
    }
    else
    {
        if (CAN_GetErrorInfo(dwDeviceHandle, ind, &err) == CAN_RESULT_OK)
        {
            errcount++;
        }
    }
    //     printf("CAN \n");

    // printf("CAN%d rcv count=%d err count:%d\n", ind, count, errcount);
}

int format_bitrate(int bitrate_in, CAN_InitConfig *config_out)
{
    if (config_out == NULL)
    {
        return -1;
    }

    switch (bitrate_in)
    {
    case 50:
        config_out->dwBtr[0] = 0x09; // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-125K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
        config_out->dwBtr[1] = 0x1c; // BTR1
        break;
    case 100:
        config_out->dwBtr[0] = 0x04; // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-125K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
        config_out->dwBtr[1] = 0x1c; // BTR1
        break;
    case 125:
        config_out->dwBtr[0] = 0x03; // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-125K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
        config_out->dwBtr[1] = 0x1c; // BTR1
        break;
    case 250:
        config_out->dwBtr[0] = 0x01; // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-125K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
        config_out->dwBtr[1] = 0x1c; // BTR1
        break;
    case 500:
        config_out->dwBtr[0] = 0x00; // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-125K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
        config_out->dwBtr[1] = 0x1c; // BTR1
        break;
    case 800:
        config_out->dwBtr[0] = 0x00; // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-125K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
        config_out->dwBtr[1] = 0x16; // BTR1
        break;
    case 1000:
        config_out->dwBtr[0] = 0x00; // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-125K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
        config_out->dwBtr[1] = 0x14; // BTR1
        break;
    default:
        config_out->dwBtr[0] = 0x03; // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-125K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
        config_out->dwBtr[1] = 0x1c; // BTR1
        return -1;
        break;
    }

    return 0;
}

int init_can(void)
{
    // printf("debug1\n");
    CAN_InitConfig config = {0};
    CAN_InitConfig config1 = {0};
    //  printf("debug2\n");
    if ((dwDeviceHandle = CAN_DeviceOpen(ACUSB_132B, 0, 0)) == 0)
    {
        printf("open deivce error\n");
        return -1;
    }
    // printf("debug3\n");
    config.dwAccCode = 0;
    config.dwAccMask = 0xffffffff;
    config.nFilter = 0;  // 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
    config.bMode = 0;    // 工作模式(0表示正常模式,1表示只听模式)
    config.nBtrType = 1; // 位定时参数模式(1表示SJA1000,0表示LPC21XX)
    // printf("debug5\n");
    format_bitrate(rcv_thread_arg0.bit_rate, &config);
    config.dwBtr[2] = 0;
    config.dwBtr[3] = 0;
    // printf("debug4\n");
    if (CAN_ChannelStart(dwDeviceHandle, rcv_thread_arg0.ch, &config) != CAN_RESULT_OK)
    {
        printf("Start CAN %d error\n", rcv_thread_arg0.ch);
        return -1;
    }
    // printf("debug6\n");
    config1.dwAccCode = 0;
    config1.dwAccMask = 0xffffffff;
    config1.nFilter = 0;  // 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
    config1.bMode = 0;    // 工作模式(0表示正常模式,1表示只听模式)
    config1.nBtrType = 1; // 位定时参数模式(1表示SJA1000,0表示LPC21XX)
    // printf("debug7\n");
    format_bitrate(rcv_thread_arg1.bit_rate, &config1);
    config1.dwBtr[2] = 0;
    config1.dwBtr[3] = 0;
    // printf("debug8\n");
    if (CAN_ChannelStart(dwDeviceHandle, rcv_thread_arg1.ch, &config1) != CAN_RESULT_OK)
    {
        printf("Start CAN %d error\n", rcv_thread_arg1.ch);
        return -1;
    }
    printf("init_can\n");

    return 0;
}

int send_func(unsigned int ch, queue<common::can> &send_queue)
{

    CAN_DataFrame *send = new CAN_DataFrame[send_queue.size()];
    int i = 0;
    while (!send_queue.empty())
    {
        auto temp = send_queue.front();
        can_tx_debug_.publish(temp);
        send_queue.pop();

        send[i].uID = temp.id;
        send[i].nSendType = 0;       // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
        send[i].bRemoteFlag = 0;     // 0-数据帧；1-远程帧
        send[i].bExternFlag = 0;     // 0-标准帧；1-扩展帧
        send[i].nDataLen = temp.len; // DLC
        for (int j = 0; j < send[i].nDataLen; j++)
        {
            send[i].arryData[j] = temp.datas[j];
        }
        // memcpy(&(send[i].arryData[0]), temp.datas[0], temp.len);
        i++;
    }

    unsigned long sndCnt = CAN_ChannelSend(dwDeviceHandle, ch, send, i);
    if (sndCnt != i)
    {
        perror("can write error");
        return -1;
    }
    return 0;
}

static void send_can_msg0(const common::canConstPtr &msg)
{
    if (can0_sendqueue.size() > 200)
    {
        std::cout << "can0 queue too long" << std::endl;
        while (!can0_sendqueue.empty())
            can0_sendqueue.pop();
        return;
    }
    can0_sendqueue.push(*msg);
}

static void send_can_msg1(const common::canConstPtr &msg)
{
    if (can1_sendqueue.size() > 200)
    {
        std::cout << "can1 queue too long" << std::endl;
        while (!can1_sendqueue.empty())
            can1_sendqueue.pop();
        return;
    }
    can1_sendqueue.push(*msg);
}

common::can can_mima_move_203_cache;
common::can can_mima_fork_203_cache;
common::can can_mima_203_cache;
common::can can_mima_303_cache;

static void send_mima_move(const common::canConstPtr &msg)
{
    agv_recv_time_ = ros::Time::now().toSec();
    if (agv_recv_time_ - fork_recv_time_ > 0.2)
    {
        // std::cout << "fork 超时" << std::endl;
        can_mima_fork_203_cache.datas[7] = 0;
        can_mima_303_cache.datas[0] = 0;
        can_mima_303_cache.datas[2] = 0;
    }
    // if (can0_sendqueue.size() > 200)
    // {
    //     std::cout << "can1 queue too long" << std::endl;
    //     while (!can0_sendqueue.empty())
    //         can0_sendqueue.pop();
    //     return;
    // }
    if (msg->id == 0x203)
    {
        can_mima_move_203_cache.id = msg->id;
        can_mima_move_203_cache.len = msg->len;
        can_mima_move_203_cache.datas[0] = msg->datas[0];
        can_mima_move_203_cache.datas[1] = msg->datas[1];
        can_mima_move_203_cache.datas[2] = msg->datas[2];
        can_mima_move_203_cache.datas[3] = msg->datas[3];
        can_mima_move_203_cache.datas[4] = msg->datas[4];
        can_mima_move_203_cache.datas[5] = msg->datas[5];
        can_mima_move_203_cache.datas[6] = msg->datas[6];
        // can0_sendqueue.push(can_mima_203_cache);
    }
        can_mima_203_cache.id =can_mima_move_203_cache.id;
        can_mima_203_cache.len =can_mima_move_203_cache.len;
        can_mima_203_cache.datas[0]  =((can_mima_move_203_cache.datas[0])|(can_mima_fork_203_cache.datas[0]));
         can_mima_203_cache.datas[1]  =can_mima_move_203_cache.datas[1];
         can_mima_203_cache.datas[2]  =can_mima_move_203_cache.datas[2];
         can_mima_203_cache.datas[3]  =can_mima_move_203_cache.datas[3];
         can_mima_203_cache.datas[4]  =can_mima_move_203_cache.datas[4];
         can_mima_203_cache.datas[5]  =can_mima_move_203_cache.datas[5];
         can_mima_203_cache.datas[6]  =can_mima_move_203_cache.datas[6];
         can_mima_203_cache.datas[7]  =can_mima_fork_203_cache.datas[7];
         can0_sendqueue.push(can_mima_203_cache);
         can0_sendqueue.push(can_mima_303_cache);
}

static void send_mima_fork(const common::canConstPtr &msg)
{
    fork_recv_time_ = ros::Time::now().toSec();
    std::cout << "send_mima_fork "<< fork_recv_time_ - agv_recv_time_ << std::endl;
    if (fork_recv_time_ - agv_recv_time_ > 0.2)
    {
      can_mima_move_203_cache.datas[0]=0;
      can_mima_move_203_cache.datas[1]=0;
      can_mima_move_203_cache.datas[2]=0;
      can_mima_move_203_cache.datas[3]=0;
      can_mima_move_203_cache.datas[4]=0;
      can_mima_move_203_cache.datas[5]=0;
      can_mima_move_203_cache.datas[6]=0;
    }
    // if (can0_sendqueue.size() > 200)
    // {
    //     std::cout << "can1 queue too long" << std::endl;
    //     while (!can0_sendqueue.empty())
    //         can0_sendqueue.pop();
    //     return;
    // }
    if (msg->id == 0x203)
    {
        can_mima_fork_203_cache.datas[0] = msg->datas[0];
        can_mima_fork_203_cache.datas[7] = msg->datas[7];
        // can0_sendqueue.push(can_mima_203_cache);
    }
    if (msg->id == 0x303)
    {
        can_mima_303_cache.id=msg->id;
        can_mima_303_cache.datas[0] = msg->datas[0];
        can_mima_303_cache.datas[2] = msg->datas[2];
        // can0_sendqueue.push(can_mima_303_cache);
    }
}

static void send_fork_floor(const common::canConstPtr &msg)
{
    fork_recv_time_ = ros::Time::now().toSec();
    if (fork_recv_time_ - agv_recv_time_ > 0.2)
    {
        can0_send_cache.datas[0] = 0;
        can0_send_cache.datas[1] = 0;
        can0_send_cache.datas[2] = 0;
        can0_send_cache.datas[3] = 0;
    }
    can0_send_cache.datas[4] = msg->datas[4];
    can0_send_cache.datas[5] = msg->datas[5];
    can0_send_cache.datas[6] = msg->datas[6];
    can0_send_cache.datas[7]++;

    if (can0_sendqueue.size() > 200)
    {
        std::cout << "can1 queue too long" << std::endl;
        while (!can0_sendqueue.empty())
            can0_sendqueue.pop();
        return;
    }
    can0_sendqueue.push(can0_send_cache);
}
static void send_agv_move(const common::canConstPtr &msg)
{
    agv_recv_time_ = ros::Time::now().toSec();
    if (agv_recv_time_ - fork_recv_time_ > 0.2)
    {
        can0_send_cache.datas[4] = 0;
        can0_send_cache.datas[5] = 0;
        can0_send_cache.datas[6] = 0;
    }
    can0_send_cache.datas[0] = msg->datas[0];
    can0_send_cache.datas[1] = msg->datas[1];
    can0_send_cache.datas[2] = msg->datas[2];
    can0_send_cache.datas[3] = msg->datas[3];
    can0_send_cache.datas[7]++;

    if (can0_sendqueue.size() > 200)
    {
        std::cout << "can1 queue too long" << std::endl;
        while (!can0_sendqueue.empty())
            can0_sendqueue.pop();
        return;
    }
    can0_sendqueue.push(can0_send_cache);
}

static void send_fork_normal_cmd(const common::canConstPtr &msg)
{
    fork_recv_time_ = ros::Time::now().toSec();
    if (fork_recv_time_ - agv_recv_time_ > 0.2)
    {
    if (msg->id == 0x3f0)
    {
        can0_send_cache.datas[0] = msg->datas[0];
    }
        can0_send_cache.datas[1] = 0;
        can0_send_cache.datas[2] = 0;
        can0_send_cache.datas[3] = 0;
    }

    if (msg->id == 0x3f0)
    {
        can0_send_cache.datas[4] = msg->datas[4];
        can0_send_cache.datas[5] = msg->datas[5];
        can0_send_cache.datas[6] = msg->datas[6];
        can0_send_cache.datas[7]++;
        if (can0_sendqueue.size() > 200)
        {
            std::cout << "can1 queue too long" << std::endl;
            while (!can0_sendqueue.empty())
                can0_sendqueue.pop();
            return;
        }
        can0_sendqueue.push(can0_send_cache);
    }
    else if (msg->id == 0x3f3)
        can0_sendqueue.push(*msg);
}

static void send_YJL_move(const common::canConstPtr &msg)
{

    std::cout << "yjl_move" << std::endl;
    can0_move_send_cache.id = msg->id;
    can0_move_send_cache.len = msg->len;
    agv_recv_time_ = ros::Time::now().toSec();
    if (agv_recv_time_ - fork_recv_time_ > 0.2)
    {
        can0_fork_send_cache.datas[0] = 0;
        can0_fork_send_cache.datas[5] = 0;
        can0_fork_send_cache.datas[6] = 0;
    }
    can0_move_send_cache.datas[0] = msg->datas[0];
    can0_move_send_cache.datas[1] = msg->datas[1];
    can0_move_send_cache.datas[2] = msg->datas[2];
    can0_move_send_cache.datas[3] = msg->datas[3];
    can0_move_send_cache.datas[4] = msg->datas[4];
    can0_move_send_cache.datas[7] = msg->datas[7];

    if (can0_sendqueue.size() > 200)
    {
        std::cout << "can1 queue too long" << std::endl;
        while (!can0_sendqueue.empty())
            can0_sendqueue.pop();
        return;
    }
    can0_send_cache.id = msg->id;
    can0_send_cache.len = msg->len;
    can0_send_cache.datas[0] = ((can0_move_send_cache.datas[0]) | (can0_fork_send_cache.datas[0]));
    std::cout << " can0_send_cache.datas[0]" << can0_send_cache.datas[0] << std::endl;
    can0_send_cache.datas[1] = can0_move_send_cache.datas[1];
    can0_send_cache.datas[2] = can0_move_send_cache.datas[2];
    can0_send_cache.datas[3] = can0_move_send_cache.datas[3];
    can0_send_cache.datas[4] = can0_move_send_cache.datas[4];
    can0_send_cache.datas[5] = can0_fork_send_cache.datas[5];
    can0_send_cache.datas[6] = can0_fork_send_cache.datas[6];
    can0_send_cache.datas[7] = can0_move_send_cache.datas[7];
    can0_sendqueue.push(can0_send_cache);
}

static void send_YJL_fork(const common::canConstPtr &msg)
{
    std::cout << "yjl_fork" << std::endl;
    can0_fork_send_cache.id = msg->id;
    can0_fork_send_cache.len = msg->len;
    fork_recv_time_ = ros::Time::now().toSec();
    if (fork_recv_time_ - agv_recv_time_ > 0.2)
    {
        can0_move_send_cache.datas[1] = 0;
        can0_move_send_cache.datas[2] = 0;
        can0_move_send_cache.datas[3] = 0;
        can0_move_send_cache.datas[4] = 0;
        can0_move_send_cache.datas[7] = 0;
    }
    can0_fork_send_cache.datas[0] = msg->datas[0];
    can0_fork_send_cache.datas[5] = msg->datas[5];
    can0_fork_send_cache.datas[6] = msg->datas[6];
    can0_send_cache.id = msg->id;
    can0_send_cache.len = msg->len;
    can0_send_cache.datas[0] = ((can0_move_send_cache.datas[0]) | (can0_fork_send_cache.datas[0]));
    std::cout << " can0_send_cache.datas[0]" << can0_send_cache.datas[0] << std::endl;
    can0_send_cache.datas[1] = can0_move_send_cache.datas[1];
    can0_send_cache.datas[2] = can0_move_send_cache.datas[2];
    can0_send_cache.datas[3] = can0_move_send_cache.datas[3];
    can0_send_cache.datas[4] = can0_move_send_cache.datas[4];
    can0_send_cache.datas[5] = can0_fork_send_cache.datas[5];
    can0_send_cache.datas[6] = can0_fork_send_cache.datas[6];
    can0_send_cache.datas[7] = can0_move_send_cache.datas[7];
    can0_sendqueue.push(can0_send_cache);
    // can0_send_cache.datas[7] = msg->datas[7];
    if (can0_sendqueue.size() > 200)
    {
        std::cout << "can1 queue too long" << std::endl;
        while (!can0_sendqueue.empty())
            can0_sendqueue.pop();
        return;
    }
    // can0_sendqueue.push(can0_send_cache);
}

// 将 common::can 转换为 can_msgs::Frame
can_msgs::Frame convertToFrame(const common::can &msg)
{
    can_msgs::Frame frame;
    frame.header = msg.header;
    frame.id = msg.id;
    frame.dlc = msg.len;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;
    for (size_t i = 0; i < 8; ++i)
    {
        if (i < msg.datas.size())
            frame.data[i] = msg.datas[i];
        else
            frame.data[i] = 0;
    }
    return frame;
}

// 将 can_msgs::Frame 转换为 common::can
common::can convertToCan(const can_msgs::Frame &frame)
{
    common::can msg;
    msg.header = frame.header;
    msg.id = frame.id;
    msg.len = frame.dlc;
    msg.type = 0;
    msg.datas.clear();
    for (int i = 0; i < frame.dlc && i < 8; ++i)
    {
        msg.datas.push_back(frame.data[i]);
    }
    return msg;
}

static void send_frame_msg0(const can_msgs::FrameConstPtr &msg)
{
    common::can can_msg = convertToCan(*msg);
    if (can0_sendqueue.size() > 200)
    {
        std::cout << "can0 queue too long" << std::endl;
        while (!can0_sendqueue.empty())
            can0_sendqueue.pop();
        return;
    }
    can0_sendqueue.push(can_msg);
}

static void send_frame_msg1(const can_msgs::FrameConstPtr &msg)
{
    common::can can_msg = convertToCan(*msg);
    if (can1_sendqueue.size() > 200)
    {
        std::cout << "can1 queue too long" << std::endl;
        while (!can1_sendqueue.empty())
            can1_sendqueue.pop();
        return;
    }
    can1_sendqueue.push(can_msg);
}

void receive_frame_func(void *param, ros::Publisher &msg_pub_, ros::Publisher &frame_pub_) // 接收线程的处理函数
{
    int reclen = 0;
    thread_arg_t *thread_arg = (thread_arg_t *)param;
    int ind = thread_arg->ch;
    int count = 0;
    int errcount = 0;
    CAN_DataFrame rec[30];
    CAN_ErrorInformation err;

    if ((reclen = CAN_ChannelReceive(dwDeviceHandle, ind, rec, __countof(rec), 10)) > 0)
    {
        for (int j = 0; j < reclen; j++)
        {
            common::can msg;
            msg.id = rec[j].uID;
            msg.len = rec[j].nDataLen;
            msg.header.stamp = ros::Time::now();
            for (int i = 0; i < rec[j].nDataLen; i++)
            {
                msg.datas.push_back(rec[j].arryData[i]);
            }
            count += reclen;
            msg_pub_.publish(msg);
            // 同时发布 can_msgs::Frame 格式
            frame_pub_.publish(convertToFrame(msg));
        }
    }
    else
    {
        if (CAN_GetErrorInfo(dwDeviceHandle, ind, &err) == CAN_RESULT_OK)
        {
            errcount++;
        }
    }
}

int recv_can(void)
{
    receive_frame_func(&rcv_thread_arg0, can0_recv_pub_, can0_frame_pub_); // 接收线程的处理函数
    receive_frame_func(&rcv_thread_arg1, can1_recv_pub_, can1_frame_pub_); // 接收线程的处理函数
}

int send_can(void)
{
    int res;
    res = send_func(rcv_thread_arg0.ch, can0_sendqueue);
    if (res != 0)
        return res;
    res = send_func(rcv_thread_arg1.ch, can1_sendqueue);
    if (res != 0)
        return res;
    return 0;
}
int main(int argc, char *argv[])
{
    int ret;
    pthread_t rcv_threadid1;
    ros::init(argc, argv, "usbcan_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    int loop_hz;

    nh_priv.param<int>("arg_bitrate0", rcv_thread_arg0.bit_rate, 250);
    nh_priv.param<int>("arg_ch0", rcv_thread_arg0.ch, 0);
    nh_priv.param<int>("arg_bitrate1", rcv_thread_arg1.bit_rate, 250);
    nh_priv.param<int>("arg_ch1", rcv_thread_arg1.ch, 1);
    nh_priv.param<int>("loop_hz", loop_hz, 4000);
    can0_send_cache.id = 0x3f0;
    can0_send_cache.len = 8;
    for (int i = 0; i < can0_send_cache.len; i++)
    {
        can0_send_cache.datas.push_back(0);
        can0_move_send_cache.datas.push_back(0);
        can0_fork_send_cache.datas.push_back(0);
    }
    fork_recv_time_ = ros::Time::now().toSec();
    agv_recv_time_ = ros::Time::now().toSec();

    ros::Rate loop_rate(loop_hz);
    init_can();
    can0_recv_pub_ = nh_priv.advertise<common::can>("/can0_rx", 100);
    can1_recv_pub_ = nh_priv.advertise<common::can>("/can1_rx", 100);
    can_tx_debug_ = nh_priv.advertise<common::can>("/can_tx_debug", 100);
    
    // can_msgs::Frame 格式发布和订阅（ROS标准CAN消息格式）
    can0_frame_pub_ = nh_priv.advertise<can_msgs::Frame>("received_messages", 100);
    can1_frame_pub_ = nh_priv.advertise<can_msgs::Frame>("/can1_frame_rx", 100);
    
    // 创建一个Subscriber,订阅上层数据usb_can_send写入can驱动
    can0_send_sub = nh_priv.subscribe("/can0_tx", 1000, send_can_msg0);
    can1_send_sub = nh_priv.subscribe("/can1_tx", 1000, send_can_msg1);
    
    // can_msgs::Frame 格式订阅（ROS标准CAN消息格式）
    can0_frame_sub = nh_priv.subscribe("sent_messages", 1000, send_frame_msg0);
    can1_frame_sub = nh_priv.subscribe("/can1_frame_tx", 1000, send_frame_msg1);

    agv_move_send_sub = nh_priv.subscribe("/agv_move", 100, send_agv_move);
    fork_floor_send_sub = nh_priv.subscribe("/fork_floor", 100, send_fork_floor);

    fork_normal_send_sub = nh_priv.subscribe("/fork_normal_cmd", 100, send_fork_normal_cmd);

    can_mima_move_203_cache.datas.resize(8);
    can_mima_fork_203_cache.datas.resize(8);
    can_mima_203_cache.datas.resize(8);
    can_mima_303_cache.datas.resize(8);
    can_mima_303_cache.id=0x303;
    can_mima_303_cache.len=8;
    can_mima_203_cache.id=0x203;
    
    // mima_agv_move_send_sub = nh_priv.subscribe("/mima_move", 100, send_mima_move);
    mima_fork_floor_send_sub = nh_priv.subscribe("/mima_fork", 100, send_mima_fork);
    YJL_agv_move_send_sub = nh_priv.subscribe("/YJL_move", 100, send_YJL_move);
    YJL_fork_floor_send_sub = nh_priv.subscribe("/YJL_fork", 100, send_YJL_fork);

    // ros::AsyncSpinner spinner(2);
    // spinner.start();
    while (ros::ok())
    {
        recv_can();
        send_can();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
