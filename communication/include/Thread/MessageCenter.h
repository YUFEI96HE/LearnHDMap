/*
 * @file: 
 * 
 * @brief: 
 * 
 * @author: heyufei
 * 
 * @data: 2020-11-10
 * 
 */
#ifndef MESSAGE_CENTER
#define MESSAGE_CENTER

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <cstdio>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <pthread.h>
// #include <boost/function.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/UInt8MultiArray.h"
#include <std_msgs/Int64MultiArray.h>
#include "Utils.h"
#include "TCPClient.h"
// #include "TCPServer.h"
// #include "ADTProtocol.h"
// #include "APPProtocol.h"
// #include "CCUProtocol.h"
// #include "Heartbeat.h"
#include "HandleCCU.h"
#include "HandleAPP.h"
#include "HandleADT.h"
#include "base.h"
// #include "communication/GaoPeiNan.h"
#include <adam_msgs/VehicleCmd.h>
#include "adam_msgs/RoboNodeStatus.h"

#define MAXBUFFERLEN 4096

struct ClientParams
{
    TCPClient client;
    unsigned char *message = NULL;
};

struct SeverParams
{
    // TCPServer server;
    unsigned char *message = nullptr;
};

class MessageCenter
{
private:
    /* data */

    class HandleCCU *han_ccu;
    class HandleAPP *handle_app;
    class HandleADT *handle_adt;
    class Utils msg_utils;
    // class Heartbeat *heartbeat;
    ros::Publisher ultrasonic_pub;
    ros::Publisher forward_backward_pub;
    ros::Publisher wheel_speed_pub;
    ros::Publisher task_pub;
    ros::Publisher node_pub;

    /* heyufeidebug */
    uint8_t sweep_task;
    uint8_t sweep_start_time[8];
    uint8_t sweep_end_time[8];
    uint8_t sweep_map_id[8];
    uint8_t sweep_traj_id[8];
    uint8_t sweep_top_left[8];
    uint8_t sweep_below_right[8];
    
    /* 回调函数 */
    void CallbackGetVolocity(const geometry_msgs::TwistStamped::ConstPtr &msg_velocity);
    void CallbackGetIMU(const sensor_msgs::Imu::ConstPtr &msg_imu);
    void CallbackGetOdom(const nav_msgs::Odometry::ConstPtr &msg_odom);

    // void *RecvMsgFromCCU(void *params);
    void PubUltrasound();
    void PubDirection();
    void PubVolocity();
    void PubTask();
    void PubRoboNode();

public:
    MessageCenter(/* args */);
    ~MessageCenter();

    /* Msg From CCU */
    double battery;
    double voltage;
    double current;

    double wheel_speed;
    double vehicle_speed;

    int ultrasound[16];

    int fault_code;

    /* Msg From ACU */
    double imu_data_linear_acceleration_x;
    double imu_data_linear_acceleration_y;
    double imu_data_linear_acceleration_z;
    double imu_data_angular_velocity_x;
    double imu_data_angular_velocity_y;
    double imu_data_angular_velocity_z;

    double quaternion_w;
    double quaternion_x;
    double quaternion_y;
    double quaternion_z;

    double angle;

    double position_x;
    double position_y;
    double position_z;

    int forward_backward;

    /* Msg From unknow */
    int remain_sweep;
    int remain_water;
    int remain_area;

    void MainLoop();
};

#endif
