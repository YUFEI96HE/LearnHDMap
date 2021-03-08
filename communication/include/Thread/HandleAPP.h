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

#ifndef HANDLEAPP_H
#define HANDLEAPP_H

#include <pthread.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <cstdio>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "APPProtocol.h"
#include "HandleCCU.h"
#include "base.h"
#include "Utils.h"
#include "MessageCenter.h"
#include "adam_msgs/RoboNodeStatus.h"

class HandleAPP
{
private:
    /* data */
    int socket_app;

    static class Utils app_utils;
    static class HandleADT *adt;

    int SetupAPPConnection(int server_port);
    ros::Publisher node_pub;

    int DoProtocol(unsigned char *app_buffer, int len, int socket);
    int ReplyMapTrajTask(unsigned char result, unsigned char *message);
    int ReplySweepTask(unsigned char result, unsigned char last_time_stamp, unsigned char *message);

    static void *PlanTask(void *params);

public:
    HandleAPP(/* args */);
    ~HandleAPP();

    uint8_t sweep_task;
    uint8_t sweep_start_time[8];
    uint8_t sweep_end_time[8];
    uint8_t sweep_map_id[8];
    uint8_t sweep_traj_id[8];
    uint8_t sweep_top_left[8];
    uint8_t sweep_below_right[8];

    adam_msgs::RoboNodeStatus tmp_status;
    bool END_MAP = false;


    int SendAPP(int socket, unsigned char *send_msg, int len);
    int RecvAPP(int socket, unsigned char *recv_buff);

    int Task(int server_port);
};

#endif
