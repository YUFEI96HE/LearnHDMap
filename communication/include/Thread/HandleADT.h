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

#ifndef HANDLEADT_H
#define HANDLEADT_H

#include <pthread.h>
#include <ros/package.h>
#include "TCPClient.h"
#include "ADTProtocol.h"
#include "Utils.h"
#include "HandleCCU.h"
#include "MessageCenter.h"
// #include "Buffer.h"
#include <adam_msgs/VehicleCmd.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <time.h>

class HandleADT
{
private:
    /* data */
    static int socket_adt;
    static pthread_t thread_heartbeat_adt;
    static class MessageCenter adt_center;
    static class Utils adt_utils;
    static class HandleCCU handle_ccu;
    // static class Buffer buffer_pool;
    static struct Pool pool;

    static int SetupADTConnection(char *server_address, int server_port);

    static void *HeartbeatThread(void *params);
    static void *PlanTask(void *params);
    int DoProtocol(unsigned char *adt_buffer, int len);

    static void interrupt_handler(int sig);

public:
    HandleADT(/* args */);
    ~HandleADT();

    static std::string map_name;
    static std::string traj_name;
    static std::string end_pr;
    static std::string node_path;
    static std::string cmd_sub;

    static long long sweep_task;
    static long long sweep_start_time;
    static long long sweep_end_time;
    static long long sweep_map_id;
    static long long sweep_traj_id;
    static adam_msgs::VehicleCmd task_seq;

    static geometry_msgs::Pose sweep_top_left;
    static geometry_msgs::Pose sweep_below_right;

    static int SendADT(unsigned char *send_msg, int len);
    int RecvADT(unsigned char *adt_buffer);
    int Task(char *server_address, int server_port);
};

#endif
