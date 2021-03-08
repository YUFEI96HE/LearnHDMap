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

#ifndef HANDLECCU_H
#define HANDLECCU_H

#include <pthread.h>
#include <ros/package.h>
// #include <boost/function.hpp>
#include "TCPClient.h"
#include "CCUProtocol.h"
#include "Utils.h"
#include "MessageCenter.h"

class HandleCCU
{
private:
	static int socket_ccu;
	static pthread_t thread_heartbeat_ccu;
	/* data */
	static class MessageCenter ccu_center;
	static class Utils ccu_utils;
	/* Funs */
	static void *HeartbeatThread(void *params);
	int SetupCCUConnection(char *server_address, int server_port);
	int DoProtocol(unsigned char *ccu_buffer, int len);

public:
	/* 用这个来表示收到的任务执行命令 */
	static std::string task_mark;

	static double voltage;
	static double current;
	static double wheel_speed;
	static double vehicle_speed;
	static int ultrasound[8];
	static int fault_code;
	static int forward_backward;

	HandleCCU(/* args */);
	~HandleCCU();

	static int SendCCU(unsigned char *send_msg, int len);
	int RecvCCU(unsigned char *ccu_buffer);
	int Task(char *server_address, int server_port);
	//int MainLoop(char *server_address, int server_port);
};

#endif
