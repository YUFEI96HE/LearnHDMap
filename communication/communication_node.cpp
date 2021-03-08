#include <ros/ros.h>
#include <ros/package.h>
#include <pthread.h>
#include "HandleCCU.h"
#include "HandleADT.h"
#include "HandleAPP.h"
#include "MessageCenter.h"

class HandleCCU ccu;
class HandleADT adt;
class HandleAPP app;
class MessageCenter message_center;

char ccu_ip[256] = "192.168.1.101";
int ccu_port = 10008;
char adt_ip[256] = "admin.techinao.com";
int adt_port = 9999;
//char app_ip[256] = "192.168.43.100";
int app_port = 6665;
std::string node_path;

void systemInit()
{
	std::string node_path = ros::package::getPath("communication");
	std::cout << "NodePath is: " << node_path << std::endl;
}

void *CCUThread(void *args)
{

	ccu.Task(ccu_ip, ccu_port);
	return NULL;
}

void *ADTThread(void *args)
{
	adt.Task(adt_ip, adt_port);
	return NULL;
}

void *APPThread(void *ptr)
{
	app.Task(app_port);
	return NULL;
}

void *MsgThread(void *args)
{
	message_center.MainLoop();
	return NULL;
}

int main(int argc, char *argv[])
{
	// systemInit();
	ros::init(argc, argv, "Communication");
	ros::NodeHandle nh_;
	ros::Rate loop_rate(1);

	std::cout << "Communication Running ..." << std::endl;

	pthread_t ccu_thread, adt_thread, app_thread, msg_thread;

	if (pthread_create(&ccu_thread, NULL, &CCUThread, NULL) < 0)
	{
		std::cerr << "Create CCU Thread Error\n";
	}
	else
	{
		std::cout << "Create CCU Thread Success\n";
		if (pthread_create(&adt_thread, NULL, &ADTThread, NULL) < 0)
		{
			std::cerr << "Create ADT Thread Error\n";
		}
		std::cout << "Create ADT Thread Success\n";
	}
	if (pthread_create(&msg_thread, NULL, &MsgThread, NULL) < 0)
	{
		std::cerr << "Create Message Thread Error\n";
	}
	std::cerr << "Create Message Thread Success\n";

	if (pthread_create(&app_thread, NULL, &APPThread, NULL) < 0)
	{
		std::cerr << "Create APP Thread Error\n";
	}
	std::cerr << "Create APP Thread Success\n";

	while (ros::ok())
	{
		// ros::spinOnce();
		// loop_rate.sleep();
		sleep(1);
	}

	return 0;
}
