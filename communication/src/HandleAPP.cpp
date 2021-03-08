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

#include "HandleAPP.h"

class Utils HandleAPP::app_utils;
class HandleADT *HandleAPP::adt;

HandleAPP::HandleAPP(/* args */) {}

HandleAPP::~HandleAPP() {}

void *HandleAPP::PlanTask(void *params)
{
    int rem = (int &)params;
    std::cout << "Planning APP Task, Remaining Time: " << rem << std::endl;
    //sleep(rem);

    std::string map_name = app_utils.LongToString(adt->sweep_map_id);
    std::string traj_name = app_utils.LongToString(adt->sweep_traj_id);
    std::string node_path = ros::package::getPath("communication");
    std::string cmd_sub = "bash " + node_path + "/../../../scripts/sweeper_run.sh sweep ";
    std::string end_by = " &";
    // std::string cmd_sub = "bash /home/nvidia/sweeper/catkin_ws/src/AutoSweeper/scripts/sweeper_run.sh sweep ";
    if (adt->sweep_traj_id == 0) /* 执行没有轨迹 （按照发布的ros点选择扫地区域） */
    {
        std::string cmd = cmd_sub + map_name + " " + traj_name + end_by;
        int ret = app_utils.RunShellCmd(cmd);
        sleep(3);
    }
    else /* 执行轨迹 */
    {
        std::string cmd = cmd_sub + map_name + " " + traj_name + end_by;
        int ret = app_utils.RunShellCmd(cmd);
        sleep(3);
    }
}

int HandleAPP::SendAPP(int socket, unsigned char *send_msg, int len)
{
    std::vector<unsigned char> send_app_msg;
    send_app_msg.clear();
    app_utils.AddEscapeCharacter(send_msg, send_app_msg, len);
    unsigned char send_app_buffer[send_app_msg.size()];
    for (int i = 0; i < send_app_msg.size(); ++i)
    {
        send_app_buffer[i] = send_app_msg[i];
    }

    int ret = send(socket, send_app_buffer, len, 0);
    if (ret > 0)
    {
        return ret;
    }
    return -1;
}

int HandleAPP::RecvAPP(int socket, unsigned char *recv_buff)
{
    int ret = recv(socket, recv_buff, MAXBUFFERLEN, 0); //test for client connect

    int recv_app_escape_char_num = app_utils.DelEscapeCharcter(recv_buff, ret);
    ret -= recv_app_escape_char_num;

    if (ret > 0)
    {
        return ret;
    }
    if (ret == 0)
    {
        return 0;
    }
    return -1;
}

int HandleAPP::SetupAPPConnection(int server_port)
{
    ros::NodeHandle app_nh_;
    node_pub = app_nh_.advertise<adam_msgs::RoboNodeStatus>("app_cmd", 1000);

    int listenfd, connfd;
    struct sockaddr_in servaddr;
    unsigned char buff[MAXBUFFERLEN];
    int n_recv = 0;

    if ((listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        //创建监听套接字
        printf("Create APP Listen Socket Fail!\n");
        return -1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    // servaddr.sin_addr.s_addr = inet_addr("192.168.31.52");
    servaddr.sin_port = htons(server_port);

    if (bind(listenfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1)
    {
        //绑定服务
        printf("Bind APP Socket Fail!\n");
        return -1;
    }

    if (listen(listenfd, 10) == -1)
    {
        //主服务监听连接
        printf("Listen APP Socket Fail!\n");
        return -1;
    }

    unsigned int listendAddrLen = sizeof(servaddr);
    //获取监听的地址和端口
    if (getsockname(listenfd, (struct sockaddr *)&servaddr, &listendAddrLen) == -1)
    {
        printf("Getsockname Error\n");
        return -1;
    }
    printf("Listen APP Client Address = %s:%d\n", inet_ntoa(servaddr.sin_addr), ntohs(servaddr.sin_port));

    printf("====== Acu Waiting For APP Client Request ======\n");

    for (;;)
    {
        socket_app = accept(listenfd, (struct sockaddr *)NULL, NULL);
        printf("====== APP Connect ACU Successfully======\n");
        struct timeval timeout = {
            1,
        };
        int ret = setsockopt(socket_app, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));

        for (;;)
        {
            /* 这里循环客户端 */
            unsigned char recv_buff[MAXBUFFERLEN];
            int count = RecvAPP(socket_app, recv_buff);
            if (count > 0)
            {
                DoProtocol(recv_buff, count, socket_app);
            }
            else if ((count < 0) && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) //这几种错误码，认为连接是正常的，继续接收
            {
                /* 发送心跳包 */
                struct VehicleStatus vehicle_status;
                memset(&vehicle_status, 0, sizeof(struct VehicleStatus));

                /* 开始封包 */
                vehicle_status.start = 0xFA;
                vehicle_status.type = 0x12;
                vehicle_status.no = 0x00;

                /* 车辆ID */
                unsigned char vehicle_id_buffer[8];
                memset(vehicle_id_buffer, 0, sizeof(vehicle_id_buffer));
                app_utils.DecToHex(app_utils.GetVehicleID(), vehicle_id_buffer, 8);
                memcpy(vehicle_status.id, vehicle_id_buffer, sizeof(vehicle_status.id));

                vehicle_status.statu_mark = 0x00;
                for (int g = 0; g < 12; ++g)
                {
                    vehicle_status.gps[g] = 0x00;
                    vehicle_status.position[g] = 0x00;
                }
                for (int t = 0; t < 2; ++t)
                {
                    vehicle_status.battery[t] = 0x00;
                    vehicle_status.voltage[t] = 0x00;
                    vehicle_status.current[t] = 0x00;
                    vehicle_status.wheel_speed[t] = 0x00;
                    vehicle_status.angle[t] = 0x00;
                    vehicle_status.error_code[t] = 0x00;
                }
                for (int m = 0; m < 8; ++m)
                {
                    vehicle_status.time_stamp[m] = 0x00;
                }
                for (int n = 0; n < 16; ++n)
                {
                    vehicle_status.ultrasound[n] = 0x00;
                }

                vehicle_status.val_code = 0x00;
                vehicle_status.end = 0XFB;

                unsigned char heart_buffer[MAXBUFFERLEN];
                memcpy(heart_buffer, &vehicle_status, sizeof(struct VehicleStatus));
                int ret = SendAPP(socket_app, heart_buffer, sizeof(struct VehicleStatus));
                continue; //继续接收数据
            }
            else
            {
                printf("Break APP Receive Lopp\n");
                break; //跳出接收循环
            }
        }
    }
    close(listenfd);
    return 1;
}

int HandleAPP::Task(int server_port)
{
    SetupAPPConnection(server_port);
    return 0;
}

int HandleAPP::ReplyMapTrajTask(unsigned char result, unsigned char *message)
{
    struct MapTrajTaskReply tasks_reply;
    memset(&tasks_reply, 0, sizeof(struct MapTrajTaskReply));
    tasks_reply.start = 0xFA;
    tasks_reply.type = 0x01;
    tasks_reply.no = 0x00;

    unsigned char vehicle_id_buffer[8];
    memset(vehicle_id_buffer, 0, sizeof(vehicle_id_buffer));
    app_utils.DecToHex(app_utils.GetVehicleID(), vehicle_id_buffer, 8);
    memcpy(tasks_reply.id, vehicle_id_buffer, sizeof(tasks_reply.id));

    tasks_reply.statu_mark = 0x00;
    tasks_reply.result = result;

    unsigned char time_stamp_buffer[8];
    long long time_stamp = app_utils.GetTimeStamp();
    app_utils.DecToHex(time_stamp, time_stamp_buffer, 8);
    memcpy(&tasks_reply.time_stamp, time_stamp_buffer, sizeof(tasks_reply.time_stamp));

    tasks_reply.val_code = 0x00;
    tasks_reply.reason = 0x00;
    tasks_reply.end = 0xFB;

    memset(message, 0, sizeof(tasks_reply));

    memcpy(message, &tasks_reply, sizeof(struct MapTrajTaskReply));

    std::cout << "DEBUG  CCU  BUILD  REPLY\n";

    return 0;
}

int HandleAPP::ReplySweepTask(unsigned char result, unsigned char last_time_stamp, unsigned char *message)
{
    struct SweepTaskReply sweep_reply;
    memset(&sweep_reply, 0, sizeof(struct MapTrajTaskReply));
    sweep_reply.start = 0xFA;
    sweep_reply.type = 0x04;
    sweep_reply.no = 0x00;

    unsigned char vehicle_id_buffer[8];
    memset(vehicle_id_buffer, 0, sizeof(vehicle_id_buffer));
    app_utils.DecToHex(app_utils.GetVehicleID(), vehicle_id_buffer, 8);
    memcpy(sweep_reply.id, vehicle_id_buffer, sizeof(sweep_reply.id));

    sweep_reply.statu_mark = 0x00;
    sweep_reply.result = result;
    sweep_reply.task_time = last_time_stamp;

    unsigned char time_stamp_buffer[8];
    long long time_stamp = app_utils.GetTimeStamp();
    app_utils.DecToHex(time_stamp, time_stamp_buffer, 8);
    memcpy(&sweep_reply.time_stamp, time_stamp_buffer, sizeof(sweep_reply.time_stamp));

    sweep_reply.val_code = 0x00;
    sweep_reply.reason = 0x00;
    sweep_reply.end = 0xFB;

    // unsigned char send_buffer[MAXBUFFERLEN];
    memset(message, 0, sizeof(sweep_reply));
    memcpy(message, &sweep_reply, sizeof(struct SweepTaskReply));
    return 0;
}

int HandleAPP::DoProtocol(unsigned char *app_buffer, int len, int socket)
{
    unsigned char origin_buffer[MAXBUFFERLEN];
    memset(origin_buffer, 0, sizeof(origin_buffer));
    memcpy(origin_buffer, app_buffer, len);

    if (origin_buffer[0] != 0xFA)
    {
        return -1;
    }

    // for (int i = 0; i < len; ++i)
    // {
    //     printf("%hhu ", origin_buffer[i]);
    // }
    if (len > 0)
    {
        std::cout << "(金亚看这里)APP MSG： " << std::endl;
        for (int e = 0; e < len; e++)
        {
            std::cout << std::hex << static_cast<unsigned short>(origin_buffer[e]) << " ";
        }
        printf("\n\n");
        /* APP对车辆状态信息的回复 */
        if (origin_buffer[1] == 0x12)
        {
            std::cout << "收到APP车辆状态信息回复\n";
        }
        // 参数设置命令
        else if (origin_buffer[1] == 0x51)
        {
            // 回复
            struct ParamConfigReply param_config_reply;
            memset(&param_config_reply, 0, sizeof(struct ParamConfigReply));
            param_config_reply.start = 0xFA;
            param_config_reply.type = 0x51;
            param_config_reply.no = 0x00;

            unsigned char vehicle_id_buffer[8];
            memset(vehicle_id_buffer, 0, sizeof(vehicle_id_buffer));
            app_utils.DecToHex(app_utils.GetVehicleID(), vehicle_id_buffer, 8);
            memcpy(param_config_reply.car_id, vehicle_id_buffer, sizeof(param_config_reply.car_id));

            param_config_reply.statu_mark = 0x00;
            param_config_reply.result = 0x01;

            unsigned char time_stamp_buffer[8];
            long long time_stamp = app_utils.GetTimeStamp();
            app_utils.DecToHex(time_stamp, time_stamp_buffer, 8);
            memcpy(&param_config_reply.time_stamp, time_stamp_buffer, sizeof(param_config_reply.time_stamp));

            param_config_reply.valcode = 0x00;
            param_config_reply.reason = 0x00;
            param_config_reply.end = 0xFB;

            unsigned char param_config_message[MAXBUFFERLEN];

            memset(param_config_message, 0, sizeof(param_config_reply));
            memcpy(param_config_message, &param_config_reply, sizeof(struct ParamConfigReply));
            SendAPP(socket, param_config_message, sizeof(param_config_message));

            // 处理
            
        }
        // 关机及急停命令
        else if (origin_buffer[1] == 0x50)
        {
            // 回复
            struct OFFMsgReply off_msg_reply;
            memset(&off_msg_reply, 0, sizeof(struct OFFMsgReply));
            off_msg_reply.start = 0xFA;
            off_msg_reply.type = 0x50;
            off_msg_reply.no = 0x00;

            unsigned char vehicle_id_buffer[8];
            memset(vehicle_id_buffer, 0, sizeof(vehicle_id_buffer));
            app_utils.DecToHex(app_utils.GetVehicleID(), vehicle_id_buffer, 8);
            memcpy(off_msg_reply.car_id, vehicle_id_buffer, sizeof(off_msg_reply.car_id));

            off_msg_reply.statu_mark = 0x00;
            off_msg_reply.result = 0x01;

            unsigned char time_stamp_buffer[8];
            long long time_stamp = app_utils.GetTimeStamp();
            app_utils.DecToHex(time_stamp, time_stamp_buffer, 8);
            memcpy(&off_msg_reply.time_stamp, time_stamp_buffer, sizeof(off_msg_reply.time_stamp));

            off_msg_reply.valcode = 0x00;
            off_msg_reply.reason = 0x00;
            off_msg_reply.end = 0xFB;

            unsigned char off_message[MAXBUFFERLEN];

            memset(off_message, 0, sizeof(off_msg_reply));
            memcpy(off_message, &off_msg_reply, sizeof(struct OFFMsgReply));
            SendAPP(socket, off_message, sizeof(off_message));

            // 处理
            if (origin_buffer[12] == 0x01)
            {
                // 关机
                // 获取节点路径
                std::string node_path = ros::package::getPath("communication");
                std::string cmd_off = "bash " + node_path + "/launch/switch.sh &";
                std::cout << "关机命令" << cmd_off << std::endl;
                // 关闭所有节点 poweroff
                // std::string cmd_off = "rosnode kill --all && sleep 5 && sudo poweroff";
                const char *charcmd_off = cmd_off.c_str();
                system(charcmd_off);
            }
            else if (origin_buffer[12] == 0x02)
            {
                // 急停
                // 关闭 drive_ethernet节点
                std::string node_path = ros::package::getPath("communication");
                std::string cmd_stop = "bash " + node_path + "/launch/drive_ethernet_killer.sh &";
                std::cout << "急停命令" << cmd_stop << std::endl;
                // 关闭所有节点 poweroff
                // std::string cmd_off = "rosnode kill --all && sleep 5 && sudo poweroff";
                const char *charcmd_stop = cmd_stop.c_str();
                system(charcmd_stop);
            }
            else
            {
                // 错误处理
                std::cout << "Receive Error Msg 0x50" << std::endl;
            }
        }
        /* APP实时构建命令 */
        else if (origin_buffer[1] == 0x01)
        {
            struct MapTrajTask build_task;
            memset(&build_task, 0, sizeof(build_task));
            memcpy(&build_task, origin_buffer, sizeof(struct MapTrajTask));
            /* 获取文件ID */
            unsigned char file_id_app[sizeof(build_task.file_id)];
            memset(file_id_app, 0, sizeof(build_task.file_id));
            memcpy(file_id_app, build_task.file_id, sizeof(build_task.file_id));
            long long file_id_f = app_utils.HexToDec(file_id_app, sizeof(build_task.file_id));

            std::string file_name = app_utils.LongToString(file_id_f);
            std::string end_cmd = " &";
            /* 获取数据 */
            std::cout << "收到实时构建命令\n";
            /* 构建实时地图 */
            if (origin_buffer[12] == 0x01)
            {
                /* 开始 */
                if (origin_buffer[13] == 0x01)
                {
                    //收到命令先行回复，再执行脚本
                    unsigned char send_buffer[sizeof(struct MapTrajTaskReply)];
                    memset(send_buffer, 0, sizeof(struct MapTrajTaskReply));
                    unsigned char result = 0x01;
                    ReplyMapTrajTask(result, send_buffer);
                    SendAPP(socket, send_buffer, sizeof(send_buffer));

                    std::string node_path = ros::package::getPath("communication");
                    std::string cmd_sub = "bash " + node_path + "/../../../scripts/sweeper_run.sh sweeper ";
                    std::string cmd = cmd_sub + file_name + end_cmd;
                    std::cout << "Bash Command: " << cmd << std::endl;

                    sleep(3);
                    const char *charcmd = cmd.c_str();
                    system(charcmd);
                    sleep(10);
                }
                /*结束*/
                else if (origin_buffer[13] == 0x02)
                {
                    //收到命令先行回复，再执行脚本
                    unsigned char send_buffer[sizeof(struct MapTrajTaskReply)];
                    memset(send_buffer, 0, sizeof(struct MapTrajTaskReply));
                    unsigned char result = 0x01;
                    ReplyMapTrajTask(result, send_buffer);
                    // std::cout << sizeof(send_buffer) << std::endl;
                    SendAPP(socket, send_buffer, sizeof(send_buffer));

                    /* 结束建图任务 */
                    tmp_status.node_name = "communication_node";
                    tmp_status.cmd_id = adam_msgs::RoboNodeStatus::CMD_END_MAP;
                    node_pub.publish(tmp_status);
                }
                else
                {
                    unsigned char send_buffer[sizeof(struct MapTrajTaskReply)];
                    memset(send_buffer, 0, sizeof(struct MapTrajTaskReply));
                    unsigned char result = 0x02;
                    ReplyMapTrajTask(result, send_buffer);
                    // std::cout << sizeof(send_buffer) << std::endl;
                    SendAPP(socket, send_buffer, sizeof(send_buffer));
                    std::cerr << "ErrorMsg:APP011201\n";
                }
            }
            /* 构建实时路径规划 */
            else if (origin_buffer[12] == 0x02)
            {
                if (origin_buffer[13] == 0x01)
                {
                    /* 直接回复 */
                    unsigned char send_buffer[sizeof(struct MapTrajTaskReply)];
                    memset(send_buffer, 0, sizeof(struct MapTrajTaskReply));
                    unsigned char result = 0x01;
                    ReplyMapTrajTask(result, send_buffer);
                    SendAPP(socket, send_buffer, sizeof(send_buffer));

                    /* 注释：本版本暂不使用成功失败判断逻辑 */
                    // if (ret == 0)
                    // {
                    //     unsigned char send_buffer[sizeof(struct MapTrajTaskReply)];
                    //     memset(send_buffer, 0, sizeof(struct MapTrajTaskReply));
                    //     unsigned char result = 0x01;
                    //     ReplyMapTrajTask(result, send_buffer);
                    //     SendAPP(socket, send_buffer, sizeof(send_buffer));
                    // }
                    // else
                    // {
                    //     unsigned char send_buffer[sizeof(struct MapTrajTaskReply)];
                    //     memset(send_buffer, 0, sizeof(struct MapTrajTaskReply));
                    //     unsigned char result = 0x02;
                    //     ReplyMapTrajTask(result, send_buffer);
                    //     SendAPP(socket, send_buffer, sizeof(send_buffer));
                    // }

                    std::string node_path = ros::package::getPath("communication");
                    std::string cmd = "bash " + node_path + "/../../../scripts/sweeper_run.sh record " + file_name + " " + file_name + " &";
                    std::cout << "Bash Command: " << cmd << std::endl;

                    const char *charcmmd = cmd.c_str();
                    system(charcmmd);
                }
                else if (origin_buffer[13] == 0x02)
                {
                    unsigned char send_buffer[sizeof(struct MapTrajTaskReply)];
                    memset(send_buffer, 0, sizeof(struct MapTrajTaskReply));
                    unsigned char result = 0x01;
                    ReplyMapTrajTask(result, send_buffer);
                    SendAPP(socket, send_buffer, sizeof(send_buffer));

                    std::string cmd = "rosnode kill build_traj";
                    int ret = app_utils.RunShellCmd(cmd);

                    // if (ret == 0)
                    // {
                    //     unsigned char send_buffer[sizeof(struct MapTrajTaskReply)];
                    //     memset(send_buffer, 0, sizeof(struct MapTrajTaskReply));
                    //     unsigned char result = 0x01;
                    //     ReplyMapTrajTask(result, send_buffer);
                    //     SendAPP(socket, send_buffer, sizeof(send_buffer));
                    // }
                    // else
                    // {
                    //     unsigned char send_buffer[sizeof(struct MapTrajTaskReply)];
                    //     memset(send_buffer, 0, sizeof(struct MapTrajTaskReply));
                    //     unsigned char result = 0x02;
                    //     ReplyMapTrajTask(result, send_buffer);
                    //     SendAPP(socket, send_buffer, sizeof(send_buffer));
                    // }
                }
                else
                {
                    unsigned char send_buffer[sizeof(struct MapTrajTaskReply)];
                    memset(send_buffer, 0, sizeof(struct MapTrajTaskReply));
                    unsigned char result = 0x02;
                    ReplyMapTrajTask(result, send_buffer);
                    SendAPP(socket, send_buffer, sizeof(send_buffer));
                    std::cerr << "ErrorMsg:APP011202\n";
                }
            }
            else
            {
                unsigned char send_buffer[sizeof(struct MapTrajTaskReply)];
                memset(send_buffer, 0, sizeof(struct MapTrajTaskReply));
                unsigned char result = 0x02;
                ReplyMapTrajTask(result, send_buffer);
                SendAPP(socket, send_buffer, sizeof(send_buffer));
                std::cerr << "ErrorMsg:APP011203\n";
            }
        }
        /* APP任务计划执行命令 */
        else if (origin_buffer[1] == 0x04)
        {
            std::cout << "收到APP任务计划执行命令\n";
            struct TaskMsg task_msg;
            memset(&task_msg, 0, sizeof(struct TaskMsg));
            memcpy(&task_msg, origin_buffer, sizeof(struct TaskMsg));

            /* 获取这些数据 然后通过ROS在MessageCenter发布到app_task的话题中 */
            adt->task_seq.header.stamp = ros::Time::now();
            adt->sweep_task = (int32_t)task_msg.task_class;
            //std::cout << "peinan kan zhe " << adt->sweep_task << std::endl;
            adt->task_seq.task_id = adt->sweep_task;

            unsigned char sweep_tl_x[4], sweep_tl_y[4], sweep_bl_x[4], sweep_bl_y[4];
            memset(sweep_tl_x, 0, 4);
            memset(sweep_tl_y, 0, 4);
            memset(sweep_bl_x, 0, 4);
            memset(sweep_bl_y, 0, 4);
            for (int m = 0; m < 4; ++m)
            {
                sweep_tl_x[m] = task_msg.area_top_left[m];
                sweep_tl_y[m] = task_msg.area_top_left[4 + m];
                sweep_bl_x[m] = task_msg.area_blow_right[m];
                sweep_bl_y[m] = task_msg.area_blow_right[4 + m];
            }
            
            adt->task_seq.top_left.x = (app_utils.HexToDec(sweep_tl_x, sizeof(sweep_tl_x))) / 1000;
            adt->task_seq.top_left.y = (app_utils.HexToDec(sweep_tl_y, sizeof(sweep_tl_y))) / 1000;
            adt->task_seq.below_right.x = (app_utils.HexToDec(sweep_bl_x, sizeof(sweep_bl_x))) / 1000;
            adt->task_seq.below_right.y = (app_utils.HexToDec(sweep_bl_y, sizeof(sweep_bl_y))) / 1000;

            unsigned char map_id[sizeof(task_msg.map_id)];
            memset(map_id, 0, sizeof(task_msg.map_id));
            memcpy(map_id, task_msg.map_id, sizeof(task_msg.map_id));
            adt->sweep_map_id = app_utils.HexToDec(map_id, sizeof(task_msg.map_id));

            unsigned char traj_id[sizeof(task_msg.planning_id)];
            memset(traj_id, 0, sizeof(task_msg.planning_id));
            memcpy(traj_id, task_msg.planning_id, sizeof(task_msg.planning_id));
            adt->sweep_traj_id = app_utils.HexToDec(traj_id, sizeof(task_msg.planning_id));

            adt->task_seq.map_id = adt->sweep_map_id;
            adt->task_seq.traj_id = adt->sweep_traj_id;

            /* 计划任务定时执行 */
            /* 起止时间 */
            unsigned char start_time_stamp_buffer[8];
            memset(start_time_stamp_buffer, 0, sizeof(start_time_stamp_buffer));
            memcpy(start_time_stamp_buffer, &task_msg.start_time, sizeof(task_msg.start_time));
            long long sweep_start_time = app_utils.HexToDec(start_time_stamp_buffer, 8);
            printf("app start timestamp is %lld\n", sweep_start_time);

            unsigned char end_time_stamp_buffer[8];
            memset(end_time_stamp_buffer, 0, sizeof(end_time_stamp_buffer));
            memcpy(end_time_stamp_buffer, &task_msg.end_time, sizeof(task_msg.end_time));
            long long sweep_end_time = app_utils.HexToDec(end_time_stamp_buffer, 8);
            printf("app end timestamp is %lld\n", sweep_end_time);

            /* 同样也要回复消息给APP 好在封装过了 */
            /* 无操作 */
            unsigned char send_app_buffer[sizeof(struct SweepTaskReply)];
            memset(send_app_buffer, 0, sizeof(struct SweepTaskReply));

            unsigned char last_time_stamp[8];
            for (int i = 0; i < 8; ++i)
            {
                last_time_stamp[i] = origin_buffer[60 + i];
            }
            // std::string end_by = " &";
            // if (origin_buffer[12] == 0x00)
            // {
            /* 回复APP */
            unsigned char result = 0x01;
            ReplySweepTask(result, (unsigned char)*last_time_stamp, send_app_buffer);
            int rest = SendAPP(socket, send_app_buffer, sizeof(send_app_buffer));
            if (rest > 0)
            {
                std::cout << "计划任务回复成功" << std::endl;
            }
            else
            {
                std::cout << "计划任务回复失败" << std::endl;
            }

            // }

            /* 获取当前时间时间戳 */
            long long time_stamp_now = app_utils.GetTimeStamp();
            /* 距离开始的间隔 */
            int start_left_s = app_utils.RemainSeconds(time_stamp_now, sweep_start_time);
            /* 开辟单独线程执行任务 */

            if (start_left_s <= 0)
            {
                start_left_s = 3;
            }

            pthread_t app_plan;
            pthread_create(&app_plan, NULL, PlanTask, (void *)start_left_s);
        }
        /* 获取任务计划列表任务 */
        else if (origin_buffer[1] == 0x06)
        {
            std::cout << "获取任务计划列表任务（预留）\n";
        }
        /* 获取具体任务信息命令 */
        else if (origin_buffer[1] == 0x07)
        {
            std::cout << "获取具体任务信息命令（预留）\n";
        }
        else
        {

            std::cerr << "Cannot Parse Msg From APP\n";
        }
        return len;
    }
    else if (len < 0)
    {
        std::cerr << "Recv Msg From APP Failed\n";
        return len;
    }
    else if (len == 0)
    {
        std::cout << "No Msg From APP\n";
        return len;
    }
    return len;
}
