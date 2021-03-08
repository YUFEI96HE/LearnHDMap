/*
 * @file: 
 * 
 * @brief: 
 * 
 * @author: heyufei
 * 
 * @data: 2020-11-12
 * 
 */

#include "HandleADT.h"

pthread_t HandleADT::thread_heartbeat_adt;
class Utils HandleADT::adt_utils;
class HandleCCU HandleADT::handle_ccu;
adam_msgs::VehicleCmd HandleADT::task_seq;

long long HandleADT::sweep_task;
long long HandleADT::sweep_start_time;
long long HandleADT::sweep_end_time;
long long HandleADT::sweep_map_id;
long long HandleADT::sweep_traj_id;

std::string HandleADT::map_name;
std::string HandleADT::traj_name;
std::string HandleADT::end_pr;
std::string HandleADT::node_path;
std::string HandleADT::cmd_sub;

geometry_msgs::Pose HandleADT::sweep_top_left;
geometry_msgs::Pose HandleADT::sweep_below_right;

int HandleADT::socket_adt;

HandleADT::HandleADT(/* args */) {}
HandleADT::~HandleADT() {}

void *HandleADT::HeartbeatThread(void *params)
{
    struct VehicleStatuMsg vehicle_msg;
    vehicle_msg.start = 0xFA;
    vehicle_msg.type = 0x12;
    vehicle_msg.no = 0x01;

    /* 只发一个车辆ID */
    unsigned char vehicle_id_buffer[8];
    memset(vehicle_id_buffer, 0, sizeof(vehicle_id_buffer));
    adt_utils.DecToHex(adt_utils.LOCALCARID, vehicle_id_buffer, 8);
    memcpy(vehicle_msg.car_id, vehicle_id_buffer, sizeof(vehicle_msg.car_id));

    vehicle_msg.statu_mark = 0x01;

    for (int i = 0; i < 12; ++i)
    {
        vehicle_msg.gps[i] = 0x00;
        /* position TODO */
        vehicle_msg.position[i] = 0x00;
    }
    for (int m = 0; m < 2; ++m)
    {
        vehicle_msg.battery[m] = 0x00;
        vehicle_msg.voltage[m] = 0x00;
        vehicle_msg.current[m] = 0x00;
        vehicle_msg.vehicle_speed[m] = 0x00;
        vehicle_msg.angle[m] = 0x00;
        vehicle_msg.faultcode[m] = 0x00;
        vehicle_msg.wheel_speed[m] = 0x00;
        vehicle_msg.imu_a_x[m] = 0x00;
        vehicle_msg.imu_a_y[m] = 0x00;
        vehicle_msg.imu_a_z[m] = 0x00;
        vehicle_msg.imu_g_x[m] = 0x00;
        vehicle_msg.imu_g_y[m] = 0x00;
        vehicle_msg.imu_g_z[m] = 0x00;
    }
    for (int n = 0; n < 8; ++n)
    {
        vehicle_msg.time_stamp[n] = 0x00;
    }
    for (int w = 0; w < 16; ++w)
    {
        vehicle_msg.ultrasonic[w] = 0x00;
    }
    vehicle_msg.remain_sweep = 0x00;
    vehicle_msg.remain_water = 0x00;
    vehicle_msg.remain_area = 0x00;
    vehicle_msg.valcode = 0x00;
    vehicle_msg.end = 0xFB;

    unsigned char heartbeat_msg[sizeof(struct VehicleStatuMsg)];
    memcpy(heartbeat_msg, &vehicle_msg, sizeof(struct VehicleStatuMsg));

    /* 暂时只发车辆ID 所以数据包不会变 */
    for (;;)
    {
        SendADT(heartbeat_msg, sizeof(heartbeat_msg));
        // client->Send(heartbeat_msg, sizeof(struct VehicleStatuMsg));
        sleep(15);
    }
    return NULL;
}

void *HandleADT::PlanTask(void *params)
{
    int rem = (int &)params;
    std::cout << "Planning ADT Task, Remaining Time: " << rem << std::endl;
    //sleep(rem);

    if (sweep_traj_id == 0) /* 执行没有轨迹 （按照发布的ros点选择扫地区域） */
    {
        std::string cmd = cmd_sub + map_name + " " + traj_name + end_pr;
        //heyufeidebug
        std::cout << "009: " << cmd << std::endl;
        adt_utils.RunShellCmd(cmd);
        sleep(3);
    }
    else /* 执行轨迹 */
    {
        std::string cmd = cmd_sub + map_name + " " + traj_name + end_pr;
        //heyufeidebug
        std::cout << "010: " << cmd << std::endl;
        adt_utils.RunShellCmd(cmd);
        sleep(3);
    }
}

int HandleADT::DoProtocol(unsigned char *adt_buffer, int len)
{
    if (adt_buffer[0] != 0xFA)
    {
        return -1;
    }

    std::cout << "ADT Msg TRUE Size is:  " << len << std::endl;

    for (int i = 0; i < len; ++i)
    {
        printf("%hhu ", adt_buffer[i]);
    }

    if (adt_buffer[1] == 0x12)
    {
        std::cout << "收到ADT关于车辆状态的回复\n";
    }
    /* 摇杆控制回复 */
    else if (adt_buffer[1] == 0x21)
    {
        std::cout << "收到ADT摇杆控制的数据\n";

        /* 不处理数据了 把报文中ADT给CCU的控制数据直接透传到CCU */
        unsigned char control_msg_to_ccu[sizeof(struct ControlRawData)];
        memset(control_msg_to_ccu, 0, sizeof(struct ControlRawData));
        /* WARNNINGGGGGGG 这里有个问题 报文不一致，需要ADT修改 */
        control_msg_to_ccu[0] = 0xFA;
        control_msg_to_ccu[14] = 0xFB;

        for (int m = 0; m < 13; ++m)
        {
            control_msg_to_ccu[m+1] = adt_buffer[m+12];
        }

        std::cout << "ADT Msg TRUE Size is:  " << len << std::endl;

        for (int x = 0; x < 15; ++x)
        {
            printf("%hhu ", control_msg_to_ccu[x]);
        }

        /* 发送到ccu  ccu是不会给回复的 */
        handle_ccu.SendCCU(control_msg_to_ccu, sizeof(control_msg_to_ccu));

        /* 然后要给ADT回复 */
        struct JoystickMsgReply joy_reply;
        memset(&joy_reply, 0, sizeof(struct JoystickMsgReply));
        joy_reply.start = 0xFA;
        joy_reply.type = 0x21;
        joy_reply.no = 0x00;

        unsigned char vehicle_id_buffer[8];
        memset(vehicle_id_buffer, 0, sizeof(vehicle_id_buffer));
        adt_utils.DecToHex(adt_utils.LOCALCARID, vehicle_id_buffer, 8);
        memcpy(joy_reply.car_id, vehicle_id_buffer, sizeof(joy_reply.car_id));

        joy_reply.statu_mark = 0x00;
        joy_reply.result = 0x01;
        /* 时间戳 */
        unsigned char time_stamp_buffer[8];
        long long time_stamp = adt_utils.GetTimeStamp();
        adt_utils.DecToHex(time_stamp, time_stamp_buffer, 8);
        memcpy(&joy_reply.time_stamp, time_stamp_buffer, sizeof(joy_reply.time_stamp));
        joy_reply.valcode = 0x00;
        joy_reply.end = 0xFB;

        unsigned char joy_reply_msg[sizeof(struct JoystickMsgReply)];
        memset(joy_reply_msg, 0, sizeof(struct JoystickMsgReply));
        memcpy(joy_reply_msg, &joy_reply, sizeof(struct JoystickMsgReply));
        /* 发送 */
        this->SendADT(joy_reply_msg, sizeof(joy_reply_msg));
    }
    /* 视频开关回复 */
    else if (adt_buffer[1] == 0x22)
    {
        std::cout << "收到ADT视频开关的数据\n";
        struct VideoSwitchMsg video_msg;
        memset(&video_msg, 0, sizeof(struct VideoSwitchMsgReply));
        memcpy(&video_msg, adt_buffer, sizeof(struct VideoSwitchMsgReply));
        /* 获取节点路径 */
        std::string node_path = ros::package::getPath("communication");
        std::string cmd_dir = "bash " + node_path + "/launch/";
        //std::string cmd_dir = "bash /home/nvidia/sweeper/catkin_ws/src/AutoSweeper/modules/communication/communication/launch/";
        std::string cmd_end = " &";
        std::string local_id = adt_utils.LongToString(adt_utils.LOCALCARID);

        uint8_t ctrl_switch = video_msg.video_switch;
        int splicing = (ctrl_switch >> 7) & 1;
        if (splicing == 1)
        {
            std::string cmd_main = "launch_transfer_video.sh ";
            std::string cmd = cmd_dir + cmd_main + local_id + cmd_end;

            const char *charcmd = cmd.c_str();
            system(charcmd);
            sleep(3);
        }
        else
        {
            // 单摄像头推流
            // 首先关掉所有推流
            std::string cmd_main = "launch_shutdown_video.sh ";
            std::string cmd = cmd_dir + cmd_main + local_id + cmd_end;
            const char *charcmd_close = cmd.c_str();
            system(charcmd_close);
            sleep(3);

            int front = ctrl_switch & 1;
            int left = (ctrl_switch >> 1) & 1;
            int right = (ctrl_switch >> 2) & 1;
            printf("front:%d,left:%d,right:%d.\n", front, left, right);

            std::string camera_no;
            std::string camera_cmd;

            if (front == 1)
            {
                camera_no = "launch_transfer_video_front.sh ";
                camera_cmd = cmd_dir + camera_no + local_id + cmd_end;
                const char *charcmd_front = camera_cmd.c_str();
                system(charcmd_front);
                sleep(3);
            }
            if (left == 1)
            {
                camera_no = "launch_transfer_video_left.sh ";
                camera_cmd = cmd_dir + camera_no + local_id + cmd_end;
                const char *charcmd_left = camera_cmd.c_str();
                system(charcmd_left);
                sleep(3);
            }
            if (right == 1)
            {
                camera_no = "launch_transfer_video_right.sh ";
                camera_cmd = cmd_dir + camera_no + local_id + cmd_end;
                const char *charcmd_right = camera_cmd.c_str();
                system(charcmd_right);
                sleep(3);
            }
        }

        /* 暂时不知道作何处理 直接回复 */
        struct VideoSwitchMsgReply video_reply;
        memset(&video_reply, 0, sizeof(struct VideoSwitchMsgReply));
        video_reply.start = 0xFA;
        video_reply.type = 0x22;
        video_reply.no = 0x00;

        unsigned char vehicle_id_buffer[8];
        memset(vehicle_id_buffer, 0, sizeof(vehicle_id_buffer));
        adt_utils.DecToHex(adt_utils.LOCALCARID, vehicle_id_buffer, 8);
        memcpy(video_reply.car_id, vehicle_id_buffer, sizeof(video_reply.car_id));

        video_reply.statu_mark = 0x00;
        video_reply.result = 0x01;
        /* 时间戳 */
        unsigned char time_stamp_buffer[8];
        long long time_stamp = adt_utils.GetTimeStamp();
        adt_utils.DecToHex(time_stamp, time_stamp_buffer, 8);
        memcpy(&video_reply.time_stamp, time_stamp_buffer, sizeof(video_reply.time_stamp));
        video_reply.valcode = 0x00;
        video_reply.end = 0xFB;

        unsigned char video_reply_msg[sizeof(struct VideoSwitchMsgReply)];
        memset(video_reply_msg, 0, sizeof(struct VideoSwitchMsgReply));
        memcpy(video_reply_msg, &video_reply, sizeof(struct VideoSwitchMsgReply));
        /* 发送 */
        this->SendADT(video_reply_msg, sizeof(video_reply_msg));
    }
    /* 计划任务回复 */
    else if (adt_buffer[1] == 0x04)
    {
        std::cout << "收到ADT计划任务的数据\n";
        struct TaskMsg task_msg;
        memset(&task_msg, 0, sizeof(struct TaskMsg));
        memcpy(&task_msg, adt_buffer, sizeof(struct TaskMsg));

        task_seq.header.stamp = ros::Time::now();

        sweep_task = (int32_t)task_msg.task_class;
        task_seq.task_id = sweep_task;

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

        task_seq.top_left.position.x = (adt_utils.HexToDec(sweep_tl_x, sizeof(sweep_tl_x))) / 1000;
        task_seq.top_left.position.y = (adt_utils.HexToDec(sweep_tl_y, sizeof(sweep_tl_y))) / 1000;
        task_seq.below_right.position.x = (adt_utils.HexToDec(sweep_bl_x, sizeof(sweep_bl_x))) / 1000;
        task_seq.below_right.position.y = (adt_utils.HexToDec(sweep_bl_y, sizeof(sweep_bl_y))) / 1000;

        unsigned char map_id[sizeof(task_msg.map_id)];
        memset(map_id, 0, sizeof(task_msg.map_id));
        memcpy(map_id, task_msg.map_id, sizeof(task_msg.map_id));
        sweep_map_id = adt_utils.HexToDec(map_id, sizeof(task_msg.map_id));

        unsigned char traj_id[sizeof(task_msg.planning_id)];
        memset(traj_id, 0, sizeof(task_msg.planning_id));
        memcpy(traj_id, task_msg.planning_id, sizeof(task_msg.planning_id));
        sweep_traj_id = adt_utils.HexToDec(traj_id, sizeof(task_msg.planning_id));

        task_seq.map_id = sweep_map_id;
        task_seq.traj_id = sweep_traj_id;

        /* 需要先回复消息给ADT */
        struct TaskMsgReply task_reply;
        memset(&task_reply, 0, sizeof(struct TaskMsgReply));
        task_reply.start = 0xFA;
        task_reply.type = 0x04;
        task_reply.no = 0x00;

        unsigned char vehicle_id_buffer[8];
        memset(vehicle_id_buffer, 0, sizeof(vehicle_id_buffer));
        adt_utils.DecToHex(adt_utils.LOCALCARID, vehicle_id_buffer, 8);
        memcpy(task_reply.car_id, vehicle_id_buffer, sizeof(task_reply.car_id));

        task_reply.statu_mark = 0x00;
        task_reply.result = 0x01;
        task_reply.fail_reason = 0x00;

        /* heyufeidebug */
        for (int i = 0; i < 8; ++i)
        {
            task_reply.serial_num[i] = adt_buffer[60 + i];
        }
        /* 时间戳 */
        unsigned char time_stamp_buffer[8];
        long long time_stamp = adt_utils.GetTimeStamp();
        adt_utils.DecToHex(time_stamp, time_stamp_buffer, 8);
        memcpy(&task_reply.time_stamp, time_stamp_buffer, sizeof(task_reply.time_stamp));

        task_reply.valcode = 0x00;
        task_reply.end = 0xFB;

        unsigned char reply_buffer[sizeof(struct TaskMsgReply)];
        memset(reply_buffer, 0, sizeof(struct TaskMsgReply));
        memcpy(reply_buffer, &task_reply, sizeof(task_reply));
        /* 发送 */
        this->SendADT(reply_buffer, sizeof(reply_buffer));

        unsigned char start_time_stamp_buffer[8];
        memset(start_time_stamp_buffer, 0, sizeof(start_time_stamp_buffer));
        memcpy(start_time_stamp_buffer, &task_msg.start_time, sizeof(task_msg.start_time));
        long long sweep_start_time = adt_utils.HexToDec(start_time_stamp_buffer, 8);
        printf("start timestamp is %lld\n", sweep_start_time);

        unsigned char end_time_stamp_buffer[8];
        memset(end_time_stamp_buffer, 0, sizeof(end_time_stamp_buffer));
        memcpy(end_time_stamp_buffer, &task_msg.end_time, sizeof(task_msg.end_time));
        long long sweep_end_time = adt_utils.HexToDec(end_time_stamp_buffer, 8);
        printf("end timestamp is %lld\n", sweep_end_time);

        map_name = adt_utils.LongToString(sweep_map_id);
        traj_name = adt_utils.LongToString(sweep_traj_id);
        end_pr = " &";
        node_path = ros::package::getPath("communication");
        cmd_sub = "bash " + node_path + "/../../../scripts/sweeper_run.sh sweep ";

        /* 定时执行任务脚本 */
        /* 获取当前时间时间戳 */
        long long time_stamp_now = adt_utils.GetTimeStamp();
        /* 距离开始的间隔 */
        int start_left_s = adt_utils.RemainSeconds(time_stamp_now, sweep_start_time);
        if (start_left_s <= 0)
        {
            start_left_s = 3;
        }
        /* 开辟单独线程执行任务 */
        pthread_t plantask_id;
        pthread_create(&plantask_id, NULL, PlanTask, (void *)start_left_s);
    }
    else
    {
        std::cerr << "Cannot Parse Msg From ADT\n";
    }
    return 1;
}

int HandleADT::SetupADTConnection(char *server_address, int server_port)
{
    struct hostent *hptr;
    hptr = gethostbyname(server_address);
    if (!hptr)
    {
        printf("Get IP Address Error!\n");
        return -1;
    }

    struct sockaddr_in conn_addr;
    if ((socket_adt = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("socket error\n");
        return -1;
    }

    bzero(&conn_addr, sizeof(conn_addr));
    conn_addr.sin_family = AF_INET;
    conn_addr.sin_port = htons(server_port);
    conn_addr.sin_addr = *((struct in_addr *)hptr->h_addr_list[0]);

    if (connect(socket_adt, (struct sockaddr *)&conn_addr, sizeof(conn_addr)) < 0)
    {
        printf("Connect ADT Server Error\n");
        close(socket_adt);
        return false;
    }
    printf("Connect ADT Server Success\n");
    return 1;
}

int HandleADT::SendADT(unsigned char *send_msg, int len)
{
    std::vector<unsigned char> send_adt_msg;
    send_adt_msg.clear();
    adt_utils.AddEscapeCharacter(send_msg, send_adt_msg, len);
    unsigned char send_adt_buffer[send_adt_msg.size()];
    for (int i = 0; i < send_adt_msg.size(); i++)
    {
        send_adt_buffer[i] = send_adt_msg[i];
    }
    int bytes_left;
    int written_bytes;
    unsigned char *ptr;
    ptr = send_adt_buffer;
    bytes_left = send_adt_msg.size();
    while (bytes_left > 0)
    {
        /* 开始写*/
        written_bytes = write(socket_adt, ptr, bytes_left);
        if (written_bytes <= 0) /* 出错了*/
        {
            if (errno == EINTR)
            {
                written_bytes = 0;
                printf("errno = EINTR in SendADT\n");
            }    /* 中断错误 我们继续写*/
            else /* 其他错误 没有办法,只好撤退了*/
            {
                printf("errno = other in SendADT\n");
            }
        }
        bytes_left -= written_bytes;
        ptr += written_bytes; /* 从剩下的地方继续写   */
    }
    return 1;
}

int HandleADT::RecvADT(unsigned char *adt_buffer)
{
    int count = -1;

    unsigned char recv_buf[256];
    memset(recv_buf, 0, sizeof(recv_buf));

    count = read(socket_adt, recv_buf, sizeof(recv_buf));
    if (count == 0)
    {
        std::cout << "No Msg From ADT Server\n";
    }
    else if (count < 0)
    {
        std::cout << "Lose Connection With ADT Server\n";
    }

    int recv_adt_escape_char_num = adt_utils.DelEscapeCharcter(recv_buf, count);
    count -= recv_adt_escape_char_num;
    //printf("recv num = %d\n",count);
    if (count > 0)
    {
        /* do protocol*/
        memcpy(adt_buffer, recv_buf, count);
    }
    else
    {
        printf("Recv From ADT Server Fail!\n");
        return -1;
    }
    return count;
}

void HandleADT::interrupt_handler(int sig)
{
    exit(1);
}

int HandleADT::Task(char *server_address, int server_port)
{
    signal(SIGPIPE, interrupt_handler);

    int ret = -10;
    if ((ret = this->SetupADTConnection(server_address, server_port)) == true)
    {
        if ((ret = pthread_create(&thread_heartbeat_adt, NULL, HeartbeatThread, (void *)&socket_adt)) < 0)
        {
            std::cerr << "Create ADT Heartbeat Thread Error\n";
            return -1;
        }
    }
    else
    {
        return -1;
    }

    for (;;)
    {
        // std::cout << "debug00000" << std::endl;
        unsigned char adt_buffer[MAXBUFFERLEN];
        memset(adt_buffer, 0, sizeof(adt_buffer));
        if ((ret = RecvADT(adt_buffer)) == -1)
        {
            // std::cout << "debug11111" << std::endl;
            std::cout << "Recv ADT Msg Fail\n";
        }
        else
        {
            // std::cout << "debug22222" << std::endl;
            DoProtocol(adt_buffer, ret);
        }
    }
    return 1;
}
