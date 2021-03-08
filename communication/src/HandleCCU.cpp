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

#include "HandleCCU.h"

class MessageCenter HandleCCU::ccu_center;
class Utils HandleCCU::ccu_utils;
double HandleCCU::voltage;
double HandleCCU::current;
double HandleCCU::wheel_speed;
double HandleCCU::vehicle_speed;
int HandleCCU::ultrasound[8];
int HandleCCU::fault_code;
int HandleCCU::forward_backward;
pthread_t HandleCCU::thread_heartbeat_ccu;
int HandleCCU::socket_ccu;

HandleCCU::HandleCCU() {}
HandleCCU::~HandleCCU() {}

void *HandleCCU::HeartbeatThread(void *params)
{
    int *client = (int *)params;

    struct LocalizationMsg local;
    unsigned char heartbeat_msg[sizeof(struct LocalizationMsg)];

    for (;;)
    {
        // mutex.lock();
        /* 数据准备打包 */
        memset(&local, 0, sizeof(struct LocalizationMsg));

        /* 包头 报文标识 序号 来源 */
        local.start = 0xFA;
        local.type = 0x35;
        local.no = 0x01;
        local.datasource = 0x01;

        /* 加速度计xyz 陀螺仪xyz */
        unsigned int u_imu_data_linear_acceleration_x = ccu_center.imu_data_linear_acceleration_x * 100;
        unsigned int u_imu_data_linear_acceleration_y = ccu_center.imu_data_linear_acceleration_y * 100;
        unsigned int u_imu_data_linear_acceleration_z = ccu_center.imu_data_linear_acceleration_z * 100;
        unsigned int u_imu_data_angular_velocity_x = ccu_center.imu_data_angular_velocity_x * 100;
        unsigned int u_imu_data_angular_velocity_y = ccu_center.imu_data_angular_velocity_y * 100;
        unsigned int u_imu_data_angular_velocity_z = ccu_center.imu_data_angular_velocity_z * 100;
        // unsigned int u_wheel_speed = ccu_center.wheel_speed * 100;
        unsigned char linear_acceleration_x_buffer[2];
        unsigned char linear_acceleration_y_buffer[2];
        unsigned char linear_acceleration_z_buffer[2];
        unsigned char angular_velocity_x_buffer[2];
        unsigned char angular_velocity_y_buffer[2];
        unsigned char angular_velocity_z_buffer[2];
        // unsigned char wheel_speed_buffer[2];
        ccu_utils.DecToHex(u_imu_data_linear_acceleration_x, linear_acceleration_x_buffer, 2);
        ccu_utils.DecToHex(u_imu_data_linear_acceleration_y, linear_acceleration_y_buffer, 2);
        ccu_utils.DecToHex(u_imu_data_linear_acceleration_z, linear_acceleration_z_buffer, 2);
        ccu_utils.DecToHex(u_imu_data_angular_velocity_x, angular_velocity_x_buffer, 2);
        ccu_utils.DecToHex(u_imu_data_angular_velocity_y, angular_velocity_y_buffer, 2);
        ccu_utils.DecToHex(u_imu_data_angular_velocity_z, angular_velocity_z_buffer, 2);
        // ccu_utils.DecToHex(u_wheel_speed, wheel_speed_buffer, 2);
        memcpy(&local.imu_linear_x, linear_acceleration_x_buffer, sizeof(local.imu_linear_x));
        memcpy(&local.imu_linear_y, linear_acceleration_y_buffer, sizeof(local.imu_linear_y));
        memcpy(&local.imu_linear_z, linear_acceleration_z_buffer, sizeof(local.imu_linear_z));
        memcpy(&local.imu_angular_x, angular_velocity_x_buffer, sizeof(local.imu_angular_x));
        memcpy(&local.imu_angular_y, angular_velocity_y_buffer, sizeof(local.imu_angular_y));
        memcpy(&local.imu_angular_z, angular_velocity_z_buffer, sizeof(local.imu_angular_z));
        // memcpy(&local.velocity, wheel_speed_buffer, sizeof(local.velocity));

        /* 轮速 控制器温度 磁针xy （暂时没这些数据，置0留接口） */
        for (int m = 0; m < 2; ++m)
        {
            local.velocity[m] == 0x00;
            local.temperature[m] == 0x00;
            local.imu_magnet_x[m] == 0x00;
            local.imu_magnet_y[m] == 0x00;
        }

        /* 超声波数据（没必要重新发送给CUU，置0留接口） */
        for (int i = 0; i < 12; ++i)
        {
            local.gpsinfo[i] = 0x00;
        }

        /* 时间戳 */
        unsigned char time_stamp_buffer[8];
        long long time_stamp = ccu_utils.GetTimeStamp();
        ccu_utils.DecToHex(time_stamp, time_stamp_buffer, 8);
        memcpy(&local.time_stamp, time_stamp_buffer, sizeof(local.time_stamp));

        /* 校验码 包尾 */
        local.valcode = 0x00;
        local.end = 0x00;

        /* 在这里发送 */
        memcpy(heartbeat_msg, &local, sizeof(struct LocalizationMsg));

        SendCCU(heartbeat_msg, sizeof(heartbeat_msg));
        sleep(1);
    }
    return NULL;
}

int HandleCCU::DoProtocol(unsigned char *ccu_buffer, int len)
{
    if (ccu_buffer[0] != 0xFA)
    {
        return -1;
    }

    struct VehicleBaseMsg vehicle_msg;
    memset(&vehicle_msg, 0, len);
    memcpy(&vehicle_msg, ccu_buffer, len);

    /* 电压电流 */
    voltage = double((vehicle_msg.voltage[0] * 256 + vehicle_msg.voltage[1])) / 100;
    current = double((vehicle_msg.current[0] * 256 + vehicle_msg.current[1])) / 100;
    // todo
    // battery = 0.0;
    /* 车速 */
    signed short wheel = (signed short)ntohs(vehicle_msg.wheel_speed);
    vehicle_speed = wheel_speed = int(wheel) / 100.0; /* 单位cm转m */

    /* 超声波数据 */
    for (int i = 0; i < 8; i++)
    {
        int a = 0;
        int b = 0;
        if (vehicle_msg.ultrasonic[i * 2] < 0)
        {
            a = vehicle_msg.ultrasonic[i * 2] + 256;
        }
        else
        {
            a = vehicle_msg.ultrasonic[i * 2];
        }
        if (vehicle_msg.ultrasonic[i * 2 + 1] < 0)
        {
            b = vehicle_msg.ultrasonic[i * 2 + 1] + 256;
        }
        else
        {
            b = vehicle_msg.ultrasonic[i * 2 + 1];
        }
        ultrasound[i] = a * 256 + b;
    }

    int f1 = 0;
    int f2 = 0;
    //故障码
    if (vehicle_msg.faultcode[0] < 0)
    {
        f1 = vehicle_msg.faultcode[0] + 256;
    }
    else
    {
        f1 = vehicle_msg.faultcode[0];
    }
    if (vehicle_msg.faultcode[1] < 0)
    {
        f2 = vehicle_msg.faultcode[1] + 256;
    }
    else
    {
        f2 = vehicle_msg.faultcode[1];
    }
    fault_code = f1 * 256 + f2;

    /* 前进后退 */
    forward_backward = vehicle_msg.forward_backward;

    return 0;
}

int HandleCCU::SetupCCUConnection(char *server_address, int server_port)
{
    struct sockaddr_in conn_addr;
    if ((socket_ccu = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("ccu socket error\n");
        return false;
    }

    bzero(&conn_addr, sizeof(conn_addr));
    conn_addr.sin_family = AF_INET;
    conn_addr.sin_port = htons(server_port);
    conn_addr.sin_addr.s_addr = inet_addr(server_address);

    if (connect(socket_ccu, (struct sockaddr *)&conn_addr, sizeof(conn_addr)) < 0)
    {
        printf("Connect CCU Server Fail\n");
        close(socket_ccu);
        return false;
    }
    printf("Connect CCU Server Success\n");
    struct timeval timeout = {0, 500000};
    int ret = setsockopt(socket_ccu, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));
    return true;
}

int HandleCCU::SendCCU(unsigned char *send_msg, int len)
{
    std::vector<unsigned char> send_ccu_msg;
    send_ccu_msg.clear();
    ccu_utils.AddEscapeCharacter(send_msg, send_ccu_msg, len);
    unsigned char send_ccu_buffer[send_ccu_msg.size()];
    for (int i = 0; i < send_ccu_msg.size(); ++i)
    {
        send_ccu_buffer[i] = send_ccu_msg[i];
    }
    int send_num = 0;
    int bytes_left;
    int written_bytes;
    unsigned char *ptr;
    ptr = send_ccu_buffer;
    bytes_left = send_ccu_msg.size();
    while (bytes_left > 0)
    {
        /* 开始写*/
        send_num++;
        written_bytes = write(socket_ccu, ptr, bytes_left);
        if (written_bytes <= 0) /* 出错了*/
        {
            if (errno == EINTR)
            {
                written_bytes = 0;
                printf("errno = EINTR\n");
            } /* 中断错误 我们继续写*/

            else /* 其他错误 没有办法,只好撤退了*/
            {
                printf("other error\n");
                
            }
        }
        bytes_left -= written_bytes;
        ptr += written_bytes; /* 从剩下的地方继续写 */
    }
    return send_num;
}

int HandleCCU::RecvCCU(unsigned char *ccu_buffer)
{
    int count = -1;
    int len = sizeof(struct VehicleBaseMsg);
    unsigned char recv_buf[MAXBUFFERLEN];
    memset(recv_buf, 0, sizeof(recv_buf));
    /* read return value: >0 bytesize; =0 对端关闭socket；
    = -1 看errno: errno==ENITR表示系统当前终端直接忽略
    errno==EAGAIN表示接收缓存区没有数据，非阻塞的socket则直接忽略  */
    count = read(socket_ccu, (void *)recv_buf, 64);

    int recv_ccu_escape_char_num = ccu_utils.DelEscapeCharcter(recv_buf, count);
    count -= recv_ccu_escape_char_num;

    if (count > 0)
    {
        memcpy(ccu_buffer, recv_buf, count);
        return count;
    }
    else
    {
        printf("recv from ccu fail!\n");
        return -1;
    }
    return count;
}

int HandleCCU::Task(char *server_address, int server_port)
{
    /* local params */
    int ret = -10;
    /* build connection */
    if ((ret = this->SetupCCUConnection(server_address, server_port)) == true)
    {
        if ((ret = pthread_create(&thread_heartbeat_ccu, NULL, HeartbeatThread, (void *)&socket_ccu)) < 0)
        {
            std::cerr << "Create CCU Heartbeat Thread Error\n";
            return -1;
        }
    }
    else
    {
        return -1;
    }
    
    /* send heartbeat */

    for (;;)
    {
        // mutex.lock();
        unsigned char ccu_buffer[MAXBUFFERLEN];
        memset(ccu_buffer, 0, sizeof(ccu_buffer));

        if ((ret = this->RecvCCU(ccu_buffer)) == -1)
        {
            std::cout << "Recv CCU Msg Fail\n";
            continue;
        }
        else
        {
            DoProtocol(ccu_buffer, ret);
        }

        //TODO 设置包头包尾检测
        
    }
    return 1;
}
