      /*
 * @file: 
 * 
 * @brief: 
 * 
 * 
 * @author: heyufei
 * 
 * @data: 2020-11-11 11:08
 * 
 */
#ifndef ADT_PROTOCOL_H
#define ADT_PROTOCOL_H

#include <stdint.h>
#include <string>

const std::string ADT_ADDR = "admin.techinao.com";
const int ADT_PORT = 9999;

/* 车辆状态信息数据 */
struct VehicleStatuMsg
{
    uint8_t start;            //包头0xFA      0
    uint8_t type;             //报文标识 0x30    1
    uint8_t no;               //0-255循环    2
    uint8_t car_id[8];        //车辆ID     4-11
    uint8_t statu_mark;       //车辆增删标志位
    uint8_t gps[12];          //GPS经纬度
    uint8_t position[12];     //空间坐标（X/Y/Z）
    uint8_t battery[2];       //电量
    uint8_t voltage[2];       //电压
    uint8_t current[2];       //电流
    uint8_t vehicle_speed[2]; //车速
    uint8_t angle[2];         //角度
    uint8_t faultcode[2];     //故障码      55
    uint8_t time_stamp[8];    //时间戳
    uint8_t ultrasonic[16];   //超声波雷达数据    30-45
    uint8_t wheel_speed[2];   //车速
    uint8_t imu_a_x[2];       //加速度计x        12
    uint8_t imu_a_y[2];       //加速度计y    14
    uint8_t imu_a_z[2];       //加速度计z    16
    uint8_t imu_g_x[2];       //陀螺仪x;     20
    uint8_t imu_g_y[2];       //陀螺仪y;     22
    uint8_t imu_g_z[2];       //陀螺仪z;     24
    uint8_t remain_sweep;     //清扫箱剩余状态
    uint8_t remain_water;     //水箱剩余状态
    uint8_t remain_area;      //剩余清扫区域
    uint8_t valcode;          //校验码 0x00      62
    uint8_t end;              //包尾0xFB      63
};

/* 车辆状态信息回复 */
struct VehicleStatuMsgReply
{
    uint8_t start;         //包头0xFA      0
    uint8_t type;          //报文标识 0x30    1
    uint8_t no;            //0-255循环    2
    uint8_t car_id[8];     //车辆ID     4-11
    uint8_t statu_mark;    //车辆增删标志位
    uint8_t time_stamp[8]; //时间戳
    uint8_t valcode;       //校验码 0x00      62
    uint8_t end;           //包尾0xFB      63
};

/* 摇杆控制命令 */
struct JoystickMsg
{
    uint8_t start;      //包头0xFA      0
    uint8_t type;       //报文标识 0x30    1
    uint8_t no;         //0-255循环    2
    uint8_t car_id[8];  //车辆ID     4-11
    uint8_t statu_mark; //车辆增删标志位
    uint8_t ccu_type;
    uint8_t ccu_no;
    uint8_t ccu_src;
    uint8_t ccu_quad;
    uint8_t ccu_angle;
    uint8_t ccu_speed_lvl;
    uint8_t ccu_light_mark;
    uint8_t ccu_secure_switch;
    uint8_t ccu_sweep_task;
    uint8_t ccu_spare[2]; //备用      58-61
    uint8_t ccu_valcode;
    uint8_t time_stamp[8]; //时间戳
    uint8_t valcode;       //校验码 0x00      62
    uint8_t end;           //包尾0xFB      63
};

/* 摇杆控制回复 */
struct JoystickMsgReply
{
    uint8_t start;      //包头0xFA      0
    uint8_t type;       //报文标识 0x30    1
    uint8_t no;         //0-255循环    2
    uint8_t car_id[8];  //车辆ID     4-11
    uint8_t statu_mark; //车辆增删标志位
    uint8_t result;
    uint8_t time_stamp[8];
    uint8_t valcode;
    uint8_t end;
};

/* 视频开启关闭命令 */
struct VideoSwitchMsg
{
    uint8_t start;      //包头0xFA      0
    uint8_t type;       //报文标识 0x30    1
    uint8_t no;         //0-255循环    2
    uint8_t car_id[8];  //车辆ID     4-11
    uint8_t statu_mark; //车辆增删标志位
    uint8_t video_switch;
    uint8_t time_stamp[8];
    uint8_t valcode;
    uint8_t end;
};

/* 视频开关命令回复 */
struct VideoSwitchMsgReply
{
    uint8_t start;      //包头0xFA      0
    uint8_t type;       //报文标识 0x30    1
    uint8_t no;         //0-255循环    2
    uint8_t car_id[8];  //车辆ID     4-11
    uint8_t statu_mark; //车辆增删标志位
    uint8_t result;
    uint8_t time_stamp[8];
    uint8_t valcode;
    uint8_t end;
};

/* 任务计划执行命令 */
struct TaskMsg
{
    uint8_t start;      //包头0xFA      0
    uint8_t type;       //报文标识 0x30    1
    uint8_t no;         //0-255循环    2
    uint8_t car_id[8];  //车辆ID     4-11
    uint8_t statu_mark; //车辆增删标志位
    uint8_t task_class;
    uint8_t start_time[8];
    uint8_t end_time[8];
    uint8_t map_id[8];
    uint8_t planning_id[8];
    uint8_t area_top_left[8];
    uint8_t area_blow_right[8];
    uint8_t time_stamp[8];
    uint8_t valcode;
    uint8_t end;
};

/* 任务计划执行回复 */
struct TaskMsgReply
{
    uint8_t start;      //包头0xFA      0
    uint8_t type;       //报文标识 0x30    1
    uint8_t no;         //0-255循环    2
    uint8_t car_id[8];  //车辆ID     4-11
    uint8_t statu_mark; //车辆增删标志位
    uint8_t result;
    uint8_t fail_reason;
    uint8_t serial_num[8];
    uint8_t time_stamp[8];
    uint8_t valcode;
    uint8_t end;
};

#endif