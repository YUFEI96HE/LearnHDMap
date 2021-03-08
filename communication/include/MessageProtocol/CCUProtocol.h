/*
 * @file: 
 * 
 * @brief: 
 * 
 * @author: heyufei
 * 
 * @data: 2020-11-11
 * 
 */
#ifndef CCU_PROTOCOL_H
#define CCU_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>
#include <string>

const std::string CCU_ADDR = "192.168.43.48";
const int CCU_PORT = 10008;

//CCU 发到 ACU的消息 车辆的基本信息
struct VehicleBaseMsg
{
	uint8_t start; //包头0xFA      0
	uint8_t type;  //报文标识 0x30    1
	uint8_t no;	   //0-255循环    2
	uint8_t datasource; //数据来源   ACU0x01 APP0x11 ADT0x21    3
	uint8_t car_id[8];	//车辆ID     4-11
	uint8_t imut_x[2]; //加速度计x        12
	uint8_t imut_y[2]; //加速度计y    14
	uint8_t imut_z[2]; //加速度计z    16
	uint8_t imut_t[2]; //温度         18
	uint8_t imua_x[2]; //陀螺仪x;     20
	uint8_t imua_y[2]; //陀螺仪y;     22
	uint8_t imua_z[2]; //陀螺仪z;     24
	uint8_t imuh_x[2]; //磁针x;       26
	uint8_t imuh_y[2]; //磁针y;       28
	uint8_t ultrasonic[16]; //超声波雷达数据    30-45
	int16_t wheel_speed; //车轮转速，浮点数×100    46
	uint8_t temperature[2]; //控制器温度     48
	uint8_t io; //从高到低 推杆上限(左转)，推杆下限（右转），车子状态（开关），防撞杆，低四位暂时保留   50
	uint8_t voltage[2]; //电压      51
	uint8_t current[2]; //电流      53
	uint8_t faultcode[2];	  //故障码      55
	uint8_t forward_backward; //前进后退标志位     57
	uint8_t spare[4]; //备用      58-61
	uint8_t valcode;  //校验码 0x00      62
	uint8_t end;	  //包尾0xFB      63
};

//ACU 发到 CCU的消息
struct LocalizationMsg
{
	uint8_t start;			 //包头0xFA
	uint8_t type;			 // 报文标识 0x35
	uint8_t no;				 //0-255循环
	uint8_t datasource;		 //数据来源
	uint8_t imu_linear_x[2]; //加速度计
	uint8_t imu_linear_y[2];
	uint8_t imu_linear_z[2];
	uint8_t temperature[2];
	uint8_t imu_angular_x[2]; //陀螺仪
	uint8_t imu_angular_y[2];
	uint8_t imu_angular_z[2];
	uint8_t imu_magnet_x[2]; //磁力计
	uint8_t imu_magnet_y[2];
	uint8_t gpsinfo[12];
	uint8_t velocity[2];
	uint8_t time_stamp[8]; //时间戳
	uint8_t valcode;	   // 校验码 0x00
	uint8_t end;		   //包尾0xFB
};

struct ControlRawData
{
	uint8_t start; //包头0xFA
	uint8_t type;  // 报文标识 0x31
	uint8_t no;	  // 0-255循环
	uint8_t datasource;
	uint8_t quadrant; // 象限 0-4，0-不操作，1,4-前进，2,3-后退
	uint8_t angle;	 // 角度 0-90，1,2向右，3,4向左
	uint8_t velocity_lvl; // 0-10 速度等级
	uint8_t light_mark; //大灯标志位 0关闭 1打开
	uint8_t safe_switch;	//安全防护模块 0开启 1关闭
	uint8_t sweep_task;		//扫地车任务 0开启 1关闭  bite0喇叭 bite1洒水 Bite2震尘 bite3扫地
	uint8_t true_velocity;	//真实速度
	uint8_t spare[2]; //备用
	uint8_t valcode;	 // 校验码 0x00
	uint8_t end;		 //包尾0xFB
};

#endif
