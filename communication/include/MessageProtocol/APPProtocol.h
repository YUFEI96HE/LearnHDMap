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
#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H
#include <stdint.h>
#include <string>


//=====ACU 发送车辆状态信息到 APP=====//
struct VehicleStatus
{
	uint8_t start;
	uint8_t type;			//数据类型
	uint8_t no;				//数据包序号
	uint8_t id[8];			//车辆ID
	uint8_t statu_mark;		//车辆增删标志位
	uint8_t gps[12];		//GPS经纬度
	uint8_t position[12];	//空间坐标（X/Y/Z）
	uint8_t battery[2];		//电量
	uint8_t voltage[2];		//电压
	uint8_t current[2];		//电流
	uint8_t wheel_speed[2]; //轮速
	uint8_t angle[2];		//角度
	uint8_t error_code[2];	//故障码
	uint8_t time_stamp[8];	//时间戳
	uint8_t ultrasound[16]; //超声波数据
	uint8_t val_code;		//校验码
	uint8_t end;
};

struct VehicleStatusReply
{
	uint8_t start;
	uint8_t type;		//数据类型
	uint8_t no;			//数据包序号
	uint8_t id[8];		//车辆ID
	uint8_t statu_mark; //车辆增删标志位
	uint8_t time_stamp[8];
	uint8_t val_code;
	uint8_t end;
};

//=====APP 发送任务命令到 ACU=====//

/* 1- 实时构建命令（用于构建地图和路径） */
struct MapTrajTask
{
	uint8_t start;
	uint8_t type; //表示实时构建命令
	uint8_t no;
	uint8_t id[8];
	uint8_t statu_mark;
	uint8_t task;		//1-构建地图 2-录制轨迹
	uint8_t operation;	//1-开始构建 2-结束构建
	uint8_t file_id[8]; //文件ID
	uint8_t time_stamp[8];
	uint8_t val_code;
	uint8_t end;
};

struct MapTrajTaskReply
{
	uint8_t start;
	uint8_t type;
	uint8_t no;
	uint8_t id[8];
	uint8_t statu_mark;
	uint8_t result; //1-收到命令开始构建 2-收到命令构建失败
	uint8_t reason; //上一个字节失败的原因
	uint8_t time_stamp[8];
	uint8_t val_code;
	uint8_t end;
};

/* 2- 任务计划执行命令 （用于执行扫地相关工作） */
struct SweepTask
{
	uint8_t start;
	uint8_t type; //表示实时构建命令
	uint8_t no;
	uint8_t id[8];
	uint8_t statu_mark;
	uint8_t task; //0-默认无操作 1-洒水 2-震尘 3-扫地
	uint8_t start_time[8];
	uint8_t end_time[8];
	uint8_t map_id[8];	 //任务执行所需的地图文件ID
	uint8_t plan_id[8];	 //路径规划任务所需文件ID 0表示不启动路径规划
	uint8_t tl_point[8]; //扫地区域左上角坐标点
	uint8_t br_point[8]; //右下角坐标点
	uint8_t time_stamp[8];
	uint8_t val_code;
	uint8_t end;
};

struct SweepTaskReply
{
	uint8_t start;
	uint8_t type;
	uint8_t no;
	uint8_t id[8];
	uint8_t statu_mark;
	uint8_t result; //1-收到命令开始执行 2-收到命令执行失败
	uint8_t reason; //上一个字节失败的原因
	uint8_t task_time;
	uint8_t time_stamp[8];
	uint8_t val_code;
	uint8_t end;
};

/* 3- 获取任务计划列表命令 （预留） */

/* 4- 获取具体任务信息命令 （预留） */

// struct APPMsgOrder
// {
//         uint8_t start;
//         uint8_t click; //0第一次按下开始建图 1第二次按下结束建图
//         uint8_t task;  //APP下发 建图 录轨 作业 命令
//         uint8_t child_task;
//         uint8_t numer; //这是干嘛的
//         uint8_t vehicle_id[8];
//         uint8_t statu_mark;
//         uint8_t file_id[8]; //建图 录轨等文件的ID
//         uint8_t time_stamp[8];
//         uint8_t valcode;
//         uint8_t end;
// };

// //Feedback - Mapping TrajRecorder WorkMode
// struct APPMsgFB
// {
//         uint8_t start;         //包头0xFA
//         uint8_t click;         //0第一次按下开始建图 1第二次按下结束建图
//         uint8_t type;          //报文标识-确定执行何种任务 建图-录轨-作业
//         uint8_t number;        //序列号-不知道干嘛的
//         uint8_t vehicle_id[8]; //车辆ID
//         uint8_t statu_mark;    //增删标志位
//         uint8_t stat;          //工作状态 进行中 完成
//         uint8_t error_code;    //监控模块的错误码
//         uint8_t time_stamp[8]; //当前时间戳 毫秒
//         uint8_t valcode;       //校验码 0x00
//         uint8_t end;           //包尾0xFB
// };

/* 关机及急停命令 */
struct OFFMsg
{
	uint8_t start;
	uint8_t type;
	uint8_t no;
	uint8_t car_id[8];
	uint8_t statu_mark;
	uint8_t onoff;				// 相关指令 1-关机 2-急停
	uint8_t time_stamp[8];		// 时间戳
	uint8_t valcode;
	uint8_t end;
};

struct OFFMsgReply
{
	uint8_t start;
	uint8_t type;
	uint8_t no;		
	uint8_t car_id[8];	
	uint8_t statu_mark;
	uint8_t result;	
	uint8_t reason;
	uint8_t time_stamp[8];
	uint8_t valcode;
	uint8_t end;
};

/* 参数配置 */
struct ParamConfig
{
	uint8_t start;
	uint8_t type;
	uint8_t no;
	uint8_t car_id[8];
	uint8_t statu_mark;
	uint8_t param_num[2];
	uint8_t param_list[12];
	uint8_t time_stamp[8];
	uint8_t valcode;
	uint8_t end;
};

struct ParamConfigReply
{
	uint8_t start;
	uint8_t	type;
	uint8_t no;
	uint8_t car_id[8];
	uint8_t	statu_mark;
	uint8_t result;
	uint8_t reason;
	uint8_t time_stamp[8];
	uint8_t	valcode;
	uint8_t end;
};

#endif
