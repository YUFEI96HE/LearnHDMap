/*
 * @file: Utils.h
 * 
 * @brief: utils
 * 
 * @author: heyufei
 * 
 * @data: 2020-11-09
 * 
 */

#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

using namespace std;
class Utils
{
public:
	Utils();
	~Utils();

	long long LOCALCARID;
	/* 剥离 添加   转义字符 */
	void AddEscapeCharacter(unsigned char *msg, vector<unsigned char> &temp, int len);
	int DelEscapeCharcter(unsigned char *buffer, int len);
	/* 类型转换 */
	unsigned char *StringToCharArray(std::string &src, int &len);
	/* 十进制 十六进制 转换*/
	long long HexToDec(unsigned char *buffer, unsigned int len);
	bool DecToHex(long long a, unsigned char *buffer, unsigned int len);
	bool DecToHex(unsigned char *buffer, int32_t a, unsigned int len);
	/* 读取配置文件 */
	bool ReadConfigFile(char *cfgfilepath, string &key, string &value);
	/* 执行shell命令 */
	int ExecuteShell(char *shcmd);
	/* 获取时间戳 */
	long long GetTimeStamp();

	long long GetVehicleID();

	int RunShellCmd(std::string cmd);

	std::string LongToString(long long param);

	int PrintTimestamp(long long stamp_start, long long stamp_end);
	int RemainSeconds(long long stamp_start, long long stamp_end);

private:
};

#endif
