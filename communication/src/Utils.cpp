/*
 * @file: APPCom.h
 * 
 * @brief: comminication with app, acu-server app-client
 * 
 * @author: heyufei
 * 
 * @data: 2020-11-09
 * 
 */

#include "Utils.h"

using namespace std;

Utils::Utils()
{
	LOCALCARID = GetVehicleID();
}

Utils::~Utils()
{
}

/*添加转义字符*/
void Utils::AddEscapeCharacter(unsigned char *buffer, vector<unsigned char> &temp, int len)
{
	// unsigned char buffer[] = msg.c_str();
	unsigned char a = 0xFA; //包头
	unsigned char b = 0xFB; //报尾
	unsigned char c = 0x5C; //转义字符
	temp.push_back(a);

	for (int i = 1; i < len - 1; i++)
	{
		if (buffer[i] == 0x5C || buffer[i] == 0xFA || buffer[i] == 0xFB)
		{
			temp.push_back(c);
			temp.push_back(buffer[i]);
		}
		else
		{
			temp.push_back(buffer[i]);
		}
	}
	temp.push_back(b);
}

/* 剥离转义字符*/
int Utils::DelEscapeCharcter(unsigned char *buffer, int len)
{
	int i = 0, j = 0, cnt = 0;
	while (i < len - 1)
	{
		if ((buffer[i] == 0x5C && buffer[i + 1] == 0xFA) || (buffer[i] == 0x5C && buffer[i + 1] == 0xFB) ||
			(buffer[i] == 0x5C && buffer[i + 1] == 0x5C))
		{
			buffer[j++] = buffer[++i];
			i++;
			cnt++;
		}
		else
		{
			buffer[j] = buffer[i];
			i++;
			j++;
		}
	}
	buffer[j] = buffer[i];
	return cnt;
}

/* 十进制 十六进制 互转 */
long long Utils::HexToDec(unsigned char *buffer, unsigned int len)
{
	unsigned char str[2 * len]; //len=2, 这里用4个字节表示整数，一个字节一个位的十六进制。ABCD。无符号。
	memset(str, 0, 2 * len);
	unsigned int sum = 2 * len; //4
	unsigned int i = 0;
	long long temp = 0;

	int ins = len - 1;
	for (int im = sum - 1; im >= 0; im = im - 2) //从低位开始，存在高位3 1
	{
		str[im - 1] = buffer[ins]; //2
		str[im] = buffer[ins];	   //3 。第一个字节超过16的将进位

		str[im - 1] = (str[im - 1] >> 4) & 0x0F; //2：除以16，留下进位。
		str[im] = str[im] & 0x0F;				 //3，除掉高位

		ins--;
	}
	for (i = 0; i < sum; i++) //遍历4个字节，低位存在最后，高位存在最前
	{
		temp = temp + str[sum - i - 1] * pow(16, i);
	}
	return temp;
}

bool Utils::DecToHex(long long a, unsigned char *buffer, unsigned int len)
{
	unsigned char str[2 * len];
	memset(str, 0, 2 * len);
	unsigned int i = 0;
	long long temp = 0;
	while (a > 0)
	{
		temp = a % 16; // 取余数
		str[i++] = temp;
		a = a >> 4;
	}
	if ((i % 2) == 1)
	{
		str[i++] = 0;
	}
	// 翻转 , 将高位与地位进行翻转
	int t = 0;
	for (int j = 0; j < i / 2; j++)
	{
		t = str[j];
		str[j] = str[i - j - 1];
		str[i - j - 1] = t;
	}
	memset(buffer, 0, len);
	int ins = len - 1;
	for (int im = i - 1; im >= 0; im = im - 2)
	{
		buffer[ins] = (str[im - 1] << 4) & 0xF0;
		buffer[ins] = (((str[im]) & 0x0F) | buffer[ins]);
		ins--;
	}
	return true;
}

/* 获取时间戳 */
long long Utils::GetTimeStamp()
{
	struct timeval tv;
	gettimeofday(&tv, NULL); //该函数在sys/time.h头文件中
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

long long Utils::GetVehicleID()
{
	char id_filepath[512] = "/etc/car_id.ini";
	/* 判断文件是否存在 */
	if (access(id_filepath, 0) == -1)
	{
		std::cerr << "Could Not Find VechileID File In Path: /etc/\n";
	}
	std::ifstream id_file;
	id_file.open(id_filepath, ifstream::in);
	if (!id_file.is_open())
	{
		std::cerr << "Could Not Open VechileID File\n";
	}

	char vehicle_id[101];
	memset(vehicle_id, 0, sizeof(vehicle_id));

	id_file.getline(vehicle_id, 101);
	if (strcmp(vehicle_id, "") == 0)
	{
		std::cerr << "VehicleID File is Empty, Error\n";
	}
	id_file.close();
	return atoll(vehicle_id);
}

int Utils::RunShellCmd(std::string cmd)
{
	char line[300];
	FILE *fp;
	//string cmd = "bash /home/heyufei/Documents/catkin_ws/src/AutoSweeper/modules/communication/scripts/sweeper_run.sh map";
	//引号内是你的linux指令
	// 系统调用
	const char *sysCommand = cmd.data();
	if ((fp = popen(sysCommand, "r")) == NULL)
	{
		cout << "error" << endl;
		return -1;
	}
	while (fgets(line, sizeof(line) - 1, fp) != NULL)
	{
		cout << line;
	}
	pclose(fp);
	return 0;
}

std::string Utils::LongToString(long long param)
{
	std::stringstream stream;
	std::string result;
	stream << param;
	stream >> result;
	return result;
}


int Utils::PrintTimestamp(long long stamp_start, long long stamp_end)
{
	stamp_start = stamp_start / 1000;
	stamp_end = stamp_end / 1000;

	time_t time_s = (time_t)stamp_start;
	time_t time_e = (time_t)stamp_end;

	struct tm tm_start = *localtime(&time_s);
	struct tm tm_end = *localtime(&time_e);

	char s[100];
	strftime(s, sizeof(s), "%Y-%m-%d %H:%M:%S", &tm_start);
	strftime(s, sizeof(s), "%Y-%m-%d %H:%M:%S", &tm_end);
	printf("%d: %s\n", (int)time_s, s);
	printf("%d: %s\n", (int)time_e, s);

	int reamin_time = difftime(time_e, time_s);


	
	// time_t time_s = (time_t)stamp_start;
	// time_t time_e = (time_t)stamp_end;
	// int remain_time = difftime(time_e, time_s);
	return reamin_time;
}

int Utils::RemainSeconds(long long stamp_start, long long stamp_end)
{
	stamp_start = stamp_start / 1000;
	stamp_end = stamp_end / 1000;

	time_t time_s = (time_t)stamp_start;
	time_t time_e = (time_t)stamp_end;

	int reamin_time = difftime(time_e, time_s);
	
	return reamin_time;
}
