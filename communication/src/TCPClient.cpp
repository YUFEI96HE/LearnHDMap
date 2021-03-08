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

#include "TCPClient.h"

TCPClient::TCPClient()
{
	TCPClientInit();
}

TCPClient::~TCPClient()
{
}

void TCPClient::TCPClientInit()
{
	client_sockfd = -1;
}

bool TCPClient::Setup(std::string server_address, int server_port)
{
	if (client_sockfd == -1)
	{
		client_sockfd = socket(AF_INET, SOCK_STREAM, 0);
		// std::cout << "socket: " << client_sockfd << std::endl;
		if (client_sockfd == -1)
		{
			std::cout << "Could Not Create Client Socket\n";
		}
	}

	if ((signed)inet_addr(server_address.c_str()) == -1)
	{
		struct hostent *hs;
		struct in_addr **addr_list;
		if ((hs = gethostbyname(server_address.c_str())) == NULL)
		{
			herror("GetHostByName");
			std::cout << "Failed to Resolve Jost";
			return false;
		}
		addr_list = (struct in_addr **)hs->h_addr_list;
		for (int i = 0; addr_list[i] != NULL; i++)
		{
			server.sin_addr = *addr_list[i];
			break;
		}
	}
	else
	{
		server.sin_addr.s_addr = inet_addr(server_address.c_str());
	}
	server.sin_family = AF_INET;
	server.sin_port = htons(server_port);
	if (connect(client_sockfd, (struct sockaddr *)&server, sizeof(server)) < 0)
	{
		perror("Connect Server Error");
		return false;
	}
	return true;
}

bool TCPClient::Send(unsigned char *msg, int length)
{
	if (client_sockfd != -1)
	{
		if (send(client_sockfd, msg, length, 0) < 0)
		{
			std::cout << "Send Failed: " << msg << std::endl;
			return false;
		}
	}
	else
	{
		return false;
	}
	return true;
}

int TCPClient::Receive(unsigned char *msg)
{
	unsigned char buffer[MAXBUFFERLEN];
	memset(buffer, 0, sizeof(buffer));

	//设置收发时限
	struct timeval timeout = {0, 500000}; //0.5s
	if (setsockopt(client_sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout)) < 0)
	{
		std::cerr << "Set Timeout 0.5s Error\n";
	}

	int count = -1;
	count = recv(client_sockfd, buffer, MAXBUFFERLEN, 0);

	std::cout << "RREECCVV SSIIZZEE: " << count << std::endl;
	if (count < 0)
	// if (recv(client_sockfd, buffer, MAXBUFFERLEN, 0) < 0)
	{
		std::cerr << "Recv CCU Msg Error\n";
		return -1;
	}
	else if (count == 0)
	{
		return 0;
	}
	else if (count > 0)
	{
		memcpy(msg, buffer, count);
		return count;
	}
	return -2;
}
