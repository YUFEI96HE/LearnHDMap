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

#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#define MAXBUFFERLEN 4096

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netdb.h>
#include <vector>
#include <string.h>


class TCPClient
{
public:
	TCPClient();
	~TCPClient();

	int client_sockfd;

	void TCPClientInit();

	bool Setup(std::string server_address, int server_port);
	bool Send(unsigned char *msg, int length);
	int Receive(unsigned char *msg);
	void Exit();

private:
	struct sockaddr_in server;
};

#endif