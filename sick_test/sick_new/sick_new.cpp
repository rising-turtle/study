//============================================================================
// Name        : sick_new.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C, Ansi-style
//============================================================================


#include <stdio.h>
#include <string>
#include <iostream>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/time.h>

#define INVALID_SOCKET -1

using namespace std;

int main(void) {

	struct timeval start, end;
	long mtime, seconds, useconds;

	string SICK_ip = "192.168.1.2";
	int SICK_port = 2112;

	int	m_hSock = socket(AF_INET, SOCK_STREAM, 0);
	if(m_hSock==INVALID_SOCKET)
	{
		cout<<"Invalid socket!"<<endl;
		return 0;
	}

	struct sockaddr_in otherAddress;
	otherAddress.sin_family = AF_INET;
	otherAddress.sin_port = htons(SICK_port);
	otherAddress.sin_addr.s_addr = inet_addr(SICK_ip.c_str());

	// Try to connect:
	int connectNow = ::connect( m_hSock , (struct sockaddr *)&otherAddress,sizeof(otherAddress));
	if(connectNow == INVALID_SOCKET)
	{
		cout<<"connection error"<<endl;
		return 0;
	}

	char msg[] = {"sRN LMDscandata"};
	int writeNow = ::send( m_hSock, (char*)msg, sizeof(msg), 0);

	if(writeNow == INVALID_SOCKET)
	{
		cout<<"write error"<<endl;
		return 0;
	}

	char Buffer[16*1024];
	int readNow = ::recv( m_hSock, ((char*)Buffer), sizeof(Buffer), 0);

	if(readNow == INVALID_SOCKET)
	{
		cout<<"write error"<<endl;
		return 0;
	}

	while(1 )
	{
		gettimeofday(&start, NULL);

		int writeNow = ::send( m_hSock, (char*)msg, sizeof(msg), 0);
		if(writeNow == INVALID_SOCKET)
		{
			cout<<"write error"<<endl;
			return 0;
		}
		int readNow = ::recv( m_hSock, ((char*)Buffer), sizeof(Buffer), 0);
		if(readNow == INVALID_SOCKET)
		{
			cout<<"write error"<<endl;
			return 0;
		}
		gettimeofday(&end, NULL);

		seconds  = end.tv_sec  - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;

		mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
		printf("Elapsed time: %ld ms\n", mtime);
	}


	// Delete socket:
	if (m_hSock != -1)
	{
		shutdown(m_hSock, SHUT_RDWR  );
		::close( m_hSock );
		m_hSock = -1;
	}

	puts("!!!Hello World!!!");
	return 0;
}
