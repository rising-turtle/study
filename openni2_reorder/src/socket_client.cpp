#include "socket_client.h"
#include <iostream>
#include <arpa/inet.h> //inet_addr
#include <string.h>
#include <unistd.h>
#include <stdio.h>

using namespace std;

CSocketClient::CSocketClient(): 
m_bConneted(false)
{
}
CSocketClient::~CSocketClient()
{
    if(m_bConneted) 
        close(m_socket_desc);
}

void CSocketClient::connect(char* hip, int hport)
{
    // Create socket
    m_socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (m_socket_desc == -1)
    {
        printf("Could not create socket\n");
        return ;
    }
    // Hoster info
    sockaddr_in server;
    server.sin_addr.s_addr = inet_addr(hip);
    server.sin_family = AF_INET; 
    server.sin_port = htons(hport);
    
    if(::connect(m_socket_desc, (sockaddr*)&server, sizeof(server)) < 0)
    {
        cerr<<"socket_client.cpp: could not connect to ip: "<<hip<<" with port: "<<hport<<endl;
        return ;
    }else
    {
        cout<<"socket_client.cpp: succeed connect to ip: "<<hip<<" with port: "<<hport<<endl;
    }
    m_bConneted = true;
}

bool CSocketClient::sendData(char* sendbuf, unsigned int len)
{
    bool ret = false;
    if(!m_bConneted)
    {
        cout<<"socket_client.cpp: not connect yet!"<<endl;
        return false;
    }
    unsigned int iResult = 0;
    iResult = send(m_socket_desc, sendbuf, len, 0);
    if(iResult <= 0)
        cout<<"socket_client.cpp: failed to send data! iResult < 0 "<<endl;
    else if(iResult != len)
        cout<<"socket_client.cpp: failed to send complete data! iResult < len "<<endl;
    else
    { 
        ret = true;
        cout<<"socket_client.cpp: succeed to send data len: "<<iResult<<endl;
    }
    return ret;
}

bool CSocketClient::recvData(char* recvbuf, unsigned int len)
{
    if(!m_bConneted)
    {
        cout<<"socket_client.cpp: not connect yet!"<<endl;
        return false;
    }
    unsigned int iResult = 0; 
    bool ret = false;
    iResult = recv(m_socket_desc, recvbuf, len, 0);
    if(iResult <= 0)
        cout<<"socket_client.cpp: failed to recv data! iResult < 0 "<<endl;
    else
    { 
        ret = true;
        cout<<"socket_client.cpp: succeed to recv data len: "<<iResult<<endl;
    }
    return ret;
}

