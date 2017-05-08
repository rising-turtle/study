#ifndef SOCKET_SERVER_H
#define SOCKET_SERVER_H

#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include "server_data_dealer.h"

class CSocketServer
{
public:
    CSocketServer();
    virtual ~CSocketServer();
    bool listenTo(int hport);
    bool recvData(char* recvbuf, int len);
    bool sendData(char* sendbuf, int len);
    void recvXtionData();
    void stopRecvXtion();
    void recvSICKData();
    void recvODOData();
private:
    bool m_bConneted;
    mutable bool m_bStopRecvXtion;
    int m_listen_socket_desc; 
    int m_accept_socket_desc;
    int m_recvNum;
    CServerDealer m_handler;
public:
    static void* thread_start(void* param); 
};

#endif
