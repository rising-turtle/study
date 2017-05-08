#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include <sys/socket.h>
#include <netinet/in.h>

// 1. Create a socket
// 2. Connect to remote server
// 3. Send some data
// 4. Receive a reply

class CSocketClient
{
public:
    CSocketClient();
    virtual ~CSocketClient();
    void connect(char* hip, int hport);
    bool recvData(char* recvbuf, unsigned int len);
    bool sendData(char* sendbuf, unsigned int len);
    bool sendXtion();
protected:
    int m_socket_desc;
    bool m_bConneted;
};


#endif
