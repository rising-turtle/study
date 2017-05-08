#include <iostream>
#include <string.h>
#include "socket_server.h"
#include <pthread.h>
#include <sys/time.h>
using namespace std; 

void test_socket_server();
void test_xtion_server();

int main(int argc, char * argv[])
{
    // test_socket_server();
    test_xtion_server();
    return 0; 
}

void test_xtion_server()
{
    CSocketServer server; 
    if(!server.listenTo(6060))
        return ;
    char sbuf[4096];
    char rbuf[4096];
    memset(sbuf, 0, sizeof(sbuf));
    memset(rbuf, 0, sizeof(rbuf));
    int cmd; 
    double t = 1.31242e5;
    int msg_len = sizeof(int) + sizeof(double);
    
    pthread_t xtion_thread;
    if(pthread_create(&xtion_thread, NULL, &CSocketServer::thread_start, (void*)(&server)) !=0)
    {
        cout<<"failed to create thread for start recving xtion!"<<endl;
        return ;
    }
    while(1)
    {
        cout<<"please input cmd for next cmd!"<<endl;
        cin>>cmd;
        cout<<"current cmd: "<<cmd<<endl;
        memcpy(sbuf, &cmd, sizeof(int));
        if(cmd != 0)
        {
            server.sendData(sbuf, sizeof(int));
        }else
        {
            struct timeval t;
            gettimeofday(&t, 0); 
            memcpy(sbuf + 4, &t.tv_sec, sizeof(long)); 
            memcpy(sbuf + 4 + sizeof(long), &t.tv_usec, sizeof(long));
            cout<<"asy time server: "<<t.tv_sec<<"."<<t.tv_usec<<endl;
            server.sendData(sbuf, sizeof(int) + 2*sizeof(long));
        }
        if(!(cmd >=0 && cmd <=5))
        {
            cout<<"unknown cmd: "<<cmd<<" quit!"<<endl;
            break;
        }
        usleep(10000);
    }
    // 1 send cmd 1 
   /* cmd = 1;
    memcpy(sbuf, &cmd, sizeof(int)); 
    memcpy(sbuf+4, &t, sizeof(double));
    cout<<"send cmd: "<<cmd<<endl;
    server.sendData(sbuf, sizeof(int) + sizeof(double)); 
    usleep(100);
    cout<<"waiting for recv xtion thread!"<<endl;
    pthread_join(xtion_thread, NULL);
    cout<<"ok, after waiting!"<<endl;
    // 2 send cmd 10
    cmd = 10; 
    memcpy(sbuf, &cmd, sizeof(int));
    cout<<"send cmd: "<<cmd<<" to close connection!"<<endl;
    server.sendData(sbuf, 12);
    usleep(100);*/
}

void test_socket_server()
{
    CSocketServer server; 
    if(!server.listenTo(6060))
        return ;
    char sbuf[4096];
    char rbuf[4096];
    memset(sbuf, 0, sizeof(sbuf));
    memset(rbuf, 0, sizeof(rbuf));
    int cmd; 
    double t = 1.31242e5;
    int msg_len = sizeof(int) + sizeof(double);
    // 1 send cmd 0 
    cmd = 0;
    memcpy(sbuf, &cmd, sizeof(int)); 
    memcpy(sbuf+4, &t, sizeof(double));
    cout<<"send cmd: "<<cmd<<endl;
    server.sendData(sbuf, sizeof(int) + sizeof(double)); 
    usleep(5000);
    // 2 send cmd 1
    cmd = 1; 
    memcpy(sbuf, &cmd, sizeof(int));
    cout<<"send cmd: "<<cmd<<endl;
    server.sendData(sbuf, msg_len);
    server.recvData(rbuf, sizeof(rbuf));
    cout<<"recv data: "<<rbuf<<endl;
    // 3 send cmd 3
    cmd = 3; 
    memcpy(sbuf, &cmd, sizeof(int));
    cout<<"send cmd: "<<cmd<<endl;
    server.sendData(sbuf, msg_len);
    // 4 send cmd 10 
    cmd = 10; 
    memcpy(sbuf, &cmd, sizeof(int)); 
    cout<<"send cmd: "<<cmd<<" to close this connection!"; 
    server.sendData(sbuf, msg_len);
    while(server.recvData(rbuf, sizeof(rbuf)))
    {
        cout<<"waiting to close connection!"<<endl; 
    }
    cout<<"connection has been closed!"<<endl;
}
