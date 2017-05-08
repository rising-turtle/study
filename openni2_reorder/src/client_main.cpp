#include <iostream>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include "socket_client.h"
#include "xtion_client.h"
#include <sys/time.h>

using namespace std;

void test_socket_client();
void test_xtion_client();

int main(int argc, char* argv[])
{
    // test_socket_client();
    test_xtion_client();
    return 0; 
}

void test_xtion_client()
{
    CXtionClient client; 
    client.connect("127.0.0.1", 6060);
    // client.connect("192.168.0.4", 9012);
    // client.connect("192.168.0.4", 6060);

    char rbuf[4096];
    memset(rbuf, 0, sizeof(rbuf));
    int cmd  = -1; 
    bool m_close = false;
    pthread_t xtion_thread_record;
    pthread_t xtion_thread_send;
    while(client.recvData(rbuf, sizeof(rbuf)))
    {
        memcpy(&cmd, rbuf, sizeof(4));
        switch(cmd)
        {
            case 0: 
                long sec, usec; 
                memcpy(&sec, rbuf+4, sizeof(long));
                memcpy(&usec, rbuf + 4 + sizeof(long), sizeof(long));
                struct timeval t;
                t.tv_sec = sec;
                t.tv_usec = usec; 
                settimeofday(&t, 0); 
                cout<<"cmd = 0, OK, let's asy our time : !"<<t.tv_sec<<"."<<t.tv_usec<<endl;
                break;
            case 1:
                cout<<"cmd = 1, OK, let's record xtion data to local disk!"<<endl;
                if(pthread_create(&xtion_thread_record, NULL, &CXtionClient::thread_start_record2local, (void*)(&client)) != 0)
                {
                    cout<<"failed to create xtion_thread to record data locally!"<<endl;
                }else
                {
                    cout<<"succeed to create thread to record data locally!"<<endl;
                }
                usleep(100000);
                break; 
            case 2:
                cout<<"cmd = 2, OK, let's stop send xtion data!"<<endl;
                client.stopRecordXtion();
                // client.stopSendXtion();
                break; 
            case 3:
                cout<<"cmd = 3, OK, let's clear all the data in the DIR!"<<endl;
                
                break; 
            case 4:
                cout<<"cmd = 4, OK, let's send all the data to server offline!"<<endl;
                if(pthread_create(&xtion_thread_send, NULL, &CXtionClient::thread_start_send2server, (void*)(&client)) != 0)
                {
                    cout<<"failed to create xtion_thread to send data to server!"<<endl;
                }else
                {
                    cout<<"succeed to create thread to send data to server!"<<endl;
                }
                usleep(100000);
                break;
            default:
                cout<<"Oh, no, Unknown cmd: "<<cmd<<" let's close"<<endl;
                m_close = true;
                break;
        }
        if(m_close ) break;
        usleep(100);
    }
}

void test_socket_client()
{
    CSocketClient client; 
    client.connect("127.0.0.1", 6060);
    char rbuf[4096]; 
    char sbuf[4096];
    memset(sbuf, 0, sizeof(sbuf)); 
    memset(rbuf, 0, sizeof(rbuf));
    char* sendMsg = "Hello server, now I can only send you This!";
    memcpy(sbuf, sendMsg, strlen(sendMsg));
    int cmd = -1;
    bool bclose = false;
    double timestamp = 0;

    // receive data from server 
    while(client.recvData(rbuf, sizeof(rbuf)))
    {
        memcpy(&cmd, rbuf, 4);
        switch(cmd)
        {
            case 0: 
                memcpy(&timestamp, rbuf+4, sizeof(double));
                cout<<"cmd = 0: time asy! recv timestamp: "<< timestamp<<endl;
                break; 
            case 1:
                cout<<"cmd = 1: begin send data!"<<endl;
                client.sendData(sbuf, strlen(sbuf));
                break; 
            case 2:
                cout<<"cmd = 2: stop send data!"<<endl; 
                break; 
            case 3:
                cout<<"cmd = 3: send all data to server !"<<endl;
                break; 
            default:
                cout<<"unknown cmd = "<<cmd<<" now close connection!"<<endl;
                bclose = true;
                break;
        }
        if(bclose) break;
    }
}
