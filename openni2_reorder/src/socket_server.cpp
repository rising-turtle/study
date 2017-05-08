#include "socket_server.h"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string.h>

// #include "opencv/cv.h"
// #include "opencv/highgui.h"
// #include "opencv/cvwimage.h"

using namespace std;
void* CSocketServer::thread_start(void* param)
{
    CSocketServer* pSeaver = static_cast<CSocketServer*>(param);
      pSeaver->recvXtionData();
   // pSeaver->recvSICKData();
   // pSeaver->recvODOData();
}

CSocketServer::CSocketServer() : 
m_bConneted(false),
m_bStopRecvXtion(false),
m_recvNum(-1)
{}
CSocketServer::~CSocketServer(){}

bool CSocketServer::listenTo(int hport)
{
    if(m_bConneted)
    {
        cout<<"socket_server.cpp: has listened, listen another ? "<<endl;
        return false;
    }
    // Create socket
    m_listen_socket_desc = socket(AF_INET, SOCK_STREAM, 0);
    if(m_listen_socket_desc == -1)
    {
        cout<<"socket_server.cpp: failed to create listen socket!"<<endl;
        return false;
    }
    
    // Prepare sockaddr_in structure
    sockaddr_in server, client; 
    server.sin_family = AF_INET; 
    server.sin_addr.s_addr = INADDR_ANY; 
    server.sin_port = htons(hport); 

    // Bind 
    if(bind(m_listen_socket_desc, (sockaddr*)&server, sizeof(server)) < 0)
    {
        cout<<"socket_server.cpp: failed to bind hport: "<<hport<<endl;
        return false;
    }

    // Listen 
    listen(m_listen_socket_desc, 10); 

    cout<<"socket_server.cpp: succeed to listen, waiting for the incomming info..." <<endl; 
    
    int len_addr =  sizeof(sockaddr_in); 
    m_accept_socket_desc = accept(m_listen_socket_desc, (sockaddr*)&client, (socklen_t*)&len_addr);
    if(m_accept_socket_desc < 0)
    {
        cout<<"socket_server.cpp: failed to accept"<<endl;
        return false;
    }else{
        cout<<"socket_server.cpp: succeed to accept connection!"<<endl;
        m_bConneted = true; 
    }
    return true;
}
void CSocketServer::recvODOData()
{
    if(!m_bConneted)
    {   
        cout<<"socket_server.cpp: not connect yet!"<<endl;
        return ;
    }
    static const int total_size = (3+1)*8; // 541 points + timestamp + tag 
    char *path = "../data_s";
    char* rbuf = new char [total_size];
    char* cbuf = new char [total_size];
    m_bStopRecvXtion = false;
    int ncout = 0;
    double t;

    int received_data = 0; 
    int curr_left_data = 0;
    while(!m_bStopRecvXtion && (m_recvNum < 0 || ncout < m_recvNum))
    {
        int iResult = recv(m_accept_socket_desc, cbuf, total_size, 0);
        if(iResult <=0 )
        {
            cout<<"socket_server.cpp: receive iResult = "<<iResult<<" something wrong!"<<endl;
            sleep(2);
            break;
        }
        // cout<<"socket_server.cpp receive data size: "<<iResult<<endl;
        curr_left_data = iResult;
        if(iResult > total_size)
        {
            cout<<"socket_server.cpp: receive iResult = "<<iResult<<" > total_size: "<<total_size<<endl;
        }else{
            cout<<"socket_server.cpp: receive iResult = "<<iResult<<endl;
        }
        if(iResult + received_data > total_size)
        {
            curr_left_data = total_size - received_data;
        }
        memcpy(rbuf + received_data, cbuf, curr_left_data);
        received_data += curr_left_data; 
        if(received_data == total_size)
        {
            if(!m_handler.handleODO(rbuf, total_size, path))
            {
                cout<<"socket_server.cpp: failed to handleODO data!"<<endl;
                break;
            }
            ncout++;
            memcpy(rbuf, cbuf + curr_left_data, iResult - curr_left_data);
            received_data = iResult - curr_left_data ;
        }else{
            continue;
         }
    }
    delete []rbuf;
    delete []cbuf;
 
}
// actually sick + odo
void CSocketServer::recvSICKData()
{
    if(!m_bConneted)
    {   
        cout<<"socket_server.cpp: not connect yet!"<<endl;
        return ;
    }
    static const int total_size = (541+1)*8; // 541 points + timestamp + tag 
    char *path = "../data_s";
    char* rbuf = new char [total_size];
    char* cbuf = new char [total_size];
    m_bStopRecvXtion = false;
    int ncout = 0;
    double t;

    int received_data = 0; 
    int curr_left_data = 0;
    while(!m_bStopRecvXtion && (m_recvNum < 0 || ncout < m_recvNum))
    {
        int iResult = recv(m_accept_socket_desc, cbuf, total_size, 0);
        if(iResult <=0 )
        {
            cout<<"socket_server.cpp: receive iResult = "<<iResult<<" something wrong!"<<endl;
            sleep(2);
            break;
        }
        // cout<<"socket_server.cpp receive data size: "<<iResult<<endl;
        curr_left_data = iResult;

        if(iResult + received_data > total_size)
        {
            curr_left_data = total_size - received_data;
        }
        memcpy(rbuf + received_data, cbuf, curr_left_data);
        received_data += curr_left_data; 
        if(received_data == total_size)
        {
            if(!m_handler.handleSICK(rbuf, total_size, path))
            {
                cout<<"socket_server.cpp: failed to handleSICK data!"<<endl;
                break;
            }
            ncout++;
            memcpy(rbuf, cbuf + curr_left_data, iResult - curr_left_data);
            received_data = iResult - curr_left_data ;
        }else{
            continue;
         }
    }
    delete []rbuf;
    delete []cbuf;
 
}

void CSocketServer::recvXtionData()
{
    if(!m_bConneted)
    {   
        cout<<"socket_server.cpp: not connect yet!"<<endl;
        return ;
    }
    static const int total_size = 8 + 640*480*5; 
    char *path = "../data_s";
    char* rbuf = new char [total_size];
    char* cbuf = new char [total_size];
    m_bStopRecvXtion = false;
    int ncout = 0;
    double t;

    int received_data = 0; 
    int curr_left_data = 0;
    while(!m_bStopRecvXtion && (m_recvNum < 0 || ncout < m_recvNum))
    {
        int iResult = recv(m_accept_socket_desc, cbuf, total_size, 0);
        if(iResult <=0 )
        {
            cout<<"socket_server.cpp: receive iResult = "<<iResult<<" something wrong!"<<endl;
            sleep(2);
            break;
        }
        // cout<<"socket_server.cpp receive data size: "<<iResult<<endl;
        curr_left_data = iResult;
        if(iResult + received_data > total_size)
        {
            curr_left_data = total_size - received_data;
        }
        memcpy(rbuf + received_data, cbuf, curr_left_data);
        received_data += curr_left_data; 
        if(received_data == total_size)
        {
            if(!m_handler.handleXtion(rbuf, total_size, path))
            {
                cout<<"socket_server.cpp: failed to handleXtion data!"<<endl;
                break;
            }
            ncout++;
            memcpy(rbuf, cbuf + curr_left_data, iResult - curr_left_data);
            received_data = iResult - curr_left_data ;
        }else{
            continue;
         }
    }
    delete []rbuf;
    delete []cbuf;
}

/*
void CSocketServer::recvXtionData()
{
    if(!m_bConneted)
    {   
        cout<<"socket_server.cpp: not connect yet!"<<endl;
        return ;
    }
    static const int width = 640; 
    static const int height = 480; 
    static const int n_pixel = width*height;
    static const int header_size = 8; // timestamp
    static const int depth_size = 2 * n_pixel;
    static const int rgb_size = 3 * n_pixel;
    static const int total_size = header_size + depth_size + rgb_size;

    char* rbuf = new char[total_size];
    char* cbuf = new char[total_size];
    char savePathDep[255];
    char savePathRGB[255];

    m_bStopRecvXtion = false;
    int ncout = 0;
    double t;
    cv::Mat rgb(480, 640, CV_8UC3);
    cv::Mat depth(480, 640, CV_16UC1);
    ofstream out_rgbd_t("../data/s_rgbd_t.txt");
    
    int received_data = 0; 
    int curr_left_data = 0;
    while(!m_bStopRecvXtion && (m_recvNum < 0 || ncout < m_recvNum))
    {
        int iResult = recv(m_accept_socket_desc, cbuf, total_size, 0);
        if(iResult <=0 )
        {
            cout<<"socket_server.cpp: receive iResult = "<<iResult<<" something wrong!"<<endl;
            sleep(2);
            break;
        }
        // cout<<"socket_server.cpp receive data size: "<<iResult<<endl;
        curr_left_data = iResult;
        if(iResult + received_data > total_size)
        {
            curr_left_data = total_size - received_data;
        }
        memcpy(rbuf + received_data, cbuf, curr_left_data);
        received_data += curr_left_data; 
        if(received_data == total_size)
        {
            memcpy(&t, rbuf, header_size);
            out_rgbd_t<<std::fixed<<t<<endl;
            // write received data 
            // file name 
            sprintf(savePathDep,"../data/depth_s/%lf.png", t);
            sprintf(savePathRGB,"../data/rgb_s/%lf.png", t);
            // copy data 
            memcpy(depth.data, rbuf+header_size, depth_size);
            memcpy(rgb.data, rbuf+header_size+depth_size, rgb_size);
            // write data
            imwrite(savePathDep, depth);
            imwrite(savePathRGB, rgb);
            ncout++;
            memcpy(rbuf, cbuf + curr_left_data, iResult - curr_left_data);
            received_data = iResult - curr_left_data ;
        }else{
            continue;
         }
    }
    // send stop command to client 
    char sbuf[128];
    int cmd = 2; 
    memcpy(sbuf, &cmd, 4);
    send(m_accept_socket_desc, sbuf, 12, 0);
    delete []rbuf;
    delete []cbuf;
}
*/
bool CSocketServer::recvData(char* recvbuf, int len)
{
    if(!m_bConneted)
    {   
        cout<<"socket_server.cpp: not connect yet!"<<endl;
        return false;
    }   
    unsigned int iResult = 0;  
    bool ret = false;
    iResult = recv(m_accept_socket_desc, recvbuf, len, 0); 
    if(iResult <= 0)
        cout<<"socket_server.cpp: failed to recv data! iResult < 0 "<<endl;
    else
    {   
        ret = true;
        cout<<"socket_server.cpp: succeed to recv data len: "<<iResult<<endl;
    }   
    return ret;
}

bool CSocketServer::sendData(char* sendbuf, int len)
{
    bool ret = false;
    if(!m_bConneted)
    {
        cout<<"socket_server.cpp: not connect yet!"<<endl;
        return false;
    }
    unsigned int iResult = 0;
    iResult = send(m_accept_socket_desc, sendbuf, len, 0);
    if(iResult <= 0)
        cout<<"socket_server.cpp: failed to send data! iResult < 0 "<<endl;
    else if(iResult != len)
        cout<<"socket_server.cpp: failed to send complete data! iResult < len "<<endl;
    else
    {
        ret = true;
        cout<<"socket_server.cpp: succeed to send data len: "<<iResult<<endl;
    }
    return ret;
}

void CSocketServer::stopRecvXtion()
{
    m_bStopRecvXtion = true;
}
