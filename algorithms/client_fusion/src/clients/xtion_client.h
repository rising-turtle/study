#ifndef XTION_CLIENT_H
#define XTION_CLIENT_H

#include "socket_client.h"
// #include "mopenni2.h"
#include <string>
#include "opencv2/core/core.hpp"
// #include <opencv/cv.h>

using namespace std;

class MOpenni2;

class CXtionClient : public CSocketClient
{
	friend class CXSickOdoClient;
public:
	CXtionClient(string path_ = "D:\\client_fusion\\data");
    virtual ~CXtionClient();

    void stopSendDataOffline();    // send local disk data to server 
    bool startSendDataOffline();

    bool startRecordData();     // record Xtion data to local disk
    void stopRecordData();
protected:
	bool startRecordXtion();
    bool recordXtion2File(); // record current data in m_rgb m_dpt into disk
    bool sendXtion2Server(char* buf, int len); // send data to server
	
    bool m_bXtionReady;
    string m_path_xtion;
    MOpenni2* m_pOpenni;
    cv::Mat m_rgb; 
    cv::Mat m_dpt;
    double m_timestamp;
};


#endif
