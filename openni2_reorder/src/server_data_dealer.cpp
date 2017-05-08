#include "server_data_dealer.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <fstream>
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;

CServerDealer::CServerDealer(){}
CServerDealer::~CServerDealer(){}


bool CServerDealer::handleODO(char* buf, unsigned int len, char* path_)
{
    static const int odo_num = 3; 
    char pathODO[255]; 
    sprintf(pathODO, "%s/odo.txt", path_); 
    
    static ofstream odo_ouf(pathODO);
    if(!odo_ouf.is_open())
    {
      cout<<"server_data_dealer.cpp: failed to open file: "<<string(pathODO)<<endl;
      return false;
    }
    char* pbuf = buf; 
    double timestamp; 
    unsigned int step = sizeof(double);
    memcpy(&timestamp, pbuf, sizeof(double));
    odo_ouf<<std::fixed<<timestamp<<" "; 
    pbuf = pbuf + step;
    double v;
    for(int i=0; i<odo_num; i++)
    {
        memcpy(&v, pbuf, step);
        odo_ouf<<std::fixed<<v<<" ";
        pbuf += step;
    }
    odo_ouf<<endl;
    return true; 
}

bool CServerDealer::handleSICK(char* buf, unsigned int len, char* path_)
{
    static const int sick_num = 541; 
    char pathSICK[255];
    sprintf(pathSICK, "%s/laser.txt", path_);
    
    static ofstream sick_ouf(pathSICK);
    if(!sick_ouf.is_open())
    {
      cout<<"server_data_dealer.cpp: failed to open file: "<<string(pathSICK)<<endl;
      return false;
    }
    char* pbuf = buf; 
    double timestamp; 
    unsigned int step = sizeof(double);
    memcpy(&timestamp, pbuf, sizeof(double));
    sick_ouf<<std::fixed<<timestamp<<" "; 
    pbuf = pbuf + step;
    double v;
    for(int i=0; i<sick_num; i++)
    {
        memcpy(&v, pbuf, step);
        sick_ouf<<std::fixed<<v<<" ";
        pbuf += step;
    }
    sick_ouf<<endl;
    return true;   
}

bool CServerDealer::handleXtion(char* buf, unsigned int len, char* path_)
{
    static const int width = 640;
    static const int height = 480;
    static const int n_pixel = width*height;
    static const int header_size = 8; // timestamp
    static const int depth_size = 2 * n_pixel;
    static const int rgb_size = 3 * n_pixel;
    static const int total_size = header_size + depth_size + rgb_size;
    
    assert(len == total_size);

    cv::Mat rgb(480, 640, CV_8UC3);
    cv::Mat depth(480, 640, CV_16UC1);

    char assoc_path[255]= {0}; 
    char png_name[255] = {0};
    char pathDepth[255]= {0};
    char pathRGB[255]={0}; 
    double t; 
    char* pbuf = buf; 
    // obtain timestamp
    memcpy(&t, pbuf, sizeof(double)); 
    sprintf(png_name, "%lf.png", t);

    // record into assoc.txt (timestamp, png)
    sprintf(assoc_path, "%s/assoc.txt", path_);
    static ofstream outf_rgbd(assoc_path);
    outf_rgbd<<std::fixed<<std::setprecision(6)<<t<<"\t"<<png_name<<endl;
    
    // write to depth file 
    sprintf(pathDepth, "%s/depth/%s", path_, png_name);
    memcpy(depth.data, pbuf+sizeof(double), depth_size); 
    if(!cv::imwrite(pathDepth, depth))
    {
        cout<<"server_data_dealer.cpp: failed to write file: "<<pathDepth<<endl;
        return false;
    }

    // write to rgb file 
    sprintf(pathRGB, "%s/rgb/%s", path_, png_name);
    memcpy(rgb.data, pbuf+sizeof(double)+depth_size, rgb_size);
    if(!cv::imwrite(pathRGB, rgb))
    {
        cout<<"server_data_dealer.cpp: failed to write file: "<<pathRGB<<endl;
        return false;
    }
    return true;
}
