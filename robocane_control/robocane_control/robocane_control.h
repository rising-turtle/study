/*
 * Oct. 12, 2016 David Z 
 *
 *  An interface to control the robocane, through maxon motor and clucth
 *
 * */


#ifndef ROBOTCANE_H
#define ROBOTCANE_H

#include "Definitions.h"
#include <iostream>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <vector> 
#include <math.h>
#include <pthread.h>
#include "mraa.h"
#include <ros/ros.h>

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif

typedef enum {VelMode, PosMode=2, VoidMode=-1} MOTION_MODE; 

const static int FOLLOWING_ERROR = 0x8611; // difference between designed pos and actual pos is larger than maximum Following ERROR value 

class CRoboCane
{
  public:
    CRoboCane();
    virtual ~CRoboCane(); 

  public:
    // init
    bool init(); 
    bool uninit(); 
    void printSettings(); 
    void defaultParameters(); 

    bool initMaxonMotor(); // open device and set parameters
    bool initClutch();     // open clutch

    // open & close 
    int openMaxonMotor(unsigned int* );  // open motor
    int closeMaxonMotor(unsigned int* ); // close motor

    // enable & disable 
    int enableMaxonMotor(); 
    int disableMaxonMotor(); 

    // set mode 
    int setPosMode(); 
    int setVelMode(); 

    // halt
    int haltPosMove();            
    int haltVelMove();    

    // swing motion models, first right and then left
    bool swingPositionMode(long pos, int loops, int ts = 200); 
    bool swingVelocityMode(long vel, int loops, int ts = 200); 

    // move with a bunch of cmds
    int moveVelocityList(vector<long>& vList, vector<int>& tsList);
    int movePositionList(vector<long>& pList, vector<int>& tsList); 

    // move 
    int movePosition(long pos, int ts = 200); 
    int moveVelocity(long vel, int ts = 200); 

    // error 
    void showDeviceError(); 
    unsigned int getDeviceErrorCode();
    // void clearError(); 

  public:
    
    // methods and parameters to dump encoder measurements
    ros::Publisher mr_encoder_time;
    ros::Publisher mr_encoder_pub; 
    static void* thread_dump_encoder(void* context);
    void* dumpEncoder(); 
    bool startDumpEncoder(string f, int hz);
    bool endDumpEncoder(); 
    volatile bool mb_record_encoder; 
    string m_encoder_output_file;
    int m_encoder_hz;       
    pthread_t mh_thread_dump_encoder;

    // maxon parameters 
    void* mp_KeyHandle; 
    unsigned short m_usNodeId; 
    string m_deviceName; 
    string m_protocolStackName; 
    string m_interfaceName; 
    string m_portName; 
    int m_baudrate; 
    unsigned long m_profileVelocity; 
    unsigned long m_profileAcceleration; 
    unsigned long m_profileDeceleration; 
    unsigned int m_maxFollowingError;
    MOTION_MODE m_mode; 

    // clutch parameters
    void clucthOn(); 
    void clucthOff(); 
    mraa_gpio_context m_gpio; 
    int m_gpio_id;      // the gpio pin number 
    // int m_level ;       // 1: 3v3 output; 0: 0v output 
    bool m_clutchOn;    // the clutch is on or not 
    
};


#endif




