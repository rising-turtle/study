#include "robocane_control.h"
#include "mraa.h"
#include <sstream>
#include <fstream>
#include <assert.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Header.h>

namespace {

  void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
  {
    cerr  << " robocane_control.cpp : " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
  }

  void LogInfo(string message)
  {
    cout << message << endl;
  }


  void SeparatorLine()
  {
    const int lineLength = 60;
    for(int i=0; i<lineLength; i++)
    {
      cout << "-";
    }
    cout << endl;
  }

}


CRoboCane::CRoboCane()
{
  // init(); 
  defaultParameters(); 
}
CRoboCane::~CRoboCane()
{
  // uninit(); 
  if(m_mode != VoidMode)
     uninit(); 
  if(mh_thread_dump_encoder != 0)
  {
    endDumpEncoder();
    // pthread_join(mh_thread_dump_encoder, 2);
    sleep(1);
  }
}

void CRoboCane::defaultParameters()
{
  ros::NodeHandle n; 
  // maxon motor 
  m_usNodeId = 1; // USB
  m_deviceName = "EPOS2"; //EPOS
  m_protocolStackName = "MAXON SERIAL V2"; //MAXON_RS232
  m_interfaceName = "USB"; //RS232
  m_portName = "USB0"; // /dev/ttyS1
  m_baudrate = 1000000; //115200

  // clutch 
  m_gpio_id = 40; 
  m_clutchOn = false; 

  // encoder 
  mb_record_encoder = false; 
  m_encoder_output_file = ""; 
  m_encoder_hz = 100; // 100 hz = 10ms
  mh_thread_dump_encoder = 0;

  m_profileVelocity = 5000; 
  m_profileAcceleration = 10000; 
  m_profileDeceleration = 10000; 
  m_maxFollowingError = 2000;

  mr_encoder_time = n.advertise<std_msgs::Header>("/maxon_time", 25);
  mr_encoder_pub = n.advertise<std_msgs::Int32MultiArray>("/maxon_encoder", 25); 
}

bool CRoboCane::init()
{
  return initMaxonMotor() && initClutch(); 
}

bool CRoboCane::uninit()
{
  int lResult = MMC_SUCCESS; 
  unsigned int ulErrorCode;
  
  if(m_mode == PosMode)
  {
    haltPosMove(); 
  }
  if(m_mode == VelMode)
  {
    haltVelMove(); 
  }

  m_mode = VoidMode; 

  if(VCS_SetDisableState(mp_KeyHandle, m_usNodeId, &ulErrorCode) == 0)
  {
    LogError("VCS_SetDisableState", lResult, ulErrorCode); 
    lResult = MMC_FAILED; 
  }

  if(lResult == MMC_SUCCESS)
  {
    if((lResult = closeMaxonMotor(&ulErrorCode))!=MMC_SUCCESS)
    {
      LogError("closeMaxonMotor", lResult, ulErrorCode);
    }
  }

  clucthOff(); // disengage 

  return true; 
}

bool CRoboCane::initClutch()
{
  m_gpio = mraa_gpio_init(m_gpio_id); 
  if(m_gpio == NULL)
  {
    cerr << " initClutch: failed to init gpio at "<<m_gpio_id<<endl; 
    return false; 
  }

  mraa_gpio_dir(m_gpio, MRAA_GPIO_OUT_LOW); 
  
  if(mraa_gpio_use_mmaped(m_gpio, 1) != MRAA_SUCCESS)
  {
    cout << " initClutch: mmapped access to gpio "<<m_gpio_id << " not supported, falling back to normal model"<<endl;
  }

  clucthOn(); 

  return true; 
}

bool CRoboCane::startDumpEncoder(string f, int hz)
{ 
  if(mb_record_encoder == true) // already start 
  {
    cerr << __FILE__<<" already start dump Encoder, need start again? "<<endl;
    return false;
  }

  m_encoder_output_file = f; 
  m_encoder_hz = hz > 0 ? hz : m_encoder_hz; 
  if( pthread_create(&mh_thread_dump_encoder, 0, &CRoboCane::thread_dump_encoder, this) != 0)
  {
    cerr << __FILE__<<" failed to create thread for dump encoder "<<endl;
    return false; 
  }  

  cout << __FILE__ << " succeed to create thread for dump encoder "<<endl;
  usleep(100); 
  return true; 

}

void* CRoboCane::thread_dump_encoder(void* context)
{
  return ((CRoboCane*)context)->dumpEncoder(); 
}

void* CRoboCane::dumpEncoder()
{
  
  ofstream ouf(m_encoder_output_file.c_str()); 
  if(!ouf.is_open())
  {
    mb_record_encoder = false; 
    cerr <<__FILE__<<": failed to open file "<<m_encoder_output_file<<" to dump encoder"<<endl;
    return NULL;
  }
  
  mb_record_encoder = true; 
  int sleep_ms = (int)(1000/m_encoder_hz)  + 1; 
  
  int pos, vel, avgVel;
  short cur, avgCur; 

  unsigned int pErrorCode;
  bool bErrorOccured = false; 
  while(mb_record_encoder)
  {
    if(VCS_GetPositionIs(mp_KeyHandle, m_usNodeId, &pos, &pErrorCode) == 0)
    {
      cerr <<__FILE__<< " GetPositionIs failed "<<endl;
      bErrorOccured = true; 
      break; 
    }
    if(VCS_GetVelocityIs(mp_KeyHandle, m_usNodeId, &vel, &pErrorCode) == 0)
    {
      cerr <<__FILE__<< " GetVelocityIs failed "<<endl;
      bErrorOccured = true; 
      break;
    }
    if(VCS_GetVelocityIsAveraged(mp_KeyHandle, m_usNodeId, &avgVel, &pErrorCode) == 0)
    {
      cerr <<__FILE__<< " GetVelocityIsAverged failed "<<endl;
      bErrorOccured = true; 
      break;
    }

   if(VCS_GetCurrentIs(mp_KeyHandle, m_usNodeId, &cur, &pErrorCode) == 0)
    {
      cerr <<__FILE__<< " GetCurrentIs failed "<<endl;
      bErrorOccured = true; 
      break;
    }

   if(VCS_GetCurrentIsAveraged(mp_KeyHandle, m_usNodeId, &avgCur, &pErrorCode) == 0)
    {
      cerr <<__FILE__<< " GetCurrentIsAveraged failed "<<endl;
      bErrorOccured = true; 
      break;
    }
    
    {
      ros::Time maxon_encoder_time = ros::Time::now(); 
      // cout<<__FILE__<< ": get a valid data"<<endl;
      ouf << std::fixed<< maxon_encoder_time.toSec()<<"\t"<< pos << "\t"<< vel<<"\t"<<avgVel<<"\t"<<cur<<"\t"<<avgCur<<endl; 

      std_msgs::Header maxon_header; 
      maxon_header.stamp = maxon_encoder_time;

      std_msgs::Int32MultiArray msg; 
      msg.data.resize(5); 
      msg.data[0] = pos; msg.data[1] = vel; msg.data[2] = avgVel;
      msg.data[3] = cur; msg.data[4] = avgCur; 

      mr_encoder_time.publish(maxon_header); 
      mr_encoder_pub.publish(msg); 
      ros::spinOnce();
    }
    usleep(1000*sleep_ms); 
  }
  
  if(bErrorOccured )
  {
    LogError("SOME_GetMotion", 1, pErrorCode); 
    showDeviceError();
  }

  ouf.close(); 
  cout << __FILE__<<" quit thread dump encoder "<<endl;
  return NULL;
}

unsigned int CRoboCane::getDeviceErrorCode()
{
  unsigned int pDeviceErrorCode = 0;
  unsigned int pErrorCode = 0;
  unsigned char nbOfDeviceError;  
  if(VCS_GetNbOfDeviceError(mp_KeyHandle, m_usNodeId, &nbOfDeviceError, &pErrorCode))
  {
    if(nbOfDeviceError <= 0)
    {
      return 0; 
    }
    if(nbOfDeviceError > 1) 
    {
      cout <<" There are "<<(int)(nbOfDeviceError) <<" device errors, return the first one"<<endl;
    }
    {
      if(!VCS_GetDeviceErrorCode(mp_KeyHandle, m_usNodeId, 1, &pDeviceErrorCode, &pErrorCode))
      {
        cerr<<__FILE__<<" failed to getDeviceError "<<endl;
      }else
      {
        cerr<<__FILE__<<" DeviceError=0x"<<std::hex<<pDeviceErrorCode<<" errorCode=0x"<<std::hex<<pErrorCode<<endl;
      }
    }
  }
  return pDeviceErrorCode;
}

void CRoboCane::showDeviceError()
{
  unsigned int pDeviceErrorCode = 0;
  unsigned int pErrorCode = 0;
  unsigned char nbOfDeviceError;  
  if(VCS_GetNbOfDeviceError(mp_KeyHandle, m_usNodeId, &nbOfDeviceError, &pErrorCode))
  {
    if(nbOfDeviceError <= 0)
    {
      cout<<"In showDeviceError() no error is detected !"<<endl;
    }
    for(unsigned char errorNum = 1; errorNum <= nbOfDeviceError; errorNum++)
    {
      if(!VCS_GetDeviceErrorCode(mp_KeyHandle, m_usNodeId, errorNum, &pDeviceErrorCode, &pErrorCode))
      {
        cerr<<__FILE__<<" failed to getDeviceError "<<endl;
        break;
      }else
      {
        cerr<<__FILE__<<" get error "<<(int)errorNum<<" DeviceError=0x"<<std::hex<<pDeviceErrorCode<<" errorCode=0x"<<std::hex<<pErrorCode<<endl;
      }
    }
  }else{
    cerr<<__FILE__<<" failed to getDeviceError"<<endl;
  }
}

bool CRoboCane::endDumpEncoder()
{
  mb_record_encoder = false; 
}


void CRoboCane::clucthOn()
{
  if(m_clutchOn == true) return ; // already on
  mraa_gpio_write(m_gpio, 1); // output 3v3 
  usleep(1000*100); // sleep 100 ms
  m_clutchOn = true; 
}

void CRoboCane::clucthOff()
{
  if(m_clutchOn == false) return; // already off
  mraa_gpio_write(m_gpio, 0); // output 0v 
  usleep(1000*100); // sleep 100 ms
  m_clutchOn = false; 
}

bool CRoboCane::initMaxonMotor()
{
  unsigned int p_pErrorCode; 
  if(openMaxonMotor(&p_pErrorCode) == MMC_FAILED)
  {
    cerr<<" failed to open maxon motor " << endl; 
    return false; 
  }
  if(enableMaxonMotor() == MMC_FAILED)
  {
    cerr << " failed to enable maxon motor" <<endl;
    return false; 
  }
  
  printSettings(); 

  return true; 
}

bool CRoboCane::swingVelocityMode(long vel, int loops, int ts)
{
  if(loops <= 0)
  {
    cout<<" swingVelocityMode has loops "<<loops<<", do you want to swing infinite rounds? Right now, I just set it as 10 loops"<<endl; 
    loops = 10; 
  }
  /*
  for(int i = 0; i<loops ; i++)
  {
    // swing to the right 
    if(moveVelocity(vel, ts) == MMC_FAILED)
    {
      cerr << " swingVelocityMode at "<<i<<" round failsed, what's going on? "<<endl;
      return MMC_FAILED; 
    }
    
    // swing to the left 
    if(moveVelocity(vel*-1, ts) == MMC_FAILED)
    {
       cerr << " swingVelocityMode at "<<i<<" round failsed, what's going on? "<<endl;
       return MMC_FAILED; 
    }
  }*/

  vector<long> vList(loops*2);
  vector<int> tList(loops*2); 
  for(int i=0; i<loops; i++)
  {
    vList[i*2] = (vel); 
    tList[i*2] = (ts); 
    vList[i*2+1] = (-vel); 
    tList[i*2+1] = ts;
  }
  
  moveVelocityList(vList, tList); 

  return MMC_SUCCESS; 
}

bool CRoboCane::swingPositionMode(long pos, int loops, int ts)
{
  if(loops <= 0)
  {
    cout<<" swingPositionMode has loops "<<loops<<", do you want to swing infinite rounds? Right now, I just set it as 10 loops"<<endl; 
    loops = 10; 
  }/*
  for(int i = 0; i<loops ; i++)
  {
    // swing to the right 
    if(movePosition(pos) == MMC_FAILED)
    {
      cerr << " swingPositionMode at "<<i<<" round failsed, what's going on? "<<endl;
      return MMC_FAILED; 
    }
    
    // swing to the left 
    if(movePosition(pos*-1) == MMC_FAILED)
    {
       cerr << " swingPositionMode at "<<i<<" round failsed, what's going on? "<<endl;
       return MMC_FAILED; 
    }
  }*/
  vector<long> pList(loops*2);
  vector<int> tList(loops*2); 
  for(int i=0; i<loops; i++)
  {
    if(i==0)
      pList[i*2] = (pos/2);
    else
      pList[i*2] = (pos); 
    tList[i*2] = (ts); 
    if(i==loops-1)
      pList[i*2+1] = (-pos/2);
    else
      pList[i*2+1] = (-pos); 
    tList[i*2+1] = ts;
  }
  
  movePositionList(pList, tList); 

  return MMC_SUCCESS; 
}

int CRoboCane::openMaxonMotor(unsigned int* p_pErrorCode)
{
  int lResult = MMC_FAILED;

  char* pDeviceName = new char[255];
  char* pProtocolStackName = new char[255];
  char* pInterfaceName = new char[255];
  char* pPortName = new char[255];

  strcpy(pDeviceName, m_deviceName.c_str());
  strcpy(pProtocolStackName, m_protocolStackName.c_str());
  strcpy(pInterfaceName, m_interfaceName.c_str());
  strcpy(pPortName, m_portName.c_str());

  LogInfo("Open device...");

  mp_KeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

  if(mp_KeyHandle!=0 && *p_pErrorCode == 0)
  {
    unsigned int lBaudrate = 0;
    unsigned int lTimeout = 0;

    if(VCS_GetProtocolStackSettings(mp_KeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
    {
      if(VCS_SetProtocolStackSettings(mp_KeyHandle, m_baudrate, lTimeout, p_pErrorCode)!=0)
      {
        if(VCS_GetProtocolStackSettings(mp_KeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
        {
          if(m_baudrate==(int)lBaudrate)
          {
            lResult = MMC_SUCCESS;
          }
        }
      }
    }
  }
  else
  {
    mp_KeyHandle = 0;
  }

  delete []pDeviceName;
  delete []pProtocolStackName;
  delete []pInterfaceName;
  delete []pPortName;

  return lResult;
}

int CRoboCane::closeMaxonMotor(unsigned int* p_pErrorCode)
{
  int lResult = MMC_FAILED;

  *p_pErrorCode = 0;

  LogInfo("Close device");

  if(VCS_CloseDevice(mp_KeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
  {
    lResult = MMC_SUCCESS;
  }

  m_mode = VoidMode; 

  return lResult;
}

int CRoboCane::enableMaxonMotor()
{
  int lResult = MMC_SUCCESS;
  BOOL oIsFault = 0;
  unsigned int p_pErrorCode; 

  if(VCS_GetFaultState(mp_KeyHandle, m_usNodeId, &oIsFault, &p_pErrorCode ) == 0)
  {
    LogError("VCS_GetFaultState", lResult, p_pErrorCode);
    lResult = MMC_FAILED;
  }

  if(lResult==MMC_SUCCESS)
  {
    if(oIsFault)
    {
      stringstream msg;
      msg << "clear fault, node = '" << m_usNodeId << "'";
      LogInfo(msg.str());

      if(VCS_ClearFault(mp_KeyHandle, m_usNodeId, &p_pErrorCode) == 0)
      {
        LogError("VCS_ClearFault", lResult, p_pErrorCode);
        lResult = MMC_FAILED;
      }
    }

    if(lResult==MMC_SUCCESS)
    {
      BOOL oIsEnabled = 0;

      if(VCS_GetEnableState(mp_KeyHandle, m_usNodeId, &oIsEnabled, &p_pErrorCode) == 0)
      {
        LogError("VCS_GetEnableState", lResult, p_pErrorCode);
        lResult = MMC_FAILED;
      }

      if(lResult==MMC_SUCCESS)
      {
        if(!oIsEnabled)
        {
          if(VCS_SetEnableState(mp_KeyHandle, m_usNodeId, &p_pErrorCode) == 0)
          {
            LogError("VCS_SetEnableState", lResult, p_pErrorCode);
            lResult = MMC_FAILED;
          }
        }
      }
    }
  }
  return lResult;
}

int CRoboCane::disableMaxonMotor()
{
  int lResult = MMC_SUCCESS; 
  unsigned int lErrorCode; 
  if(VCS_SetDisableState(mp_KeyHandle, m_usNodeId, &lErrorCode) == 0)
  {
    LogError("VCS_SetDisableState", lResult, lErrorCode);
    lResult = MMC_FAILED;
  }
  return lResult; 
}

int CRoboCane::movePositionList(vector<long>& pList, vector<int>& tsList)
{
  assert(pList.size() == tsList.size()); 
  int lResult = MMC_SUCCESS; 
  if(m_mode != PosMode)
  {
    cout << " movePosition not in MovMode, setPosMode !"<<endl; 
    lResult = setPosMode(); 
  }  

  vector<int>::iterator it_t = tsList.begin();
  vector<long>::iterator it_p = pList.begin(); 
  unsigned int p_rlErrorCode;
  int bTargetReached = 0;
  unsigned int p_deviceError = 0;
  unsigned int error_cnt = 0; 
  if(lResult == MMC_SUCCESS)
  {
    while(it_t != tsList.end())
    {
      long targetPos = (*it_p); 
      int targetTs = (*it_t); 

      if(VCS_MoveToPosition(mp_KeyHandle, m_usNodeId, targetPos, 0, 1, &p_rlErrorCode) == 0)
      {
        LogError("VCS_MoveToPosition", lResult, p_rlErrorCode); 
        lResult = MMC_FAILED; 
        return lResult; 
      }
      while(1)
      {
        int move_state = VCS_GetMovementState(mp_KeyHandle, m_usNodeId, &bTargetReached, &p_rlErrorCode); 
        int pPos;
        if(bTargetReached == 1)
        {
          /*
          if(move_state != 0)
          { 
            VCS_GetPositionIs(mp_KeyHandle, m_usNodeId, &pPos, &p_rlErrorCode); 
            cout <<" target reached current pos: "<<pPos << endl; 
          }else{
            LogError("VCS_GetMovementState", move_state, p_rlErrorCode);
          }
        */
          p_deviceError = getDeviceErrorCode(); 
          if(p_deviceError == 0){ 
            VCS_GetPositionIs(mp_KeyHandle, m_usNodeId, &pPos, &p_rlErrorCode); 
            cout <<" target reached current pos: "<<pPos << endl; 
            error_cnt = 0;
            break;
          }
          
          if(p_deviceError == FOLLOWING_ERROR)
          {
            cout <<" detect FOLLOWING_ERROR, try to recovery from it at "<<++error_cnt<<" trials"<<endl;
            enableMaxonMotor(); 
          }
          if(error_cnt > 10) 
          {
            cout << " failed 10 times, break "<<endl;
            break;
          }
        }
        // sleep(targetTs);
        usleep(1000*targetTs); 
      }
      showDeviceError();
      ++it_t; 
      ++it_p;
    }

    haltPosMove(); 
  }
  return lResult; 
}

int CRoboCane::movePosition(long pos, int ts)
{
  int lResult = MMC_SUCCESS; 
  if(m_mode != PosMode)
  {
    cout << " movePosition not in MovMode, setPosMode !"<<endl; 
    lResult = setPosMode(); 
  } 
  
  LogInfo("before movePosition"); 
  unsigned int p_rlErrorCode; 
  int bTargetReached = 0; 
  if(lResult == MMC_SUCCESS)
  {
    if(VCS_MoveToPosition(mp_KeyHandle, m_usNodeId, pos, 0, 1, &p_rlErrorCode) == 0)
    {
      LogError("VCS_MoveToPosition", lResult, p_rlErrorCode); 
      lResult = MMC_FAILED; 
      return lResult; 
    }
    
    int pPos; 
    // reach target 
    while(1)
    {
      VCS_GetMovementState(mp_KeyHandle, m_usNodeId, &bTargetReached, &p_rlErrorCode); 
      // if(bTargetReached == 0)
      {
        // VCS_GetPositionIs(mp_KeyHandle, m_usNodeId, &pPos, &p_rlErrorCode); 
        // cout <<" target not reach current pos: "<<pPos << endl; 
      }
      if(bTargetReached == 1){
        // VCS_GetPositionIs(mp_KeyHandle, m_usNodeId, &pPos, &p_rlErrorCode); 
        // cout <<" target reached current pos: "<<pPos << endl; 
        break; 
      }
     //  sleep(ts);
     usleep(1000*ts);
    }
  }
  
   haltPosMove(); 

  return lResult; 
}

int CRoboCane::moveVelocityList(vector<long>& vList, vector<int>& tsList)
{
  assert(vList.size() == tsList.size());
  int lResult = MMC_SUCCESS; 
  if(m_mode != VelMode)
  {
    cout << " moveVelocity not in VelMode, setVelMode !"<<endl; 
    lResult = setVelMode(); 
  } 
  vector<int>::iterator it_t = tsList.begin();
  vector<long>::iterator it_v = vList.begin(); 
  unsigned int p_rlErrorCode; 

  if(lResult == MMC_SUCCESS)
  {
    while(it_t != tsList.end())
    {
      long targetVel = (*it_v); 
      int targetTs = (*it_t); 

      if(VCS_MoveWithVelocity(mp_KeyHandle, m_usNodeId, targetVel, &p_rlErrorCode) == 0)
      {
        LogError("VCS_MoveToPosition", lResult, p_rlErrorCode); 
        lResult = MMC_FAILED; 
        return lResult; 
      }
      // sleep(targetTs);
      usleep(1000*targetTs);
      ++it_t; 
      ++it_v;
    }
    haltVelMove(); 
  }
  return lResult;
}

int CRoboCane::moveVelocity(long vel, int ts)
{
  int lResult = MMC_SUCCESS; 
  if(m_mode != VelMode)
  {
    cout << " moveVelocity not in VelMode, setVelMode !"<<endl; 
    lResult = setVelMode(); 
  } 

  LogInfo("before move velocity");
  unsigned int  p_rlErrorCode ;
  if(lResult == MMC_SUCCESS)
  {
    if(VCS_MoveWithVelocity(mp_KeyHandle, m_usNodeId, vel, &p_rlErrorCode) == 0)
    {
      lResult = MMC_FAILED;
      LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
      return lResult; 
    }
    // sleep(ts);
    usleep(1000*ts);
  }
  // LogInfo("after move velocity");
  
  haltVelMove(); 

  return lResult; 
}

int CRoboCane::setPosMode()
{
  unsigned int p_rlErrorCode;
  int lResult = MMC_SUCCESS;
  if(VCS_ActivateProfilePositionMode(mp_KeyHandle, m_usNodeId, &p_rlErrorCode) == 0)
  {
	LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
	return MMC_FAILED;
  } 
  
  unsigned int default_max_following_error;
  if(VCS_GetMaxFollowingError(mp_KeyHandle, m_usNodeId, &default_max_following_error, &p_rlErrorCode)== 0){
    LogError("VCS_GetMaxFollowingError", 0, p_rlErrorCode);
  }else{
    // cout<<" setPoseMode() maxFollowingError = "<<default_max_following_error<<endl;
    if(default_max_following_error != m_maxFollowingError) 
    {
      if(VCS_SetMaxFollowingError(mp_KeyHandle, m_usNodeId, m_maxFollowingError, &p_rlErrorCode) == 0){
        LogError("VCS_SetMaxFollowingError", 0, p_rlErrorCode); 
        return MMC_FAILED;
      }
    }
  }

  // set the velocity and acceleration for the position mode
  if(VCS_SetPositionProfile(mp_KeyHandle, m_usNodeId, m_profileVelocity, m_profileAcceleration, m_profileDeceleration, &p_rlErrorCode) == 0)
  {
      LogError("VCS_SetPositionProfile", lResult, p_rlErrorCode); 
      return MMC_FAILED; 
  }

  m_mode = PosMode; 
  return MMC_SUCCESS; 
}

int CRoboCane::setVelMode()
{
  int lResult = MMC_SUCCESS;
  unsigned int p_rlErrorCode;
  if(VCS_ActivateProfileVelocityMode(mp_KeyHandle, m_usNodeId, &p_rlErrorCode) == 0)
  {
	LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
	// lResult = MMC_FAILED;
        return MMC_FAILED; 
  }
  m_mode = VelMode; 
  return MMC_SUCCESS; 
}

int CRoboCane::haltPosMove()
{
  int lResult = MMC_SUCCESS;
  if(m_mode != PosMode)
  {
    // cerr<<" haltPosMove in VelMode, should call haltVelMove()"<<endl; 
    // return MMC_FAILED; 
    cout << "haltPosMove not in PosMode, setPosMode()"<<endl; 
    lResult = setPosMode(); 
  }
  
  unsigned int p_rlErrorCode;
  if(lResult == MMC_SUCCESS)
  {
    LogInfo("halt position movement");
    if(VCS_HaltPositionMovement(mp_KeyHandle, m_usNodeId, &p_rlErrorCode) == 0)
    {
      LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
      lResult = MMC_FAILED;
    }
  }
  return lResult; 
}

int CRoboCane::haltVelMove()
{
  int lResult = MMC_SUCCESS; 
  if(m_mode != VelMode)
  {
    // cerr<<" haltVelMove in PosMode, should call haltPosMove()"<<endl; 
    // return MMC_FAILED; 
    cout << "haltVelMove not in VelMode, setVelMode()"<<endl;
    lResult = setVelMode(); 
  }

  unsigned int p_rlErrorCode; 
  if(lResult == MMC_SUCCESS)
  {
    LogInfo("halt velocity movement");
    if(VCS_HaltVelocityMovement(mp_KeyHandle, m_usNodeId, &p_rlErrorCode) == 0)
    {
      LogError("VCS_HaltVelocityMovement", lResult, p_rlErrorCode);
      lResult = MMC_FAILED;
    }
  }
  return lResult; 
}

void CRoboCane::printSettings()
{
  stringstream msg; 
	msg << "default settings:" << endl;
	msg << "node id             = " << m_usNodeId << endl;
	msg << "device name         = '" << m_deviceName << "'" << endl;
	msg << "protocal stack name = '" << m_protocolStackName << "'" << endl;
	msg << "interface name      = '" << m_interfaceName << "'" << endl;
	msg << "port name           = '" << m_portName << "'"<< endl;
	msg << "baudrate            = " << m_baudrate;

	LogInfo(msg.str());

	SeparatorLine();
}




