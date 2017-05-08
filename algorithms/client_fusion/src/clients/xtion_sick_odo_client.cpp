#include "xtion_sick_odo_client.h"
#include "sick_client.h"
#include "odo_client.h"

#include "HomeMessages/messages_@home_zjupanda_odometer.pb.h"
#include "HomeMessages/messages_@home_zjupanda_laser.pb.h"

DWORD WINAPI CXSickOdoClient::thread_start_record_xtion(LPVOID param_)
{
	CXtionClient* pClient = static_cast<CXtionClient*>(param_);
	pClient->startRecordXtion();
	return 1;
}

CXSickOdoClient::CXSickOdoClient(string path):
m_pSickClient(new CSickClient(path)),
m_pOdoClient(new CODOClient(path))
{}

CXSickOdoClient::~CXSickOdoClient(){
	if(m_pSickClient != 0) delete m_pSickClient;
	if(m_pOdoClient != 0) delete m_pOdoClient;
}

bool CXSickOdoClient::connectBoth(string ip, int portSick, int portOdo)
{
	bool ret = true; 
	ret &= m_pSickClient->connect(ip.c_str(), portSick); 
	ret &= m_pOdoClient->connect(ip.c_str(), portOdo);
	return ret;
}

bool CXSickOdoClient::startRecordData()
{
	if(m_bRecord)
	{
		cout<<"xtion_sick_odo_client.cpp: already listen to SICK and ODO data!"<<endl;
		return true;
	}
	static bool once = false;	
	HANDLE thread_record;
	DWORD thread_record_id;
	thread_record = CreateThread(0,0, (LPTHREAD_START_ROUTINE)&CXSickOdoClient::thread_start_record_xtion, \
		(LPVOID)this,0,&thread_record_id);
	if(thread_record == 0)
	{
		cout<<"xtion_sick_odo_client.cpp: failed to create thread in startRecordData()"<<endl;
		return false;
	}

	m_pOdoClient->m_bRecord = true;
	m_pSickClient->m_bRecord = true;

	if(!once )
	{
	
		NODE.init("sick_odo_client");
		boost::shared_ptr<Subscriber<Home_Odometer> >  subOdoPtr = NODE.subscribe<Home_Odometer>("home_odometer",boost::bind(&CODOClient::odoCallBack,m_pOdoClient,_1));
		boost::shared_ptr<Subscriber<Home_Laser> >  subLaserPtr = NODE.subscribe<Home_Laser>("home_laser",boost::bind(&CSickClient::laserCallBack,m_pSickClient,_1));
		NODE.spin();
		once = true;
	}
	return true;
}

void CXSickOdoClient::stopRecordData()
{
	cout<<"xtion_sick_odo_client.cpp: set m_bRecord = false;"<<endl;
	m_bRecord = false;
	m_pSickClient->m_bRecord = false;
	m_pOdoClient->m_bRecord = false;
}

bool CXSickOdoClient::startSendDataOffline()
{
	bool ret = true;
	if(m_pSickClient->startSendDataOffline())
		cout<<"xtion_sick_odo_client.cpp: succeed to send sick data!"<<endl;
	else 
		ret = false;
	if(m_pOdoClient->startSendDataOffline())
		cout<<"xtion_sick_odo_client.cpp: succeed to send odo data!"<<endl;
	else 
		ret = false;
	if(CXtionClient::startSendDataOffline())
		cout<<"xtion_sick_odo_client.cpp: succeed to send xtion data!"<<endl;
	else
		ret =false;
	return ret;
}

