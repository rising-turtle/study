#include "client_interface.h"

CClientInterface::CClientInterface(string configf )
{
	init(configf);
}

CClientInterface::~CClientInterface(){}
void CClientInterface::uninit()
{
	for(int i=0; i<m_client_impls.size();i++)
	{
		// 1 disconnect
		// 2 delete 
		delete m_client_impls[i]; 
	}
	m_client_impls.clear();
}

void CClientInterface::init(string configf)
{
	if(configf !="")
	{
		//TODO: config from file
		
	}else
	{
		// 1 SICK client 
		CSocketClient * pSICK = new CSickClient;
		m_client_impls.push_back(pSICK);
		// 2 ODO client 
		CODOClient * pODO = new CODOClient;
		m_client_impls.push_back(pODO);
		// 3 TODO add Xtion client
	}
	
}

bool CClientInterface::createThreadsRecord()
{
	vector<HANDLE> threads;
	vector<DWORD> ids;
	for(int i=0; i<m_client_impls.size(); i++)
	{
		if(!m_client_impls[i]->connect(m_ip_impls[i].c_str(), m_ports_impls[i]))
		{
			cout<<"client_interface.cpp: failed to connect in createThreadsRecord!"<<endl;
			return false;
		}
		threads[i] = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)&CSocketClient::thread_start_record,\
				(LPVOID)m_client_impls[i], 0, &ids[i]);
		if(threads[i] == NULL)
		{
			cout<<"client_interface.cpp: failed to createThreadsRecord."<<endl;
			return false;
		}
	}	
	cout<<"client_interface.cpp: succeed to start all threads!"<<endl;
	return true;
}

bool CClientInterface::createThreadsSend()
{
	vector<HANDLE> threads;
	vector<DWORD> ids;
	for(int i=0; i<m_client_impls.size(); i++)
	{
		if(!m_client_impls[i]->connect(m_ip_impls[i].c_str(), m_ports_impls[i]))
		{
			cout<<"client_interface.cpp: failed to connect in createThreadsSend!"<<endl;
			return false;
		}
		threads[i] = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)&CSocketClient::thread_start_send,\
			(LPVOID)m_client_impls[i], 0, &ids[i]);
		if(threads[i] == NULL)
		{
			cout<<"client_interface.cpp: failed to createThreadsSend."<<endl;
			return false;
		}
	}

	cout<<"client_interface.cpp: succeed to start all threads!"<<endl;
	return true;
}


bool CClientInterface::startRecordData()
{
	return createThreadsRecord();
}

bool CClientInterface::startSendDataOffline()
{
	return createThreadsSend();
}

void CClientInterface::stopSendDataOffline()
{
	for(int i=0; i<m_client_impls.size(); i++)
		m_client_impls[i]->stopSendDataOffline();
}

void CClientInterface::stopRecordData()
{
	for(int i=0; i<m_client_impls.size(); i++)
		m_client_impls[i]->stopRecordData();
}

bool CClientInterface::run()
{
	if(!connect(m_ip.c_str(), m_port))
	{
		cout<<"client_interface.cpp: failed to connect with ip: "<<m_ip<<" at port "<<m_port<<endl;
		return false;
	}
	char rbuf[4096];
	memset(rbuf, 0, sizeof(rbuf));
	int cmd  = -1; 
	bool m_close = false;

	vector<HANDLE> m_thread_handle;
	vector<DWORD> m_thread_id;
	while (1)
	{
		while(recvData(rbuf, sizeof(rbuf)))
		{
			memcpy(&cmd, rbuf, sizeof(4));
			switch(cmd)
			{
			// create threads and asy time
			case 0: 
				cout<<"cmd = 0, OK, let's asy our time!"<<endl;
				unsigned long sec, usec; 
				memcpy(&sec, rbuf +4, sizeof(unsigned long));
				memcpy(&usec, rbuf +4 +sizeof(unsigned long), sizeof(unsigned long));
				g_dwHighDateTime = sec; 
				g_dwLowDateTime = usec;
				g_has_time_asy = true;
			
				cout<<"client rece asy time: "<<sec<<"."<<usec<<endl;
				break;
			case 1:
				cout<<"cmd = 1, OK, let's record data to local disk!"<<endl;
				if(startRecordData())
				{
					cout<<"client_interface.cpp: succeed to create threads!"<<endl;
				}else{
					cout<<"client_interface.cpp: failed to create threads!"<<endl;
				}
				Sleep(10);
				break; 
			case 2:
				cout<<"cmd = 2, OK, let's stop send xtion data!"<<endl;
				stopRecordData();
				// client.stopSendXtion();
				break; 
			case 3:
				cout<<"cmd = 3, OK, let's clear all the data in the DIR!"<<endl;
				
				break; 
			case 4:
				cout<<"cmd = 4, OK, let's send all the data to server offline!"<<endl;
				if(startSendDataOffline())
				{
					cout<<"client_interface.cpp: succeed to create threads!"<<endl;
				}else
				{
					cout<<"client_interface.cpp: failed to create threads!"<<endl;
				}
				Sleep(10);
				break;
			case 9:
				cout<<"cmd =9, OK, let's close"<<endl;
				m_close = true;
				break;
			default:
				cout<<"Oh, no, Unknown cmd: "<<cmd<<" let's close"<<endl;
				// m_close = true;
				break;
			}
			if(m_close ) break;
			Sleep(1); // sleep 1ms
		}
		// 
	}
	return true;
}