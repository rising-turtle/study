#ifndef XSICK_ODO_CLIENT_H
#define XSICK_ODO_CLIENT_H

#include "socket_client.h"
#include "xtion_client.h"
class CSickClient;
class CODOClient;

class CXSickOdoClient : public CXtionClient
{
public:
	CXSickOdoClient(string path = "D:\\client_fusion\\data");
	~CXSickOdoClient();
	bool connectBoth(string, int portSick, int portOdo);
	bool startRecordData();
	void stopRecordData();
	bool startSendDataOffline();
protected:
	CSickClient* m_pSickClient; 
	CODOClient* m_pOdoClient;

public:
	static DWORD WINAPI thread_start_record_xtion(LPVOID param_);
};

#endif