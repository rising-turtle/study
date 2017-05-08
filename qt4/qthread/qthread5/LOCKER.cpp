#include "LOCKER.h"
#include <QMutexLocker>
#include <iostream>

using namespace std;
CLocker::CLocker():k(0)
{}

void CLocker::start()
{	
	QMutexLocker locker(&mutex);
	int sleep = 100;
	while(1){
		sleep = 2*sleep;
		if(sleep>100000000) sleep = 100000000;
		jump(sleep);
		if(k>0){
			cout<<"now K is: "<<k<<endl;
			break;
		}
		//cout<<"now K is: "<<k<<endl;
		}
	jump(-1);
	cout<<"My locker has been stolen, Oh God!"<<endl;
}

void CLocker::change()
{
	QMutexLocker locker(&mutex);
	k++;
	cout<<"WaO-, I steal the locker: "<<k<<endl;
}
