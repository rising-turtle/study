#include "producer.h"
#include <iostream>

using namespace std;

CProducer::CProducer():m_ticket(0),m_all_ticket(100){}

void CProducer::produce()
{
	if(++m_ticket>m_all_ticket){
		cout<<"all tickets are sold!"<<endl;
		finish();
		return ;
	}
	cout<<"selling ticket: "<<m_ticket<<endl;
	produced(m_ticket);
}
