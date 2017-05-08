#include "consumer.h"
#include <iostream>

using namespace std;

CConsumer::CConsumer(){}

void CConsumer::consume(int n)
{
	cout<<"get ticket: "<<n<<endl;
	consumed();
}

void CConsumer::finish()
{
	cout<<"No ticket is avaliable!"<<endl;
	exitThread();
}

