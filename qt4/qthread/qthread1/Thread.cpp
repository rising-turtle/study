#include "Thread.h"
#include <iostream>

using namespace std;

Thread::Thread():stopped(false)
{}

void Thread::setMessage(const QString& message){
	messageStr = message;
}

void Thread::run(){
	while(!stopped)
		cerr<<qPrintable(messageStr);
	stopped = false;
	cerr<<endl;
}

void Thread::stop()
{
	stopped = true;
}

