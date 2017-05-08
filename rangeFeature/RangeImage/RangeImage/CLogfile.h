#ifndef CLOGFILE_H
#define CLOGFILE_H

#include "preheader.h"
#include <boost/thread/thread.hpp>

class CLogfile{
public:
	CLogfile(){
		filename.assign("D:\\CLogfile.log");
		// first create this class initialize the first line
		if(CLogfile::first_open)
		{
			file_handle.open(filename.c_str(),ios::out);
			file_handle<<"Frame "<<"\tGS"<<"\tFE "<<"\tFM"<<"\tR_ME"<<"\tOP "<<"\tSLAM"<<"\n";
			file_handle.close();
			CLogfile::first_open = false;
		}
	}
	~CLogfile(){
		//if(file_handle.is_open())
		//	file_handle.close();
	}
	inline void getlogfile(){
		logmutex.lock();
		file_handle.open(filename.c_str(),ios::app|ios::out);
	}
	inline void releaselogfile(){
		file_handle.close();
		logmutex.unlock();
	}
	inline void writeintolog(double& start_t, double& end_t, bool anotherline = false){
		static char buf[20];
		sprintf(buf,"\t%0.0f ",end_t - start_t);
		std::string str(buf);
		writeintolog(str,anotherline);
	}
	inline void writeintolog(double& time, bool anotherline = false){
		static char buf[20];
		sprintf(buf,"\t%0.0f ",time);
		std::string str(buf);
		writeintolog(str,anotherline);
	}
	inline void writeintolog(std::string& str,bool anotherline = false){
		getlogfile(); // lock file
		if(anotherline)
			file_handle<<str<<std::endl;
		else
			file_handle<<str;
		if(anotherline){
			file_handle.flush();
		}
		releaselogfile(); // release file
	}
	inline void writeid(int id1){
		static char buf[20];
		sprintf(buf,"%d ",id1);
		std::string str(buf);
		getlogfile();
			file_handle<<str;
		releaselogfile();
	}
public:
	fstream file_handle;
	string filename;
	boost::mutex logmutex;

	static bool first_open;
};


#endif