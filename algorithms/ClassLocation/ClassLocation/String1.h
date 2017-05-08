#ifndef STRING1_H
#define STRING1_H
//#include <string.h>
#include <iostream>

using namespace std;

class String1 
{ 
private:
	char * m_data;

public: 

	String1(const char *str = NULL) // 通用构造函数 
	{
		int N = strlen(str);
		if(N ==0) {
			m_data = NULL;
		}
		else
		{
			m_data = new char[N];
			for(int i=0;i<N;i++){
				m_data[i] = str[i];
			}
		}
	}

	String1(const String1 &another) // 拷贝构造函数 
	{
		if(m_data != NULL){
			delete []m_data;
		}
		int N = another.Size();
		if( N ==0) { m_data = NULL;}
		else{
		m_data = new char[];
			for(int i=0;i<N;i++){
				m_data[i] = another.At(i);
			}
		}
		
	}
	~String1() // 析构函数 
	{
		if(m_data != NULL)
			delete []m_data;
	}

	int Size() const { return strlen(m_data);}

	char At( int index) const {
		if(index < 0 || index >= strlen(m_data) )
		{
			cout<<"error at limit!"<<endl;
			return 'a';
		}
		return m_data[index];
	}

	String1& operator=(const String1& rhs)//(/*const String1& rhs*/)// 赋值函数 
	{
	
		if(this == &rhs){
			return (*this);
		}
		if(this->m_data != NULL){
			delete []m_data;
		}
		int N = rhs.Size();
		if(N == 0) {
			m_data = NULL;
			return (*this);
		}
		m_data = new char[N];
		for(int i=0;i<N;i++){
			m_data[i] = rhs.At(i);
		}
		return (*this);
	}

//private: 
//	char * m_data; // 用于保存字符串 
}; 
ostream& operator<<(ostream& out,String1& str){
	for(int i=0;i<str.Size();i++)
		out<<str.At(i);
	out<<endl;
	return out;
}

#endif