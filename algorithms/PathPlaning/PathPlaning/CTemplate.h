#ifndef CTEMPLATE_H
#define CTEMPLATE_H

extern double foo(double);

template<typename T>
class CTemplate1{
public:
	CTemplate1(T v1=0):_v1(v1),_i1(2){}
	~CTemplate1(){}

	void type_Independent();
	void type_dependent();
private:
	T _v1;
	int _i1;
};

#include "CTemplate.hpp"

#endif