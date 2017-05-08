#ifndef _TEMPLATE_H_
#define	_TEMPLATE_H_

#include "preheader.h"

namespace template1{
template<typename T>
class C1{
public:
	C1(){}
	~C1(){}
	static const T m_sElem;
}; 

template<typename T>
const T C1<T>::m_sElem = static_cast<T>(2.0);

template<>
class C1<string>{
public:
	C1(){}
	~C1(){}
	static const string m_sElem;
};

//const string C1<string>::m_sElem = string("OK");

}
#endif