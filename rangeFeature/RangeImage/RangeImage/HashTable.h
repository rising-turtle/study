#ifndef HASHTABLE_H
#define HASHTABLE_H

#include "preheader.h"

template<typename F,typename T>
class CHashTableSet
{
public:
	CHashTableSet(size_t N=10007);
	~CHashTableSet();

	vector< set<T> > m_hasht;
	size_t m_N;
	F m_func;
	bool find(const T& );
	void insert(const T&);
	void clear();
};

#include "HashTable.hpp"

#endif