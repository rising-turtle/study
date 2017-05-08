#include "HashTable.h"

template<typename F,typename T>
CHashTableSet<F,T>::CHashTableSet(size_t N):m_N(N)
{
	m_hasht.resize(m_N);
}

template<typename F,typename T>
CHashTableSet<F,T>::~CHashTableSet(){}

template<typename F,typename T>
bool CHashTableSet<F,T>::find(const T& key)
{
	size_t index_t = m_func(key);
	index_t%=m_N;
	if(m_hasht[index_t].find(key)==m_hasht[index_t].end())
	{
		return false;
	}
	return true;
}
template<typename F,typename T>
void CHashTableSet<F,T>::insert(const T& key)
{
	size_t index_t = m_func(key);
	index_t%=m_N;
	m_hasht[index_t].insert(key);
}

template<typename F,typename T>
void CHashTableSet<F,T>::clear()
{
	for(size_t i=0;i<m_N;i++)
		m_hasht[i].clear();
}
