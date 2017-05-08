#ifndef FUNC_TEMP_H
#define FUNC_TEMP_H

#include <iostream>

using namespace std; 

template<typename Tin, typename Tout>
extern Tout* cast_go(Tin* ); 

template<typename T1, typename T2>
extern void swap(T1* p1, T2* p2)
{
  T2 * pc1 = cast_go<T1, T2>(p1); 
  T1 * pc2 = cast_go<T2, T1>(p2); 
  cout<<" p1: "<<*p1<<" p2: "<<*p2<<" pc1: "<<*pc1<<" pc2: "<<*pc2<<endl;
  return ;
}

template<typename Tin, typename Tout>
Tout * cast_go(Tin* pin)
{
  Tout * pout = static_cast<Tout*>(pin);
  // Tout * pout = new Tout; 
  // *pout = *pin; //= static_cast<Tout*>(pin); 
  return pout; 
}


#endif
