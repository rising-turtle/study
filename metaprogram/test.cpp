#include <iostream>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/vector_c.hpp>
#include <boost/mpl/equal.hpp>
#include <boost/type_traits.hpp>
#include <boost/mpl/at.hpp>
#include <boost/mpl/size.hpp>
#include <boost/mpl/less_equal.hpp>
#include <boost/mpl/less.hpp>
#include <boost/mpl/int.hpp>

namespace mpl=boost::mpl;
using namespace std;

template<bool B, typename T1, typename T2, typename Pos, typename Fin>
struct equal_element{
	static bool const value = mpl::equal<typename mpl::at<T1,Pos>::type, typename mpl::at<T2,Pos>::type>::value && \
		equal_element< mpl::less<Pos,Fin>::value ,T1,T2, mpl::plus<Pos, mpl::int_<1> >, Fin >::value;	
	typedef mpl::bool_<value> type;
};

template<bool B, typename T1, typename T2, typename F>
struct equal_element<B,T1,T2,F,F>{
	static bool const value = true;
	typedef mpl::bool_<true> type;
};

template<typename T1, typename T2, typename Pos, typename Fin>
struct equal_element<false,T1,T2,Pos,Fin>{
	typedef mpl::false_ type;
	static bool const value = true;
};
template<typename T, typename Pos, typename Fin>
struct equal_element<false,T,T,Pos,Fin>{
	typedef mpl::false_ type;
	static bool const value = true;
};

template<typename T1, typename T2>
struct is_equal
{
	typedef typename mpl::bool_<mpl::size<T1>::value == mpl::size<T2>::value>::type size_equal;
	typedef typename mpl::int_<mpl::size<T1>::value> F;
	typedef typename mpl::int_<0> S;
	typedef typename equal_element<mpl::less<S,F>::value, T1, T2, S, F>::type type;
	static const bool value = mpl::and_<size_equal,type>::type::value;
};

int main()
{
	typedef mpl::vector_c<int,1,5> t1;
	typedef mpl::vector<mpl::int_<1>, mpl::int_<5> > t2;
	/*if(is_equal<t1,t2>::value)
	{
		cout<<"t1 == t2!"<<endl;
	}else
	{
		cout<<"t1 != t2!"<<endl;
	}*/
	if(mpl::equal<t1, t2>::value)
	{
		cout<<"t1 equal t2!"<<endl;
	}
	if(boost::is_same<t1,t2>::value)
	{
		cout<<"t1 same with t2!"<<endl;
	}
	if(mpl::equal<typename mpl::at<t1,mpl::int_<2> >::type, mpl::int_<5> >::value)
	{
		cout<<"t1.at<2> == int_<5>"<<endl;
	}
	if(mpl::equal<typename mpl::at<t2,mpl::int_<2> >::type, mpl::int_<5> >::value)
	{
		cout<<"t2.at<2> == int_<5>"<<endl;
	}

	
	return 0;
}
