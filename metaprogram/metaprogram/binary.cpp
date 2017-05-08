#include <iostream>
#include <boost/static_assert.hpp>
#include <boost/type_traits/is_same.hpp>
using namespace std;
namespace mpl=boost::mpl;

template<int N>
struct binary
{
	BOOST_STATIC_ASSERT((
		boost::is_same<mpl::int_<N%10>,mpl::int_<0> >::value || \
		boost::is_same<mpl::int_<N%10>,mpl::int_<1> >::value
	));
	static const unsigned int value =
	binary<N/10>::value << 1 | N%10;
};

template<>
struct binary<0>
{
	static const signed int value = 0;
};

int main(int argc, char* argv[])
{
	if(argc == 1)
	{ 
		 cout<<"b<101>: "<<binary<101>::value<<endl;
	  	 cout<<"b<11111>: "<<binary<11111>::value<<endl;
		 // cout<<"b<21>: "<<binary<21>::value<<endl;
	}else
	{
	
	}
	return 0;
}
