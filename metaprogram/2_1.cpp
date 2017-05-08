#include <iostream>
#include <string>

#include <boost/type_traits.hpp>
#include <boost/static_assert.hpp>

template<typename C, typename X, typename Y>
struct type_replace{};

template<typename X, typename Y>
struct type_replace<X*,X,Y>{
	typedef Y* type;
};

template<typename X, typename Y>
struct type_replace<X&,X,Y>{
	typedef Y& type;
};

int main()
{
	typedef type_replace<int*,int,void>::type tv;
	BOOST_STATIC_ASSERT((
		boost::is_same<tv, void*>::value
	));
	return 0;
}
