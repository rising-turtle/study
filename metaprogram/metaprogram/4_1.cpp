#include <iostream>
#include <boost/static_assert.hpp>
#include <boost/mpl/if.hpp>
#include <boost/mpl/eval_if.hpp>
#include <boost/mpl/identity.hpp>
#include <boost/mpl/or.hpp>
#include <boost/type_traits.hpp>
#include <boost/type_traits/add_reference.hpp>
#include <boost/type_traits/is_stateless.hpp>

using namespace std;
namespace mpl=boost::mpl;

template <class T>
struct param_type
: mpl::eval_if<
	mpl::or_<
	 boost::is_scalar<T>
	 , boost::is_stateless<T>
	 , boost::is_reference<T>
	 >
	 , mpl::identity<T>
	 , boost::add_reference<T const>
	 >
{};


/*struct print_scalar{
	template <typename T>
	struct apply<T>{
	};
};*/


// execise 2
// Write a unary metafunction add_const_ref<T> that returns T if it is a reference type, and otherwise returns T const&. Write a program to test your metafunction. Hint: you can use boost::is_same to test the results.

template<typename T>
struct add_const_ref : 
	mpl::if_<
		typename boost::is_reference<T>::type,
		mpl::identity<T>,
		boost::add_reference<const T>
	>::type{};

template<typename T>
struct test {
	// typedef typename mpl::identity<T>::type type; 
	typedef typename boost::is_reference<T>::type type;
}; 


/*
int main()
{
	BOOST_STATIC_ASSERT((
		boost::is_same<
			typename add_const_ref<int>::type ,
			const int&
		>::value
	));

	BOOST_STATIC_ASSERT((
		boost::is_same<
			boost::add_reference<const int>::type ,
			mpl::identity<const int&>::type
		>::value
	));

	return 0;
}
*/

// 2-1 execise 
// Write a ternary metafunction replace_type<c,x,y> that takes an arbitrary compound type c as its first parameter, and replaces all occurrences of a type x within c with y:

#include <boost/mpl/replace_if.hpp>
#include <boost/mpl/placeholders.hpp>

using namespace mpl::placeholders;
template<typename C, typename Old, typename New>
struct replace_type :
	mpl::replace_if<C, boost::is_same<_,Old>, New >::type
{};

int main()
{
	BOOST_STATIC_ASSERT(( 
		boost::is_same< typename replace_type<int*, int, void>::type, 
			void*
		>::value ));
	return 0;
}


