#include <iostream>
#include <assert.h>
#include <iterator>
#include <string>
#include <boost/static_assert.hpp>
#include <boost/mpl/if.hpp>
#include <boost/mpl/eval_if.hpp>
#include <boost/mpl/identity.hpp>
#include <boost/mpl/or.hpp>
#include <boost/type_traits.hpp>
#include <boost/type_traits/add_reference.hpp>
#include <boost/type_traits/is_stateless.hpp>

#include <boost/mpl/vector.hpp>
#include <boost/mpl/equal.hpp>
#include <boost/mpl/replace.hpp>
#include <boost/mpl/replace_if.hpp>
#include <boost/mpl/placeholders.hpp>



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

// 2-1 execise // hard to realize!!!
// Write a ternary metafunction replace_type<c,x,y> that takes an arbitrary compound type c as its first parameter, and replaces all occurrences of a type x within c with y:
using namespace mpl::placeholders;
template<typename C, typename Old, typename New>
struct replace_type :
	mpl::replace<C, Old, New >::type
{};


struct A{
	virtual ~A(){}
};
struct B : A {};

template<typename Target, typename Source>
Target polymophic_downcast_helper(Source* x)
{
	assert(dynamic_cast<Target>(x) == x);
	return static_cast<Target>(x);
	// inline Target operator()(Source& x){}
}

template<typename Target, typename Source>
Target polymophic_downcast_helper(Source& x)
{
	return static_cast<Target>(x);
}

template<bool Is_Ref>
struct polymophic_downcast_warpper{
	template<typename Target, typename Source>
	static Target polymophic_downcast(Source x)
	{	
		cout<<"called Pointer cast!"<<endl;
		assert(dynamic_cast<Target>(x) == x);
		return static_cast<Target>(x);
	}
};

template<>
struct polymophic_downcast_warpper<true>{
	template<typename Target, typename Source>
	static Target polymophic_downcast(Source x)
	{
		cout<<"called Ref cast!"<<endl;
		return static_cast<Target>(x);
	}
};

template<typename Target, typename Source>
Target polymophic_downcast_helper2(Source x)
{
	/*typedef iterator_traits<Source> traits;
	typedef typename traits::reference ref;
	typedef typename traits::pointer point;*/
	bool const is_ref = boost::is_reference<Source>::value;
	return polymophic_downcast_warpper<is_ref>::template polymophic_downcast<Target,Source>(x);
}

int main()
{
	typedef mpl::vector<int, float> types;
	typedef mpl::vector<void, float> expected;
	typedef mpl::replace<types, int, void>::type result;
	BOOST_STATIC_ASSERT((
		mpl::equal<expected, result>::value
		// boost::is_same<expected::type , result::type>::value
	));
	/*BOOST_STATIC_ASSERT(( 
		boost::is_same< mpl::replace<types, int, void>::type,
				expected::type
		 >::value
		boost::is_same< typename replace_type<int*, int, void>::type, 
			void*
		>::value 
	));*/
	B b;
	A* pa = &b;
	A& ra = b;
	// B* pb = polymophic_downcast_helper<B*>(pa);
	// B& rb = polymophic_downcast_helper<B&>(ra);
	B* pb = polymophic_downcast_helper2<B*>(pa);
	B& rb = polymophic_downcast_helper2<B&,A&>(ra);
	// B& pb = polymophic_downcast_helper<B&>(a); 
	
	return 0;
}


