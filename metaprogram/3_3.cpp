// Turn T into T**** by using twice twice.
// Turn T into T**** using twice on itself.

#include <iostream>
#include <boost/type_traits.hpp>
#include <boost/mpl/apply.hpp>
#include <boost/mpl/placeholders.hpp>
// #include <boost/is_same.hpp>
#include <boost/static_assert.hpp>
#include <boost/mpl/lambda.hpp>
#include <boost/mpl/plus.hpp>

using namespace std;

namespace mpl=boost::mpl;
using namespace mpl::placeholders;

// template<typename F, typename X>
// struct twice : F::template apply< typename F::template apply<X>::type >
// {};

template <class UnaryMetaFunctionClass, class Arg>
struct apply1 : UnaryMetaFunctionClass::template apply<Arg>
{};

// template<typename F, typename X>
// struct twice : apply1< F, typename apply1<F,X>::type >
// {};

template<typename F, typename X>
struct twice : apply1<typename mpl::lambda<F >::type, typename apply1< typename mpl::lambda<F >::type, X>::type >
{};

struct add_pointer_f
{
        template <class T>
        struct apply : boost::add_pointer<T> {};
};

template<typename F, typename X>
struct fourth : twice<typename mpl::lambda<F>::type , typename twice< typename mpl::lambda<F>::type,X >::type >
{};

template <typename UnaryMetaFunction, typename N>
struct apply2 : UnaryMetaFunction::template apply<N>{};

template<typename N>
struct partial_metafunction : apply2< typename mpl::lambda<mpl::plus<_1,mpl::int_<42> > >::type, N>
{};

template<typename F, typename X>
struct twice2 : mpl::apply<typename mpl::lambda<F>::type, typename mpl::apply<typename mpl::lambda<F>::type, X>::type >
{};

int main()
{
	BOOST_STATIC_ASSERT((
				boost::is_same<
				twice<add_pointer_f, int>::type
				, int**
				>::value
			    ));
	int a = 1;
	int *x = &a;
	twice<add_pointer_f, int>::type px = &x;
	// twice<mpl::lambda<boost::add_pointer<_1> >::type ,int>::type qx = &x;
	twice<boost::add_pointer<_1>, int >::type qx = &x;
	int ***y;
	fourth<boost::add_pointer<_1>, int>::type py = &y;
	BOOST_STATIC_ASSERT((
		boost::is_same<
			fourth<boost::add_pointer<_1>, int>::type,
			int****
		>::value
	));	
	partial_metafunction<mpl::int_<10> > t;

	BOOST_STATIC_ASSERT((
		boost::is_same<
			twice2<boost::add_pointer<_1>, int>::type, 
			int**
		>::value
	));
	cout<<"px: "<<px<<" *px: "<<*px<<" **px: "<<**px<<endl;
	return 0;
}
