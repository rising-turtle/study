// execise 3-1
// # Turn vector_c<int,1,2,3> into a type sequence with elements (2,3,4) using transform.

#include <iostream>
#include <boost/static_assert.hpp>
#include <boost/mpl/vector_c.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/mpl/placeholders.hpp>
#include <boost/mpl/equal.hpp>

using namespace std;
namespace mpl=boost::mpl;
using namespace mpl::placeholders;

typedef mpl::vector_c<int,1,2,3> from_;
typedef mpl::vector_c<int,1,1,1> incr_;
typedef mpl::vector_c<int,2,3,4> to_;

template<typename D>
struct middleware
{
	middleware(){}
	template<typename O>
	middleware(middleware<O> const& rhs){
		BOOST_STATIC_ASSERT((
			mpl::equal<D,O>::type::value
		));
	}
};

// execise 3-2
// Turn vector_c<int,1,2,3> into a type sequence with elements (1,4,9) using TRansform.

int main()
{
	typedef mpl::transform<from_,incr_,mpl::plus<_1,_2> >::type to2;
	middleware<to2> test2;
	middleware<to_> test1 = test2;
	
	cout<<"succeed!"<<endl;
	return 0;
}
