#include <iostream>
#include <cmath>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/vector_c.hpp>
#include <boost/mpl/int.hpp>
// #include <boost/mpl/plus.hpp>
#include <boost/static_assert.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/mpl/equal.hpp>
#include <boost/mpl/placeholders.hpp>


using namespace std;
namespace mpl=boost::mpl;
using namespace mpl::placeholders;

typedef mpl::vector_c<int,1,0,0,0,0,0,0> mass;
typedef mpl::vector_c<int,0,1,0,0,0,0,0> length; 
typedef mpl::vector_c<int,0,0,1,0,0,0,0> time_;
typedef mpl::vector_c<int,0,0,0,1,0,0,0> charge;
typedef mpl::vector_c<int,0,0,0,0,1,0,0> temperature;
typedef mpl::vector_c<int,0,0,0,0,0,1,0> intensity;
typedef mpl::vector_c<int,0,0,0,0,0,0,1> angle;

typedef mpl::vector_c<int,0,1,-1,0,0,0,0> velocity; // l/t
typedef mpl::vector_c<int,0,1,-2,0,0,0,0> acceleration; // l/t2
typedef mpl::vector_c<int,1,1,-1,0,0,0,0> momentum; // mv
typedef mpl::vector_c<int,1,1,-2,0,0,0,0> force; // ma
typedef mpl::vector_c<int,1,2,-2,0,0,0,0> kinetic_energy; // mv2 = fl = mal

template<typename T, typename Dimensions>
struct quantity
{
public:
	explicit quantity(T x): m_value(x){}
	template<typename OtherDimension>
	quantity( quantity<T,OtherDimension> const& rth):m_value(rth.value())
	{
		BOOST_STATIC_ASSERT((
			mpl::equal<Dimensions,OtherDimension>::type::value
		));
	}

	T value() const { return m_value;}
private:
	T m_value;	
};

template<typename T, typename D>
quantity<T,D> operator+(quantity<T,D> x, quantity<T,D> y)
{
	return quantity<T,D>(x.value()+y.value());
}
template<typename T, typename D>
quantity<T,D> operator-(quantity<T,D> x, quantity<T,D> y)
{
	return quantity<T,D>(x.value()-y.value());
}

// typedef boost::mpl::vector<
//	signed char, short, int, long> signed_type;

struct plus_t
{
	template<typename T1, typename T2>	
	struct apply{
		typedef typename mpl::plus<T1,T2>::type type;
	};
};

template<typename T, typename D1, typename D2>
quantity<T
	, typename mpl::transform<D1,D2,plus_t>::type 
>
 operator*(quantity<T,D1> x, quantity<T,D2> y)
{
	typedef typename mpl::transform<D1,D2,plus_t>::type dim;
	return quantity<T,dim>(x.value()*y.value());
}

template<typename T, typename D1, typename D2>
quantity<T, typename mpl::transform<D1,D2,mpl::minus<_1,_2> >::type >
operator/(quantity<T,D1> x, quantity<T,D2> y)
{
	typedef typename mpl::transform<D1,D2,mpl::minus<_1,_2> >::type dim;
	return quantity<T,dim>(x.value()/y.value());
}

int main()
{
	// signed_type p;
	static int const five = mpl::int_<5>::value;
	cout<<"Hello MPL: "<<five<<endl;
	quantity<float, mass> m1(5.0f);
	quantity<float, mass> m2(1.3);
	cout<<"m1+m2: "<<(m1+m2).value()<<endl;
	quantity<float, acceleration> a(9.8f);
	quantity<float, force> f = m1 * a;
	cout<<"f = m*a : "<<f.value()<<endl;
	quantity<float, mass> m3 = f/a;
	cout<<"m = f/a : "<<m3.value()<<endl;
	cout<<"rounding error: "<<std::fabs((m3-m1).value())<<endl;
	quantity<float, length> l(1.2);

	return 0;
}

