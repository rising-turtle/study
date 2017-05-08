#include "header.h"
/*
template<typename RandomIterator, typename Pos>
struct double_value  
{
	typedef typename mpl::at<RandomIterator,Pos>::type type_value;
	typedef mpl::plus< type_value, type_value> type;
};

template<typename RandomIterator, typename OutIterator, typename Current, typename HALF>
struct change_value: 
	mpl::eval_if<
	mpl::equal<Current,HALF>,
	OutIterator,
	typename change_value<RandomIterator, typename mpl::push_back<OutIterator, typename double_value<RandomIterator,Current>::type >::type, mpl::plus<Current, mpl::int_<1> >, HALF >::type
	>::type
{};

template<typename RandomIterator>
struct double_first_half{
	typedef typename mpl::clear<RandomIterator>::type EmptyIterator;
	static const int half =  mpl::size<RandomIterator>::type::value/2;
	typedef mpl::int_<half> HALF;
	typedef mpl::int_<0> START;
	typedef typename change_value<RandomIterator, EmptyIterator, START, HALF>::type type;
};
*/
template<typename Seq>
struct double_first_half2{
    typedef typename mpl::clear<Seq>::type initS;
	typedef typename mpl::begin<Seq>::type beginS;
	typedef typename mpl::end<Seq>::type endS;
	typedef typename mpl::divides<mpl::size<Seq>, mpl::int_<2> >::type mid_index;
	typedef typename mpl::advance<beginS, mid_index>::type midS;
	typedef typename mpl::iterator_range<beginS, midS>::type first_half;
	typedef typename mpl::iterator_range<midS, endS>::type second_half;
	typedef typename mpl::transform<first_half, mpl::times<_, mpl::int_<2> >, mpl::back_inserter<initS> >::type doubled_first_half;
    typedef typename mpl::insert_range<doubled_first_half, typename mpl::end<doubled_first_half>::type, second_half>::type type;
};

template<typename Seq>
struct double_first_half3 : mpl::transform<
                                Seq,
                                mpl::range_c<int, 0, mpl::size<Seq>::value>,
                                mpl::eval_if<
                                    mpl::less<_2, mpl::int_<mpl::size<Seq>::value/2> >,
                                    mpl::times<_1, mpl::int_<2> >,
                                    _1> >
{};

int main()
{
    typedef mpl::vector_c<int, 2,3,4,5> in1;
    typedef typename double_first_half2<in1>::type out1;
    typedef typename double_first_half3<in1>::type out2;
    typedef mpl::vector_c<int, 4,6,4,5> exp1;
    if(mpl::equal<out1, mpl::vector_c<int, 4,6,4,5> >::value)
    {
        cout<<"1 succeed!"<<endl;
    }
    if(mpl::equal<out2, exp1>::value)
    {
        cout<<"2 succeed!"<<endl;
    }
    return 0;
}

/*
int main()
{
	typedef mpl::vector_c<int,1,2,5,6> InType;
	typedef mpl::vector_c<int,2,4> ExpectedType;
	typedef typename mpl::clear<InType>::type Empty;
	
	BOOST_STATIC_ASSERT((
		// mpl::equal<Empty, mpl::vector_c<int> >::value
		mpl::equal< mpl::size<InType>::type, 
				mpl::int_<4> >::value
	));
	if(mpl::size<InType>::type::value /2 == 2)
	{
		cout<<"haha fine!"<<endl;
	}
	typedef mpl::plus<mpl::int_<0>, mpl::int_<1> > First;
	typedef mpl::plus<First, mpl::int_<1> > Second;
	if(mpl::equal<Second, mpl::int_<2> >::value)
	{
		cout<<"haha fine 2!"<<endl;
	}
	typedef typename mpl::at<InType, Second>::type Sec;
	typedef typename mpl::plus<Sec, Sec> Fourth;

	if(mpl::equal<Fourth, mpl::int_<4> >::value)
	{
		cout<<"haha fine 3!"<<endl;
	}

	typedef typename mpl::push_back<Empty, typename mpl::deref<Fourth>::type >::type Push4;
	typedef typename mpl::push_back<Empty, mpl::integral_c<int,4> >::type Push4_;
	typedef typename mpl::push_back<Push4, Second>::type Push42;
	if(mpl::equal<Push4, mpl::vector_c<int,4> >::value)
	{
		cout<<"haha fine 5!"<<endl;
	}
	if(mpl::equal<Push42, mpl::vector_c<int,4,2> >::value)
	{
		cout<<"haha fine 4!"<<endl;
	}

	typedef mpl::vector_c<bool, false, true> Test;
	typedef mpl::push_back<Test, mpl::false_ >::type Test2;
	BOOST_MPL_ASSERT_RELATION(mpl::size<Test2>::value, ==, 3);
	BOOST_MPL_ASSERT_RELATION((mpl::count_if<Test2, mpl::equal_to<_1, mpl::false_> >::value), == ,2);
	// BOOST_STATIC_ASSERT((
	//	mpl::equal<Test2, mpl::vector_c<bool, false, true, false> >::value));
	typedef mpl::vector_c<int,1,1,2,3,5,8> fibonacci;
	typedef typename mpl::push_back<fibonacci, mpl::int_<13> >::type fibonacci2;
	BOOST_MPL_ASSERT_RELATION(mpl::front<fibonacci2>::type::value, == , 1);
	BOOST_MPL_ASSERT_RELATION(mpl::back<fibonacci2>::type::value, ==, 13);	

	//BOOST_STATIC_ASSERT((
	//	mpl::equal<fibonacci2, mpl::vector_c<int,1,1,2,3,5,8,13> >::value));
	if(mpl::size<fibonacci2>::value == mpl::size< mpl::vector_c<int,1,1,2,3,5,8,13> >::value){
		cout<<"haha size equal!"<<endl;
	}
	typedef mpl::vector_c<int,1,2> t1;
	typedef mpl::vector<mpl::int_<1>, mpl::int_<2> > t2;
	typedef typename mpl::push_back<Empty, mpl::int_<1> >::type tt1;
	typedef typename mpl::push_back<tt1, mpl::int_<2> >::type tt2;
	if(mpl::equal<tt2, t2>::value)
	{
		cout<<"haha tt2 == t2!"<<endl;
	}
	if(mpl::equal<tt2, t1>::value)
	{
		cout<<"haha tt2 == t1!"<<endl;
	}
	// BOOST_STATIC_ASSERT((mpl::equal<t1,t2>::value));
	
	// BOOST_STATIC_ASSERT(( mpl::equal< double_first_half<InType>::type, 
	//		ExpectedType>::value ));
	return 0;
}
*/
