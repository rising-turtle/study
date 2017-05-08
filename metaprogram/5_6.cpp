#include "header.h"

// execise 5_5
// Write a sequence adapter template called dimensions that, when instantiated on an array type, presents the array's dimensions as a forward, non-extensible sequence:
//

template<typename T>
struct test{};

template<>
struct test<char>
{
    const char* operator()()
    {
        return " char";
    }
};

template<typename T>
struct test<T[]>
{
    const char* operator()()
    {
        info = " array of";
        info += test<T>()();
        return info.c_str();
    } 
    string info;
};
template<typename T, int N>
struct test<T[N]>
{
    const char* operator()()
    {
        info = " array of";
        info += test<T>()();
        info += " with size ";
        info += N-0+'0';
        return info.c_str();
    }
    string info;
};

template<typename T, typename V>
struct gsize_iterator{
    const static int value = 0;
    typedef V type;
};

template<typename T, typename V>
struct gsize_iterator<T[],V>
{
    const static int value = gsize_iterator<T,V>::value + 1;
    typedef typename mpl::push_front<typename gsize_iterator<T,V>::type, mpl::int_<0> >::type type;
};

template<typename T, int N, typename V>
struct gsize_iterator<T[N],V>
{
    const static int value = gsize_iterator<T,V>::value + 1;
    typedef typename mpl::push_front<typename gsize_iterator<T,V>::type, mpl::int_<N> >::type type;
};

template<typename Seq, typename Pos>
struct dimensions_iterator
{
    typedef mpl::forward_iterator_tag category;
};

struct dimensions_tag{};

template<typename T>
struct dimensions{
    typedef mpl::vector<> S;
    const static int value = gsize_iterator<T,S>::value;
    typedef dimensions_tag tag;
    typedef typename gsize_iterator<T,S>::type type;
};

namespace boost{ namespace mpl{
    template<>
        struct begin_impl<dimensions_tag>
        {
            template<typename dimensions>
                struct apply{
                    typedef dimensions_iterator<dimensions, int_<0> > type;
                };
        };
   /* template<>
        struct end_impl<dimensions_tag>
        {
            template<typename Seq, typename Pos>
                struct apply{
                    
                    typedef dimensions_iterator<dimensions, >
                };
        };
    */
}}

int main()
{
    if(boost::is_array<char [10]>::type::value)
    {
        cout<<"is_array works!"<<endl;
    }
    // cout<<test<char[][5][6]>()()<<endl;
    // cout<<(gsize_iterator<char[][4][2]>::value)<<endl;
    // cout<<(gsize<char[][4][2]>::value)<<endl;

    typedef typename dimensions<char[][4][2]>::type out1;
    if(mpl::size<out1>::value == 3)
    {
        cout<<"succeed 1"<<endl;
    }
    if(mpl::at_c<out1,0>::type::value == 0 && mpl::at_c<out1,1>::type::value == 4 && \
            mpl::at_c<out1,2>::type::value ==2 )
    {
        cout<<"succeed 2"<<endl;
    }

    /*typedef typename mpl::vector<> t1;
    typedef typename mpl::push_back<t1, mpl::int_<2> >::type t2;
    typedef typename mpl::push_back<t2, mpl::int_<4> >::type t3;
    typedef typename mpl::push_back<t3, mpl::int_<0> >::type t4;

    typedef mpl::vector<mpl::int_<1>, mpl::int_<2>, mpl::int_<3> > expt2;
    typedef mpl::vector<mpl::int_<0>, mpl::int_<4>, mpl::int_<2> > expt1;

    if(mpl::equal<out1, expt1>::value)
    {
        cout<<"succeed 1 ! "<<endl;
    }
    if(mpl::equal<t4, expt1>::value)
    {
        cout<<"succeed 2 !"<<endl;
    }*/

    /*typedef dimensions<char[10]>::type seq;
    if(seq::value == 10 )
    {
        cout<<"succeed!"<<endl;
    }*/
}
