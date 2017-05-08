#include "header.h"

/*
template<typename T>
struct fiboinacci{};
*/

struct fibonacci_tag{};

template<int Pos>
struct fibonacci{
    // typedef mpl::forward_iterator_tag category;
    // typedef fibonacci<Pos+1> next;
    typedef fibonacci_tag tag;
    typedef typename mpl::plus<typename fibonacci<Pos-1>::num, mpl::int_<1> >::type num;
    typedef typename mpl::plus<typename fibonacci<Pos-1>::type, typename fibonacci<Pos-2>::type >::type type;
};


template<>
struct fibonacci<0>
{
    typedef mpl::int_<0> type;
    typedef fibonacci<1> next;
    typedef fibonacci_tag tag;
    typedef mpl::int_<0> num;
};

template<>
struct fibonacci<1>
{
    typedef mpl::int_<1> type;
    typedef fibonacci<2> next;
    typedef fibonacci_tag tag;
    typedef mpl::int_<1> num;
};

typedef fibonacci<0> fibonacci_series;

template<typename N>
struct fibonacci_iterator
{
    typedef mpl::forward_iterator_tag category;
    typedef fibonacci_iterator<typename mpl::next<N>::type> next;
};

namespace boost { namespace mpl{
    template<>
        struct size_impl<fibonacci_tag>{
            template<typename T>
            struct apply : T::num
            {};
        };
    
    template<>
        struct begin_impl<fibonacci_tag>{
            template<typename T>
            struct apply{
                typedef fibonacci_iterator<mpl::int_<0> > type;
            };
        };
    template<>
        struct end_impl<fibonacci_tag>{
            template<typename T>
                struct apply{
                    // typedef typename T::type type;
                    // typedef fibonacci_iterator<mpl::int_<30> > type;
                    typedef fibonacci_iterator<typename T::num> type;
                };
        };
    template<typename N>
        struct deref<fibonacci_iterator<N> > : fibonacci<N::value>{};  
}}
/*
template<typename T>
struct infinite : mpl::push_back<T, typename infinite<T::next>::type>::type
{
    typedef T type;
};
*/
typedef fibonacci<5> infinite_fibo; 

int main()
{
    // typedef mpl::lower_bound<fibonacci_series, mpl::int_<10> >::type n; 
    typedef typename mpl::lower_bound<infinite_fibo, mpl::int_<10> >::type n; 

    cout<<(mpl::size<fibonacci<8> >::type::value)<<endl;
    cout<<(mpl::deref<mpl::begin<fibonacci<8> >::type>::type::value)<<endl;
    cout<<(mpl::deref<mpl::end<fibonacci<8> >::type>::type::value)<<endl;
    cout<<(mpl::deref<n>::type::value)<<endl;
    return 0;
}
