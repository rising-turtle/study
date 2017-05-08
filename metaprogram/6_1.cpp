#include "header.h"

struct myInit_{};

typedef struct myInit_ myInit;

template<typename T>
struct mysizeof{
    typedef mpl::sizeof_<T> value;
};

template<typename Seq>
struct smallest 
{
   typedef typename mpl::deref< typename mpl::copy<Seq, mpl::inserter< 
        myInit, 
        mpl::if_< mpl::or_<boost::is_empty<_1>, mpl::greater< typename mysizeof<_1>::value, typename mysizeof<_2>::value > >,
            _2,
            _1> > > 
    >::type type;
};


int main()
{
    typedef mpl::vector<float, char, int> In1;
    typedef smallest<In1>::type Ou1;
    if(mpl::sizeof_<Ou1>::value == mpl::sizeof_<char>::value)
    {
        cout<<"succeed char!"<<endl;
    }
 
    return 0;
}

