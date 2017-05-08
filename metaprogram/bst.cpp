#include "header.h"

struct nono{};

// tree node

template<typename T>
struct tree_node;

// tree definition
template<typename O = nono, typename L =nono, typename R = nono>
struct tree
{
    typedef O Root;
    typedef typename mpl::if_<boost::is_same<L, nono>, 
                nono, typename tree_node<L>::type>::type Left;
    typedef typename mpl::if_<boost::is_same<R, nono>,
                nono, typename tree_node<R>::type>::type Right;
    typedef tree type;
};

template<typename T>
struct tree_node
{
    typedef tree<T, nono, nono> type;    
};

template<typename O, typename L , typename R>
struct tree_node< tree<O,L,R> >
{
    typedef tree<O,L,R> type;
};

// whether is a leaf
template<typename T>
struct is_leaf : mpl::true_{};
template<>
struct is_leaf<tree<nono,nono,nono> > : mpl::true_{};
template<typename O, typename L, typename R>
struct is_leaf<tree<O,L,R> >:  mpl::false_{};


template<typename T, typename Seq, bool OnlyRoot = false>
struct get_inorder_sequence
{
   typedef typename mpl::eval_if<is_leaf<typename T::Left>,
                    mpl::identity<Seq>,
                    get_inorder_sequence<typename T::Left, Seq>
                >::type L_Seq;
   typedef typename mpl::push_back<L_Seq, typename T::Root>::type O_Seq;
   typedef typename mpl::eval_if<is_leaf<typename T::Right>,
                    mpl::identity<O_Seq>, 
                    get_inorder_sequence<typename T::Right, O_Seq> 
               >::type type;
};

template<typename T, typename Seq>
struct get_inorder_sequence<T, Seq, true>
{
    typedef typename mpl::push_back<Seq, typename T::Root>::type type;
};

template<typename T>
struct inorder_view : get_inorder_sequence<T, mpl::vector<>, is_leaf<T>::value >::type
{};


template<typename T, typename K>
struct bin_inserter
{
    // typedef typename mpl::greater_equal<typename T::Root, K>::type branchSelect;
    typedef typename mpl::if_<mpl::less<typename T::Root, K >,
            typename T::Left,
            typename mpl::eval_if<is_leaf<typename T::Left>,
                            mpl::identity< tree<K> >,
                            bin_inserter<typename T::Left,K>
                                >::type
            >::type Left;
    typedef typename mpl::if_<mpl::greater_equal<typename T::Root, K>,
            typename T::Right,
            typename mpl::eval_if<is_leaf<typename T::Right>,
                            mpl::identity<tree<K> >,
                            bin_inserter<typename T::Right,K>
                                >::type
            >::type Right;
    typedef tree<typename T::Root, Left, Right> type;
};

template<typename K>
struct bin_inserter<tree<>,K>
{
    typedef tree<K> type;
};
/*
template<typename K, bool IsRoot >
struct bin_inserter<tree<>, K, IsRoot >
{
    typedef tree<K> type;
};
*/
struct binary_tree_inserter_op
{
    template<typename State, typename Ele>
        struct apply : bin_inserter<State, Ele>::type{};
};

template<typename T>
struct binary_tree_inserter : mpl::inserter<T, binary_tree_inserter_op >
{};

template<typename T>
struct warp
{
    // typedef typename mpl::deref<T>::type type;
};

struct print_t
{
    template<typename T>
    void operator()(warp<T>)
    {
        cout<<typeid(T).name()<<"\t";
    }
};

struct print_v
{
    template<typename T>
    void operator()(warp<T>)
    {
        cout<<T::value<<"\t";
    }    
};

int main()
{
    typedef tree<char, double, void*> left_t;
    typedef tree<int, float> right_t;
    typedef tree<void, left_t, right_t> tree_seq;

    typedef inorder_view<tree_seq> Out1;
    typedef typename mpl::transform<Out1, warp<_>, mpl::back_inserter<mpl::vector<> > >::type Out2;
    mpl::for_each<Out2>(print_t());
    cout<<endl;
    
    typedef tree<> baset;
    typedef typename bin_inserter<tree<>, mpl::int_<17> >::type t1;
    typedef typename bin_inserter<t1, mpl::int_<25> >::type t2;
    typedef typename bin_inserter<t2, mpl::int_<0> >::type t3;
    typedef typename bin_inserter<t3, mpl::int_<10> >::type t4;

    typedef typename mpl::transform<inorder_view<t4>, warp<_>, mpl::back_inserter<mpl::vector<> > >::type Out3;
    mpl::for_each<Out3>(print_v());
    cout<<endl;
    
    typedef typename mpl::copy<mpl::vector_c<int, 17, 25, 0 ,10>,
            binary_tree_inserter<tree<> > >::type Out4;

    typedef typename mpl::transform<inorder_view<Out4>, warp<_>, mpl::back_inserter<mpl::vector<> > >::type Out5;
    mpl::for_each<Out5>(print_v());
    cout<<endl;
   

    /*typedef mpl::copy<
            mpl::vector_c<int, 17, 25, 10, 2, 11>, 
            binary_tree_inserter< tree<> >
            >::type bst;*/
    return 0;
}



