#include "header.h"

// tree definition
template<typename O, typename L, typename R>
struct tree
{
    typedef O Root;
    typedef L Left;
    typedef R Right;
    typedef tree type;
};

// whether is a leaf
template<typename T>
struct is_leaf : mpl::true_{};
template<typename O, typename L, typename R>
struct is_leaf<tree<O,L,R> >: mpl::false_{};

// select_t function
struct vlhs{};
struct vtop{};
struct vrhs{};

template<typename T, typename Which>
struct select_t{
    BOOST_STATIC_ASSERT((
                mpl::or_<boost::is_same<Which, vlhs>,
                        boost::is_same<Which, vtop>,
                        boost::is_same<Which, vrhs> >::value
                ));
    typedef typename mpl::if_<boost::is_same<Which, vlhs>,
                        typename T::Left, 
                        typename mpl::if_<boost::is_same<Which, vtop>,
                        typename T::Root,
                        typename T::Right>::type
                        >::type type;
};

// end inorder
struct inorder_end_iterator
{
    typedef mpl::forward_iterator_tag category;
    typedef inorder_end_iterator type;
};

template<typename T, typename Chain, typename Which>
struct inorder_iterator;

// begin inorder 
template<typename T, typename Chain = inorder_end_iterator,  typename Which = vlhs>
struct get_begin_inorder
{
    typedef typename select_t<T,Which>::type branchType;
    typedef typename mpl::eval_if<is_leaf<branchType>,
                            mpl::identity<inorder_iterator<T, Chain, Which> >,
                            get_begin_inorder<branchType, 
                                    typename mpl::if_<boost::is_same<Which, vlhs>,
                                    inorder_iterator<T, Chain, vtop>,
                                    Chain>::type
                                    >
                    >::type type;
};

// inorder iterator
struct inorder_tag{};
template<typename T, typename Chain, typename Which>
struct inorder_iterator : select_t<T, Which>
{
    typedef mpl::forward_iterator_tag category;
    // typedef typename select_t<T,vrhs>::type rightBranch;
    typedef typename mpl::if_< boost::is_same<Which, vrhs>,
            Chain,
            typename mpl::if_< boost::is_same<Which, vlhs>,
                        inorder_iterator<T, Chain, vtop>,
                        typename get_begin_inorder<T, Chain, vrhs>::type 
                            >::type 
                        >::type next;
    /*typedef typename mpl::if_<is_leaf<T>,
            Chain, 
            get_begin_inorder<rightBranch, 
                    Chain>
            >::type next;
            */
};

template<typename T>
struct inorder_view
{
    typedef T tree_type;
    typedef inorder_tag tag;
};

template<typename T, typename Seq, typename OnlyRoot = mpl::false_>
struct get_inorder_sequence
{
   typedef typename mpl::eval_if<is_leaf<typename T::Left>,
                    mpl::push_back<Seq,typename T::Left>,
                    get_inorder_sequence<typename T::Left, Seq>
                >::type L_Seq;
   typedef typename mpl::push_back<L_Seq, typename T::Root>::type O_Seq;
   typedef typename mpl::eval_if<is_leaf<typename T::Right>,
                    mpl::push_back<O_Seq, typename T::Right>,
                    get_inorder_sequence<typename T::Right, O_Seq> 
               >::type type;
};

template<typename T, typename Seq>
struct get_inorder_sequence<T, Seq, mpl::true_>
{
    typedef typename mpl::push_back<Seq, typename T::Root>::type type;
};

template<typename T>
struct inorder_view2 : get_inorder_sequence<T, mpl::vector<>, is_leaf<T> >::type
{};


template<typename T, typename Seq, typename OnlyRoot = mpl::false_>
struct get_preorder_seq{
    typedef typename mpl::push_back<Seq, typename T::Root>::type O_Seq;
    typedef typename mpl::eval_if<is_leaf<typename T::Left>,
            mpl::push_back<O_Seq, typename T::Left>,
            get_preorder_seq<typename T::Left, O_Seq> 
                >::type L_Seq;
    typedef typename mpl::eval_if<is_leaf<typename T::Right>,
            mpl::push_back<L_Seq, typename T::Right>,
            get_preorder_seq<typename T::Right, L_Seq> 
                >::type type;
};

template<typename T, typename Seq>
struct get_preorder_seq<T, Seq, mpl::true_>
{
    typedef typename mpl::push_back<Seq, typename T::Root>::type type;
};

template<typename T>
struct preorder_view : get_preorder_seq<T, mpl::vector<>, is_leaf<T> >::type
{};

template<typename T, typename Seq, typename OnlyRoot = mpl::false_>
struct get_posorder_seq{
    typedef typename mpl::eval_if<is_leaf<typename T::Left>,
            mpl::push_back<Seq,typename T::Left>,
            get_posorder_seq<typename T::Left, Seq>
                >::type L_Seq;
    typedef typename mpl::eval_if<is_leaf<typename T::Right>,
            mpl::push_back<L_Seq, typename T::Right>,
            get_posorder_seq<typename T::Right, L_Seq> 
                >::type R_Seq;
    typedef typename mpl::push_back<R_Seq, typename T::Root>::type type;
};

template<typename T, typename Seq>
struct get_posorder_seq<T,Seq, mpl::true_>{
    typedef typename mpl::push_back<Seq, typename T::Root>::type type;
};

template<typename T>
struct posorder_view : get_posorder_seq<T, mpl::vector<>, is_leaf<T> >::type
{};


// insert BST tree
template<typename T, typename Left = mpl::true_>
struct select_tt{
    typedef typename T::Left type;
};

template<typename T>
struct select_tt<T, mpl::false_>{
    typedef typename T::Right type;
};

template<typename T, typename K, typename Leaf = mpl::false_>
struct tree_insert{
    typedef typename mpl::eval_if<is_leaf<T>,
            typename mpl::if_<mpl::greater<typename T::Root, K>,
                            K > >
};

template<typename T, typename K>
struct tree_insert<T,K,mpl::true_>{
    typedef K t
};

/*
template<typename T, typename Seq>
struct get_inorder_sequence<T,Seq, mpl::false_>
{
    typedef typename get_inorder_sequence<typename T::Left, Seq, is_leaf<typename T::Left> >::type L_Seq;
    typedef typename mpl::push_back<L_Seq, typename T::Root>::type O_Seq;
    typedef typename get_inorder_sequence<typename T::Right, O_Seq, is_leaf<typename T::Right> >::type type;
};
*/
/*
template<typename T, typename Seq = mpl::vector<>, typename Pos= mpl::int_<0>, typename Leaf = mpl::true_ >
struct inorder2_iterator : 
{
    typedef mpl::forward_iterator_tag tag;
    
    typename mpl::if_<mpl::is_empty<Seq>,
             >
    
    typedef typename mpl::push_back<Seq, typename T::Root>::type SeqT;
    typename typename mpl::at<SeqT, Pos>::type type;
    typename inorder2_iterator< typename T::Right ,SeqT, typename mpl::plus<Pos, mpl::int_<1> >, is_leaf<T> > next;

    typedef typename mpl::if_< 
        is_leaf<T>,
        typename mpl::push_back<Seq, typename T::Root>::type,
            
        >::type type;
};

template<typename T, typename Seq = mpl::vector<> , typename Pos = mpl::int_<0> >
struct inorder2_iterator<T,Seq,Pos, mpl::false_>:
{
    
};
*/
namespace boost{ namespace mpl{
   template<>
        struct end_impl<inorder_tag>
        {
            template<typename T>
                struct apply : mpl::identity<inorder_end_iterator>{};
        };

    template<>
    struct begin_impl<inorder_tag>
    {
        template<typename T>
        struct apply : get_begin_inorder<typename T::tree_type, 
            typename mpl::end<T>::type, vlhs > {};
    };
 }}
/*
template<typename T>
struct preorder_view : mpl::vector<typename T::Root, preorder_view<typename T::Left>, preorder_view<typename T::Right> >
{};
template<>
struct preorder_view<nono> : mpl::vector<>{};

template<typename T>
struct preorder_view<tree<T, mpl::vector<>, mpl::vector<> > > : mpl::identity<typename T::Root>::type{}; 
*/


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

template<typename T>
struct test_iterator
{
    typedef test_iterator<typename mpl::next<T>::type > next;
    typedef mpl::forward_iterator_tag category;
    typedef typename mpl::deref<T>::type type;
};


int main()
{
     // cout<<(begin_word_iterator::type::value::value)<<endl;
 /*   typedef tree<double, 
            tree<void*,int,long>,
            char> tree_seq;
    typedef preorder_view<tree_seq> Out;
    typedef mpl::vector<double, void*, int, long, char> Exp1;
    if(boost::is_same<Out,Exp1>::value)
    {
        cout<<"succeed!!!"<<endl;
    }
    */
    typedef tree<double, char, void*> left_seq;
    typedef tree<float, left_seq, void*> tree_seq;
    typedef inorder_view2<tree_seq> Out;
    typedef preorder_view<tree_seq> Out3;
    typedef posorder_view<tree_seq> Out5;
    // typedef typename get_inorder_sequence<tree_seq, mpl::vector<>, is_leaf<tree_seq> >::type Out;
    typedef mpl::transform<Out, warp<_>, mpl::back_inserter<mpl::vector<> > >::type Out2;
    typedef mpl::transform<Out3, warp<_>, mpl::back_inserter<mpl::vector<> > >::type Out4;
    typedef mpl::transform<Out5, warp<_>, mpl::back_inserter<mpl::vector<> > >::type Out6;

    mpl::for_each<Out2>(print_t());
    cout<<endl;
    mpl::for_each<Out4>(print_t());
    cout<<endl;
    mpl::for_each<Out6>(print_t());
    cout<<endl;

    /*
    typedef mpl::vector_c<int,1,2> f1;
    typedef typename mpl::transform<f1, mpl::plus<_1, mpl::int_<1> > >::type f2;
    typedef mpl::begin<f1>::type f1_iter1;
    cout<<mpl::deref<f1_iter1>::type::value<<endl;
    typedef mpl::plus<f1_iter1::type, mpl::int_< 1> >::type p1;
    cout<<mpl::deref<p1>::type::value<<endl;
    if(mpl::equal<f2 ,mpl::vector_c<int,2,3 > >::value)
    {
        cout<<"transform act!"<<endl;
    }*/

    /*typedef mpl::vector_c<int, 3,4> seq;
    typedef typename mpl::begin<seq>::type seq_iter;
 
    cout<<(mpl::deref<mpl::next<seq_iter>::type >::type::value)<<endl;
    typedef mpl::range_c<int, 0, 10> seq1; 
    typedef mpl::begin<seq1>::type aux_base_iter;
    typedef test_iterator<aux_base_iter> base_iter;
    cout<<(base_iter::type::value)<<endl;
    cout<<(base_iter::next::type::value)<<endl;

    typedef typename mpl::identity<test_iterator<mpl::int_<3> > >::type Out2;
    if(boost::is_same<typename Out2::next, test_iterator<mpl::int_<4> > >::value)
    {
        cout<<"test succeed!"<<endl;
    }*/

    return 0;
}

