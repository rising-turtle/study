
#include <iostream>
#include <boost/signals2.hpp>

using namespace std;
void test1();
void test2();
void test3();
void test4();

int main(int argc, char* argv[])
{
  test2();
  return 1;
}

void test4()
{
  //////////////////////////////////////////
  //
  //
  //  Automatic Connection Management : disconnect signal when the objects involved 
  /*  are destroyed 
   *  
   *  typedef boost::signals2::signal<void (const NewsItem&)> signal_type; 
   *  signal_type deliverNews;
   *   
   *   struct NewsMessageArea : public MessageArea
      {
        public:
          // ...

          void displayNews(const NewsItem& news) const
      {
          messageText = news.text();
          update();
       }     
      };
   *
   * change from 
    // ...
      NewsMessageArea *newsMessageArea = new NewsMessageArea();
    // ...
   * deliverNews.connect(boost::bind(&NewsMessageArea::displayNews,
   *  newsMessageArea, _1));
   *  
   *  to 
   *  // ...
      boost::shared_ptr<NewsMessageArea> newsMessageArea(new NewsMessageArea( ... ));
    // ...
      deliverNews.connect(signal_type::slot_type(&NewsMessageArea::displayNews,
      newsMessageArea.get(), _1).track(newsMessageArea));
   *
   * */

}



void test3()
{
/*
 * Disconnecting Slots 
 *  
 *  boost::signals2::connection c = sig.connect(HelloWorld());
  std::cout << "c is connected\n";
  sig(); // Prints "Hello, World!"

  c.disconnect(); // Disconnect the HelloWorld object
  std::cout << "c is disconnected\n";
  sig(); // Does nothing: there are no connected slots
 * 
 *   sig.connect(&foo);
  sig.connect(&bar);
  sig();

  // disconnects foo, but not bar
  sig.disconnect(&foo);
  sig();
 *
 *  Blocking Slots 
 *  
 *  boost::signals2::connection c = sig.connect(HelloWorld());
  std::cout << "c is not blocked.\n";
  sig(); // Prints "Hello, World!"

  {
    boost::signals2::shared_connection_block block(c); // block the slot
    std::cout << "c is blocked.\n";
    sig(); // No output: the slot is blocked
  } // shared_connection_block going out of scope unblocks the slot
  std::cout << "c is not blocked.\n";
  sig(); // Prints "Hello, World!"}
 *
 *  Scoped Connections 
 *    {
 boost::signals2::scoped_connection c(sig.connect(ShortLived()));
 sig(); // will call ShortLived function object
 } // scoped_connection goes out of scope and disconnects

 sig(); // ShortLived function object no longer connected to sig
 *  
 *  // doesn't compile due to compiler attempting to copy a temporary scoped_connection object
// boost::signals2::scoped_connection c0 = sig.connect(ShortLived());

// okay
boost::signals2::scoped_connection c1(sig.connect(ShortLived()));

// also okay
boost::signals2::scoped_connection c2;
c2 = sig.connect(ShortLived());
 *  
 *
 *
 * */
}



//////////////////////////////////////////////
//
//  test2 show me how to call the sigs with parameters, 
//  and how to collect, operate all the results from 
//  the slots
//
//////////////////////////////////////////////


float sum1(float x, float y)
{
  cout<<"sum: x+y = "<<x+y<<endl;
  return x+y; 
}

float minus1(float x, float y)
{
  cout<<"minus: x-y = "<<x-y<<endl;
  return x-y;
}

float product1(float x, float y)
{
  cout<<"product: x*y = "<<x*y<<endl;
  return x*y;
}

template<typename T>
struct maximum
{
  typedef T result_type; 
  template<typename InputInterator>
    T operator()(InputInterator first, InputInterator last) const 
    {
      // if there is no slots to call, just return the 
      // default-constrcuted value 
      if(last == first) return T();
      T max_value = *first++;
      while(first != last)
      {
        if(*first > max_value)
        {
          max_value = *first; 
        }
        ++first;
      }
      return max_value;
    }
};

template<typename Container>
struct aggregate
{
  typedef Container result_type;
  template<typename InputInterator>
    Container operator()(InputInterator first, InputInterator last) const
    {
      if(first == last) return Container();
      Container con; 
      while(first != last)
      {
        con.push_back(*first++);
      }
      return con;
    }
};

void test2()
{
  // boost::signals2::signal<float (float, float), maximum<float> > sig; 
  boost::signals2::signal<float (float, float), aggregate<vector<float> > > sig;
  sig.connect(&sum1);
  sig.connect(&minus1);
  sig.connect(&product1);
  // cout<<*sig(3, 5)<<endl; // only return the result of the last signal
  // cout<<sig(3,-5)<<endl;
  vector<float> results = sig(3,-5); 
  std::copy(results.begin(), results.end(),
    std::ostream_iterator<float>(std::cout, " "));
}


/////////////////////////////////////////
//  
//  test1 show me, how to input to the slots
//  and how to call the slots, as a group 
//  sequence, and non-group sequence,
//  following the first part in 
//  http://www.boost.org/doc/libs/1_56_0/doc/html/signals2/tutorial.html#idp428240080
//
////////////////////////////////////////
struct Hello
{
  void operator()()
  {
    cout<<"Hello ";
  }
};

void World()
{
  cout<<", World"<<endl;
}

struct Morning
{
  void operator()()
  {
    cout<<" ... and good morning!"<<endl;
  }
};

void test1()
{
  boost::signals2::signal<void ()> sig;
  sig.connect(Morning());
  sig.connect(1, &World); 
  sig.connect(0, Hello());
  sig();
}


