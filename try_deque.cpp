#include <deque>
#include <iostream>

using namespace std; 


int main()
{

  deque<double> q; 
  q.push_back(4);
  q.push_back(3);

  cout <<"q.size = "<<q.size()<<endl;

  q.push_back(3); 
  cout <<"q.size = "<<q.size()<<endl;

  return 0; 
}
