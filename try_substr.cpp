#include <string>
#include <iostream>

using namespace std; 

int main()
{
  {
  string s = "1413393924805760512.png"; 
  
  string nsec = s.substr(s.size() - 13, 9); 
  string sec = s.substr(0, s.size()-13); 

  cout <<"s.size() = "<<s.size()<<" nsec = "<<nsec<<" sec = "<<sec<<endl;
  }

 {
  string s = "1496343231.291172784.png"; 
  
  string nsec = s.substr(s.size() - 13, 9); 
  string sec = s.substr(0, s.size()-14); 

  cout <<"s.size() = "<<s.size()<<" nsec = "<<nsec<<" sec = "<<sec<<endl;
  }

  return 0; 
}
