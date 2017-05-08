/*
 * David Z, 
 * erase an item in map structure
 *
 * */

#include <map>
#include <string>

#include <iostream>

using namespace std;

int main()
{
  map<int, string> data;
  data[1] = "first"; 
  data.erase(1);
  data[2] = "second";

  for(map<int, string>::iterator it = data.begin(); it!=data.end(); ++it )
  {
    cout<<it->first<<" : "<<it->second<<endl;
  }

  return 1;
}
