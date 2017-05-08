/*
 * David Z, 
 * reimplement the sort algorithm using user defined compare function
 * 
 * */

#include <map>
#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>

using namespace std; 

int N = 10;

struct tmpD
{
  int index; 
  double v; 
  static bool inline compare(const tmpD& l, const tmpD& r)
  {
    return l.v < r.v;
  }
};

void construct(vector<tmpD>& v)
{
  for(int i=0; i<N ; i++)
  {
    tmpD t; 
    t.index = i;
    t.v = fabs(i-N/2);
    v.push_back(t);
  }
}

void display(vector<tmpD>& v)
{
  for(int i=0; i<v.size(); i++)
    cout<<v[i].v<<","<<v[i].index<<" ";
  cout<<endl;
}

int main()
{
  int N = 10; 
  vector<tmpD> c;
  construct(c); 
  display(c); 
  sort(c.begin(), c.end(), tmpD::compare); 
  display(c);
  return 0;
}



