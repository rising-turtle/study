#include <cmath>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <map>
#include <utility>
#define SQ(x) ((x)*(x))

using namespace std; 

/* 
 * problem: find the shorest square number sequence whose summation is N 
 * e.g. 12 = 4 + 4 + 4 is better than 9 + 1 + 1 + 1
 *
 * */

void print_v(vector<int> v); 
int greed_strategy(int N);
vector<int> decompose(int N);

int main(int argc, char* argv[])
{
  int N = 12; 
  if(argc >= 2)
  {
    N = atoi(argv[1]); 
  }
  print_v(decompose(N));
  // cout<<greed_strategy(N)<<endl;
  return 0; 
}

void next_v(vector<int>& v, int start, int end)
{
  v.resize(end-start +1); 
  for(int i=0; i<v.size(); i++)
  {
    v[i] = start+i;
  }
}

pair<bool, vector<int> > 
find_shortest_path(vector<int> cur_set, int rest, int h)
{
  // return variables
  vector<int> ret_v; 
  bool ret_b = false;
  
  // h is 0 
  if(h == 0)
  {
    return make_pair<bool, vector<int> >(ret_b, ret_v); 
  }

  // rest vector
  int m = cur_set.size();
  vector<int> rest_v(m);
  for(int i=0; i<m; i++)
  {
    // check for this level 
    int cur_p = cur_set[i]; 
    int rest_tmp = rest - SQ(cur_p); 
    if(rest_tmp == 0) // found it, it' the best solution 
    {
      ret_v.push_back(cur_p); 
      ret_b = true; 
      return make_pair<bool, vector<int> >(ret_b, ret_v);
    }
    rest_v[i] = rest_tmp; 
  }
  
  // next level loop, find the shortest 
  int largest = h+1;
  vector<int> next_best;
  int cur_item = -1;
  for(int i=0; i<m; i++)
  {
    if(rest_v[i]<0) // for these already fail
      continue;

    // construct next loop set 
    vector<int> next_set;
    next_v(next_set, cur_set[i], cur_set[m-1]);
    pair<bool, vector<int> > next_ret = find_shortest_path(next_set, rest_v[i], h-1); 
    if(next_ret.first) // find a solution 
    {
      if(largest > next_ret.second.size())
      {
        next_best = next_ret.second; 
        largest = next_best.size();
        cur_item = cur_set[i];
        ret_b = true;
      }
    }
  }
  // return the result of this level 
  if(cur_item != -1)
  {
    next_best.push_back(cur_item);
    ret_v = next_best;
  }
  return make_pair<bool, vector<int> >(ret_b, ret_v);
}

vector<int> decompose(int N)
{
  int height = greed_strategy(N); // the path of the greed_strategy 
  int max_sq_v = sqrt(N); 
  vector<int> ini_set;
  next_v(ini_set, 1, max_sq_v); 
  pair<bool, vector<int> > shortest_path = find_shortest_path(ini_set, N, height); 
  return shortest_path.second;
}

int greed_strategy(int N)
{
  int ret = 0; 
  int rest = N; 
  do{
    int max_square = sqrt(rest);
    rest -= SQ(max_square); 
    ++ret; 
  }while(rest>=1);
  return ret;
}


void print_v(vector<int> v)
{
  int sum = 0;
  for(int i=0; i<v.size(); i++)
  {
    cout<<SQ(v[i])<<" ";
    sum += SQ(v[i]);
  }
  cout<<" = "<<sum<<endl;
 //  cout<<endl;
}


