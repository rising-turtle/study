#include <iostream>
#include <vector>
#include <deque>

using namespace std;

void max_in_k(vector<int>& input, vector<int>& out, int k)
{
	deque<int> tmp;
	deque<int> index;
	int cur_max;
	for(int i=0; i<input.size(); i++)
	{
		int cur_v = input[i];
		if(tmp.empty())
		{
			cur_max = cur_v;
		}else
		{
			while(index.front() + k <= i)
			{
				tmp.pop_front(); 
				index.pop_front();
			}
			while(!tmp.empty())
			{
				if(tmp.back() <= cur_v) 
				{
					tmp.pop_back();
					index.pop_back();
				}
				else{
					break;
				}
			}
			if(tmp.empty()) {cur_max = cur_v; }
			else cur_max = tmp.front();
		}
		tmp.push_back(cur_v);
		index.push_back(i);
		out.push_back(cur_max);
	}	
}

void max_in_k_2(vector<int>& input, vector<int>& out, int k)
{
	
}


