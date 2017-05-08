/*
	单调队列的应用,	
	1 求最大子数组([s,t))累加和: 
		max(sum[i] - sum[i-k]), k >=s && k<t; 
		= sum[i] - min(sum[i-k]), 
		实际就变成了计算 min(sum[i-k]) 的最小值，
		而这实际上就等于计算，最近k个值得最值问题，见 max_in_k.cpp
	2 求最大子数组([s,t)) 平均值:
		先求出 数组的min与max，然后利用二分查找，low = min, high = max, middle 
		构造 array' = array - middle; 
		if(array' 的最大子数组累加和 >=0) 说明 array 的最大平均值 >= middle, low = middle;
		else(... <0) 说明 array 的最大平均值 < middle, high = middle;
		直到 low >= high || low+1==high
*/

#include <iostream>
#include <vector>
#include <limits>
#include <deque>

using namespace std;
extern void display(vector<int>&, string);
extern void display_range(vector<int>&, string, int from, int to);

void max_in_k_with_shifts(vector<int>& input, vector<int>& out, int k, int shift)
{
	deque<int> tmp;
	deque<int> index;
	out.resize(shift, -1);
	int cur_max;
	for(int i=shift; i<input.size(); i++)
	{
		int cur_v = input[i-shift];
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

void min_in_k_with_shifts(vector<int>& input, vector<int>& out, vector<int>& index_v, int k, int shift)
{
	deque<int> tmp;
	deque<int> index;
	out.resize(shift, 0);
	index_v.resize(shift, 0);
	int cur_min;
	int cur_index;
	for(int i=shift; i<input.size(); i++)
	{
		int cur_v = input[i-shift];
		if(tmp.empty())
		{
			cur_min = cur_v;
			cur_index = i-shift;
		}else
		{
			while(index.front() + k <= i-shift)
			{
				tmp.pop_front(); 
				index.pop_front();
			}
			while(!tmp.empty())
			{
				if(tmp.back() >= cur_v) 
				{
					tmp.pop_back();
					index.pop_back();
				}
				else{
					break;
				}
			}
			if(tmp.empty()) {cur_min = cur_v; cur_index = i-shift; }
			else { 
				cur_min = tmp.front();
				cur_index = index.front();
			}
		}
		tmp.push_back(cur_v);
		index.push_back(i-shift);
		out.push_back(cur_min);
		index_v.push_back(cur_index);
	}	
}

void cal_sum(vector<int> v, vector<int>& s)
{
	s.resize(v.size());
	int cur_sum = 0;
	for(int i=0;i<v.size();i++)
	{
		s[i] = cur_sum + v[i];
		cur_sum = s[i];
	}
}

int max_sum_sub_array_range(vector<int> v, int s, int t, int& index_s, int& index_t)
{
	vector<int> sum;
	cal_sum(v, sum);
	// display(sum, "sum:");
	vector<int> k_min, k_index;
	min_in_k_with_shifts(sum, k_min, k_index, t, s);

	// display(k_min, "min:");
	// display(k_index, "min_index:");

	int max_ret = std::numeric_limits<int>::min();
	
	if(s>0) 
	{
		max_ret = sum[s-1]; 
		index_s = 0;
		index_t = s-1;
	}

	for(int i=s; i<v.size(); i++)
	{
		int max_cur = sum[i] - k_min[i];
		if( max_cur > max_ret) 
		{
			max_ret = max_cur;
			index_s = k_index[i] + 1;
			index_t = i;
			if(i<t) 
			{
				if(max_ret < sum[i])
				{
					index_s = 0;
					max_ret = sum[i];
				}
			}
		}
	}
	return max_ret;
}

int max_sum_sub_array(vector<int> v, int& index_s, int& index_t)
{
	int max_ret = std::numeric_limits<int>::min(); 
	int cur_max = std::numeric_limits<int>::min();

	int cur_v;
	int cur_sum = 0;
	int last_sum = 0;
	int s,t,i;

	for(s=0, t=0, i=0; i<v.size(); i++)
	{
		cur_v = v[i];
		cur_sum += cur_v;
		if(cur_sum < cur_v) // current value is bigger
		{
			s = i; 
			cur_sum = cur_v;
		}
		if(cur_sum > max_ret)
		{
			max_ret = cur_sum;
			t = i;
		}
	}
	index_s = s;
	index_t = t;
	return max_ret;
}

int get_max(vector<int>& v)
{
	int max_ret = std::numeric_limits<int>::min();
	for(int i=0; i<v.size(); i++)
	{
		if(v[i]>max_ret) max_ret = v[i];
	}
	return max_ret;
}

int get_min(vector<int>& v)
{
	int min_ret = std::numeric_limits<int>::max();
	for(int i=0; i<v.size(); i++)
	{
		if(min_ret>v[i]) min_ret=v[i];
	}
	return min_ret;
}

void array_subtract(vector<int>& input, vector<int>& out, int key)
{
	out.resize(input.size());
	for(int i=0; i<input.size(); i++)
		out[i] = input[i] - key;
	return ;
}

int max_mean_sub_array(vector<int>& v, int s, int t, int& index_s, int& index_t)
{
	int low = get_min(v);
	int high = get_max(v);
	int middle;
	vector<int> tmp;
	while(1)
	{
		middle = low + (high - low)/2;
		if(low>= high) break;
		if(low + 1 == high) 
		{
			array_subtract(v, tmp, high);
			int tmps, tmpt;
			if(max_sum_sub_array_range(tmp, s, t, tmps, tmpt) >= 0) 
			{
				middle = high; 
				index_s = tmps;
				index_t = tmpt;
				break;
			}
			array_subtract(v, tmp, low);
			if(max_sum_sub_array_range(tmp, s, t, tmps, tmpt) >= 0)
			{
				middle = low;
				index_s = tmps;
				index_t = tmpt;
			}
			break;
		}
		
		array_subtract(v, tmp, middle);
		int max_v = max_sum_sub_array_range(tmp, s,t, index_s, index_t);
		if( max_v >=0)
		{
			low = middle ;
		}else{
			high = middle;
		}
	}
	return middle;
}
