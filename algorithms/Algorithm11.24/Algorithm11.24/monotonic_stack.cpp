#include <vector>
#include <stack>
#include <limits>
#include <string>
using namespace std;

extern void display(vector<int>&, string);

// 每次入栈，左边界能确定
// 每次出栈，右边界能确定
int find_max_square(vector<int> v)
{
	if(v.size()<=0) return -1; 
	stack<int> st;
	stack<int> index;
	int cur_v;
	int length;
	int max_ret = std::numeric_limits<int>::min();

	display(v, "input");

	vector<int> tmpv = v;
	tmpv.push_back(-1);
	vector<int> left_b(tmpv.size());
	vector<int> rigt_b(tmpv.size());
	vector<int> square(tmpv.size());
	
	st.push(v[0]);
	left_b[0] = 0;
	index.push(0);

	for(int i=1; i<tmpv.size(); i++)
	{
		cur_v = tmpv[i];
		int ind = i;//index.top();
		while(!st.empty() && st.top() > cur_v)
		{
			st.pop();
			ind = index.top();
			index.pop();
			rigt_b[ind] = i;
			square[ind] = tmpv[ind]*(rigt_b[ind] - left_b[ind]);
			if(square[ind] > max_ret) max_ret = square[ind];
		}
		left_b[i] = ind;//+1;
		if(!st.empty() && st.top() == cur_v) left_b[i] = left_b[ind];
		index.push(i);
		st.push(cur_v);
	}
	display(v, "input");
	display(left_b, "left_boundary");
	display(rigt_b, "right_boundary");
	display(square, "square value");
	return max_ret;
}