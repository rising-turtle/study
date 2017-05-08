#include <stdio.h>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <string>

using namespace std;

extern void max_in_k(vector<int>& ,vector<int>&, int k);
extern void max_in_k_with_shifts(vector<int>& input, vector<int>& out, int k, int shift);
extern int max_sum_sub_array(vector<int>, int&, int&);
extern int max_sum_sub_array_range(vector<int> v, int s, int t, int& index_s, int& index_t);
extern int max_mean_sub_array(vector<int>& v, int s, int t, int& index_s, int& index_t);
extern int find_max_square(vector<int>);
extern void scan(vector<int>& iv);

void randv(vector<int>& v, int n, int range, int shift = 0);
void display(vector<int>& v, string tag="");
void test_scan();

int main()
{
	char c; 
	/*while(cin>>c)
	{
		if(c == 'c') break;
	vector<int> input, output, output2;
	randv(input, 5, 10, 0);
	}*/
	/* 
	max_in_k.cpp
	max_in_k(input, output, 4);
	display(input, "input");
	display(output, "output with k=4");
	max_in_k_with_shifts(input, output2, 4, 2);
	display(output2, "output with k=4 shift=2");
	*/
	
	// max_sum_sub_array.cpp
	// int is, it; 
	// display(input, "input");
	// int max_v = max_sum_sub_array_range(input, 2, 5, is, it);
	// int max_v = max_mean_sub_array(input, 3, 5, is, it);
	// cout<<"max_mean_value: "<<max_v<<" from "<<is<<" to "<<it<<endl;
	// display_range(input, "max_range: ", is, it);

	/*
		monotonic_stack.cpp
	
	int max_v = find_max_square(input);
	cout<<"max_v: "<<max_v<<endl;
	}*/

	// test_scan();

	return 0;
}

void display(vector<int>& v, string tag)
{
	if(tag != "")
	{
		cout<<tag<<endl;
	}
	for(int i=0; i<v.size(); i++)
	{
		cout<<v[i]<<"\t";
	}
	cout<<endl;
}

void display_range(vector<int>& v, string tag, int from, int to)
{
	if(tag != "")
	{
		cout<<tag<<endl;
	}
	if(to>= v.size()) to = v.size() -1;
	for(int i=from; i<=to; i++)
	{
		cout<<v[i]<<" ";
	}
	cout<<endl;
}

void randv(vector<int>& v, int n, int range, int shift)
{
	v.resize(n, 0);
	srand((unsigned int)time(0));
	for(int i=0; i<n; i++)
	{
		v[i] = rand()%range - shift;
	}
}
