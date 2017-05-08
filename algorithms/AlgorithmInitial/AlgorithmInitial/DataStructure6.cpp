#include "preheader.h"

void swap(int & a, int& b)
{
	b^=a^= b= a^b;
}
// 6.1.1
// each time discard the first card, and put the new one to the end
void Cards(int n)
{
	queue<int> cards;
	//cards.resize(n);
	for(int i=0;i<n;i++)
	{
		cards.push(i+1);
	}
	int index=0;
	while(cards.size()>=2)
	{
		cout<<" "<<cards.front();
		cards.pop();
		cards.push(cards.front());
		cards.pop();
	}
	cout<<" "<<cards.front()<<endl;
}
//6.3.1
// falling ball
void FallingBall(int Depth, int index)
{
	int N = 1<<Depth;
	N--;
	vector<int> bucket;
	bucket.resize(N);
	static const int MAX_N = 4096;
	bitset<MAX_N> switches;
	switches.reset();
	for(int i=0;i<N;i++)
	{
		bucket[i] = i+1;
	}
	for(int t=1;t<= index; t++)
	{
		int index_of_node = 0;
		while(index_of_node<N)
		{
			int tmp_index = index_of_node*2+1;
			if(switches[index_of_node])
				tmp_index++;
			switches[index_of_node].flip();
			if(tmp_index >= N)
			{
				break;
			}
			else
				index_of_node = tmp_index;
		}
		cout<<" "<<t<<" th at bucket: "<<bucket[index_of_node]<<endl;
	}
}
// Actually each ball has its own trajectory 
// Each Node has the similar behavior
void FallingBall2(int Depth,int th)
{
	int N=(1<<Depth) -1;
	vector<int> bucket;
	bucket.resize(N);
	for(int i=0;i<N;i++)
		bucket[i] = i+1;
	int k=0;
	int tmp_th=th;
	while(k<N)
	{
		int tmp_k = k*2+1;
		if(tmp_th & 0x01) // th is odd
		{
			tmp_th = (tmp_th+1)>>1;
		}
		else
		{
			tmp_k++;
			tmp_th>>=1;
		}
		if(tmp_k>=N)
		{
			break;
		}
		k = tmp_k;
	}
	cout<<" "<<th<<" th ball falls at bucket "<<bucket[k]<<endl;
}
//6.3.3 rebuild postorder from preorder and midorder 
// mid: DBACEGF pre: ABCDEFG
void obtainPostElement(string& mid,string& pre,string& post)
{
	if(mid.size()==0)
		return;
	char tmp_c=mid[0];
	for(size_t i=0;i<pre.size();i++)
	{
		if(tmp_c ==pre[i])
		{
			post+=tmp_c;
			// traverse right-subtree
			if(i+1<mid.size()){
				string right_mid(mid,i+1,mid.size()-i-1);
				string right_pre(pre,i+1,pre.size()-i-1);
				obtainPostElement(right_mid,right_pre,post);
			}
			// traverse left-subtree
			if(i>=1){
				string left_mid(mid,1,i);
				string left_pre(pre,0,i);
				obtainPostElement(left_mid,left_pre,post);
			}
			break;
		}
	}
	return ;
}

void RebuildPostOrder(char* mid, char* pre)
{
	string pre_s(pre);
	string mid_s(mid);
	// assert(false) -> error
	assert(pre_s.size()==mid_s.size());
	string post_s;
	obtainPostElement(mid_s,pre_s,post_s);
	cout<<post_s<<endl;
}

// 6.4.3 topplogy sort
// Input n chars,and m (xi,xj): xi<xj
void TopologySort(int n,char* letters, int m,char* front,char* end)
{
	vector<char> stack;
	map<char,int> degree;
	// Initialization
	for(int i=0;i<n;i++)
	{
		degree.insert(make_pair(letters[i],0));
	}
	for(int i=0;i<m;i++)
	{
		degree[end[i]]++;
	}
	for(map<char,int>::iterator it=degree.begin();it!=degree.end();it++)
	{
		if(it->second==0)
		{
			stack.push_back(it->first);
		}
	}
	// Let's go
	bitset<10> valid;
	valid.set();
	while(!stack.empty())
	{
		char checked_char=*stack.rbegin();
		cout<<" "<<checked_char;
		stack.pop_back();
		for(int i=0;i<m;i++)
		{
			if(valid[i] && front[i]==checked_char)
			{
				degree[end[i]]--;
				if(degree[end[i]]==0)
				{
					stack.push_back(end[i]);
					valid[i].flip();
				}
			}
		}
	}
}
//int main()
//{
//	//Cards(7);
//	//FallingBall(4,8);
//	//FallingBall2(4,8);
//	//RebuildPostOrder("DBACEGF","ABCDEFG");
//	TopologySort(5,"ABCDE",5,"AACDB","BCDBE");
//	getchar();
//	return 0;
//}