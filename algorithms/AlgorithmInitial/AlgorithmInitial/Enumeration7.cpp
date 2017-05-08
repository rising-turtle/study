#include "preheader.h"

// 7-2 EgyptNum 
// 19/45 = 1/5 + 1/6 + 1/18
void swap7(int& a, int& b)
{
	b^=a^=b=a^b;
}
pair<int,int> Reduction(int a,int b)
{
	assert(a<b);
	int c=a;
	int d=b;
	while(1)
	{
		d%=c;
		if(d<c) swap7(c,d);
		if(c<=0)
			break;
	}
	return make_pair(a/d,b/d);
}
set<int> fraction;
bool finished = false;

bool EgyptNumDepth(int a,int b,int c, int depth,set<int>& record)
{
	if(depth==0)
		return false;
	bool ret=false;
	int d=depth;
	if(d==2)
		d=d;
	for(int cc=c;cc>1;cc--)
	{	
		if(a*cc-b>=1)
		{
			if(!fraction.empty() && cc>*fraction.rbegin())
				continue;
			if(record.find(cc)!=record.end()) // already used
				continue;
			record.insert(cc); //this cc maybe good

			pair<int,int> next_pair=Reduction(a*cc-b,cc*b);
			ret = EgyptNumDepth(next_pair.first,next_pair.second,next_pair.second,depth-1,record);
			if(ret) // yes, find answer at depth=d
			{
					finished = true;
			}
			record.erase(record.find(cc));	
		}
		else if(a*cc-b==0)
		{
			if(record.find(cc)==record.end())
			{
				record.insert(cc);
				
				if(fraction.empty() || *record.rbegin()< *fraction.rbegin())// update 
					fraction=record;
				record.erase(record.find(cc));
				ret = true;
				break;
			}
		}
		else // this is already a*cc-b<0 
		{
			return false;
		}
	}
	return ret;
}
void EgyptNum(int a, int b)
{
	set<int> record;
	// depth search 
	for(int d=0;;d++)
	{
		EgyptNumDepth(a,b,b,d,record);
		if(finished)
			break;
	}
	cout<<a<<"/"<<b<<" = ";
	for(set<int>::iterator it=fraction.begin();it!=fraction.end();it++)
	{
		cout<<"1/"<<*it<<" + ";
	}
	cout<<endl;
}
void testReduction(int n)
{
	srand(unsigned int(time(NULL)));
	for(int k=0;k<n;k++)
	{
		int a=rand()%100;
		int b=rand()%100;
		if(a>b) swap7(a,b);
		cout<<"before "<<a<<"/"<<b<<endl;
		pair<int,int> ret=Reduction(a,b);
		cout<<"after "<<ret.first<<"/"<<ret.second<<endl;
	}
}

// 7.5.3 Night meshes
// Input		2 6 4   Destination  8 1 5 
//				1 3 7				 7 3 6
//				0 5 8				 4 0 2
// Output 31: least step from Input to Destination to move number:0
static const int MAXSTATE=400000; // > 9!=362880
typedef int State[9];
State ST[MAXSTATE];
int t_x[]={-1,0,0,1};
int t_y[]={0,-1,1,0};

int hash_array[]={1,10,100,1000,10000,100000,1000000,10000000,100000000};
long long getHashNum(int st[9])
{
	long long ret=0;
	for(int i=0;i<9;i++)
		ret+=st[i]*hash_array[i];
	return ret;
}
void InST(int index,State cur)
{
	for(int i=0;i<9;i++)
		ST[index][i]=cur[i];
}
void NightCells(int Input[9],int Dst[9])
{
	int depth=0;
	int front=0;
	int back=0;
	State src;
	State dst;
	for(size_t i=0;i<9;i++)
	{
		src[i] = Input[i];
		dst[i] = Dst[i];
	}
	//queue<State> q_state;
	queue<int> q_depth;
	set<long long> s_hash;
	InST(back,src);
	back++;
	q_depth.push(depth);
	s_hash.insert(getHashNum(src));

	while(front!=back)
	{
		// obtain current element
		State& cur=ST[front++];
		depth=q_depth.front();
		q_depth.pop();
		
		if(memcmp(cur,dst,sizeof(dst))==0)
		{
			cout<<"final get state dst through: "<<depth<<endl;
			for(size_t i=0;i<9;)
			{
				cout<<" "<<cur[i++];
				if(i%3 ==0)
					cout<<endl;
			}
			return;
		}

		int index_of_zero=0;
		for(;index_of_zero<9;index_of_zero++) if(!cur[index_of_zero]) break; // find location of zero

		int c_x=index_of_zero%3;
		int c_y=index_of_zero/3;
		int c_z=index_of_zero;

		for(int i=0;i<4;i++) // enumerate all possible states
		{
			int n_x=c_x+t_x[i];
			int n_y=c_y+t_y[i];
			if(n_x<0 || n_x>2 || n_y<0 || n_y>2) continue; // not a valid state

			// record this valid state
			int n_z=n_x+n_y*3;
			swap7(cur[c_z],cur[n_z]);
			long long hash_state=getHashNum(cur);
			if(s_hash.find(hash_state)==s_hash.end()) // not exist
			{
				s_hash.insert(hash_state);
				InST(back,cur);
				back++;
				q_depth.push(depth+1);
			}
			swap7(cur[c_z],cur[n_z]);
		}

	}
}

//void main()
//{
//	//EgyptNum(2,3);
//	//EgyptNum(7,13);
//	//EgyptNum(19,45);
//	//testReduction(10);
//	int src[]={2,6,4,1,3,7,0,5,8};
//	int dst[]={8,1,5,7,3,6,4,0,2};
//	NightCells(src,dst);
//	getchar();
//}