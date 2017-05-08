
// not finished
#include "preheader.h"

typedef struct _TNode* TNode;
struct _TNode{
	int key;
	TNode l;
	TNode r;
};

TNode CreateNode(int key,TNode l, TNode r)
{
	TNode t=new _TNode;
	t->key=key;
	t->l=l;
	t->r=r;
	return t;
}

int getHeightTree(TNode root)
{
	if(root==NULL)
		return 0;
	return max(getHeightTree(root->l)+1,getHeightTree(root->r)+1);
}

void ShowTree(TNode& root)
{
	if(root==NULL)
		return;
	int H=getHeightTree(root);
	int total_space=1<<H;
	deque<TNode> TQueue;
	deque<int> HQueue;
	HQueue.push_back(0);
	TQueue.push_back(root);
	int cur_space=total_space>>1;
	int last_h=0;
	while(1)
	{
		TNode& pNode=TQueue.front();
		int& cur_h=HQueue.front();
		TQueue.pop_front();
		HQueue.pop_front();
		if(cur_h>=H)
			break;
		if(cur_h!=last_h)
		{
			cout<<endl;
			cur_space>>=1;
			last_h=cur_h;
		}
		for(int i=0;i<cur_space-1;i++)
			cout<<" ";
		if(pNode==NULL)
		{
			cout<<" ";
			TQueue.push_back(NULL);
			TQueue.push_back(NULL);
		}
		else
		{
			cout<<pNode->key;
			TQueue.push_back(pNode->l);
			TQueue.push_back(pNode->r);
		}
		HQueue.push_back(cur_h+1);
		HQueue.push_back(cur_h+1);
	}
}
// All these below for testing
// construct BST tree to test
void InsertNode(TNode& root,int k)
{
	if(root==NULL)
	{
		root=CreateNode(k,NULL,NULL);
		return ;
	}
	TNode p=root;
	
	if(p->key<=k)
		InsertNode(p->r,k);
	else
		InsertNode(p->l,k);
}
void testShowTree(int N)
{
	TNode root=NULL;
	srand(unsigned int(time(0)));
	for(int i=0;i<N;i++)
	{
		int tmp=rand()%20;
		cout<<" "<<tmp;
		InsertNode(root,tmp/*rand()%20*/);
	}
	cout<<endl;
	ShowTree(root);
}

//void main()
//{
//	testShowTree(5);
//	getchar();
//}