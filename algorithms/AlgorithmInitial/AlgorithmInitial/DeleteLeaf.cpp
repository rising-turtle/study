#include "preheader.h"

typedef struct _Node* pNode;
struct _Node{
	int key;
	pNode l;
	pNode r;
};

pNode CreateNode(int key,pNode l, pNode r)
{
	pNode t=new _Node;
	t->key=key;
	t->l=l;
	t->r=r;
	return t;
}

int DeleteLeafAndCount(pNode& pN)
{
	if(pN==NULL)
		return 0;
	pNode& pl=pN->l;
	pNode& pr=pN->r;
	if(pl==NULL && pr==NULL)//this is a leaf
	{
		delete pN;
		pN=NULL;
		return 1;
	}
	return DeleteLeafAndCount(pl)+DeleteLeafAndCount(pr);
}
// All these below for testing
// construct BST tree to test
void InsertNode(pNode& root,int k)
{
	if(root==NULL)
	{
		root=CreateNode(k,NULL,NULL);
		return ;
	}
	pNode p=root;
	
	if(p->key<=k)
		InsertNode(p->r,k);
	else
		InsertNode(p->l,k);
}
void traverseMid(pNode root)
{
	if(root==NULL)
		return;
	traverseMid(root->l);
	cout<<" "<<root->key;
	traverseMid(root->r);
}
void test(int N)
{
	pNode root=NULL;
	srand(unsigned int(time(0)));
	for(int i=0;i<N;i++)
	{
		int tmp=rand()%20;
		cout<<" "<<tmp;
		InsertNode(root,tmp/*rand()%20*/);
	}
	cout<<endl;
	traverseMid(root);
	cout<<endl;
	cout<<"Delete Num of leaf:"<<DeleteLeafAndCount(root)<<endl;
	cout<<endl;
	traverseMid(root);
}
//void main()
//{
//	test(6);
//	getchar();
//}
