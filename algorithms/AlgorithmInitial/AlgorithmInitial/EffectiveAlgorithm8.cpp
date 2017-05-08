#include "preheader.h"
// 8.4 Range statistic
// input n  
//
//typedef struct _TNode* TNode;
//struct _TNode
//{
//	int lower;
//	int upper;
//	int key;
//	int num;
//	struct _TNode* l;
//	struct _TNode* r;
//};
//
//
//TNode createNode(int lower,int upper,int key,int num,TNode l, TNode r)
//{
//	TNode node=(TNode)malloc(sizeof(struct _TNode));
//	node->lower=lower;
//	node->upper=upper;
//	node->key=key;
//	node->num=num;
//	node->l=l;
//	node->r=r;
//	return node;
//}
//TNode root=NULL;
//void CreateRange(int input[],int n)
//{
//	TNode* pcur;
//	for(int i=0;i<n;i++)
//	{
//		int key=input[i];
//		int lower=key;
//		int upper=key;
//		int num=1;
//		pcur=&root;
//		while(*pcur!=NULL)
//		{
//			(*pcur)->num++;////
//			// three conditions 
//			if(key>= (*pcur)->key)
//			{
//				if((*pcur)->upper<key)
//					(*pcur)->upper=key;
//				pcur=&((*pcur)->r);
//			}
//			else
//			{
//				if((*pcur)->lower>key)
//					(*pcur)->lower=key;
//				pcur=&((*pcur)->l);
//			}
//		}
//		*pcur=createNode(lower,upper,key,num,NULL,NULL);
//	}
//	
//}
//int RangeStatics(TNode node,int a, int b)
//{
//	if(node==NULL)
//		return 0;
//	if(b<node->lower || a>node->upper)
//		return 0;
//	int ret=0;
//	if(node->lower>=a && node->upper<=b)
//	{
//		ret+=node->num;
//	}
//	else 
//	{
//		if(node->key<=b && node->key>=a) // a<key<b
//		{
//			ret++;
//			ret+=RangeStatics(node->l,a,b);
//			ret+=RangeStatics(node->r,a,b);
//		}
//		else if(node->key>b)
//			ret+=RangeStatics(node->l,a,b);
//		else if(node->key<a)
//			ret+=RangeStatics(node->r,a,b);
//	}
//	return ret;
//}

//void main()
//{
//	// output 5
//	int input[]={1,5,3,8,10,21,11,15};
//	CreateRange(input,8);
//	cout<<RangeStatics(root,4,20)<<endl;
//	getchar();
//}