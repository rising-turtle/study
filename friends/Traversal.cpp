#include <iostream>
#include <string>
using namespace std;

typedef struct BiNode{					//����������
	char content;//����
	struct BiNode *left,*right;//�������Һ��ӵĵ�ַ
	BiNode():left(NULL),right(NULL){}
	~BiNode(){
		if(left)
			delete left;
		if(right) 
			delete right;
	}
}BiTNode,*BiTree;

void show(int type,BiTree Root)//�����������
{
	if(Root == NULL)
		return ;
	if (type==-1)//ǰ�����
	{
		if (Root!=NULL) {
			cout<<Root->content<<'\0';
			show(type,Root->left);
			show(type,Root->right);
		}
	}
	else if (type==0)//�������
	{
		if (Root!=NULL) {
			show(type,Root->left);
			cout<<Root->content<<'\0';
			show(type,Root->right);
		}
	}
	else if (type==1)//�������
	{
		if (Root) {
			show(type,Root->left);
			show(type,Root->right);
			cout<<Root->content<<'\0';
		}
	}
}

void deleteTree(BiTree root)
{
	if(root == NULL)
		return ;
	if ((root->left==NULL)&&(root->right!=NULL))
	{
		deleteTree(root->right);
	}
	if ((root->left!=NULL)&&(root->right==NULL))
	{
		deleteTree(root->left);
	}
	if ((root->left!=NULL)&&(root->right!=NULL))
	{
		deleteTree(root->left);
		deleteTree(root->right);
	}
	if ((root->left==NULL)&&(root->right==NULL))
	{
		// free(root);
		delete root;
	}
}

BiTNode *createBiTree(char *pre,char *in,int len)
{
	int k;
	if(len<=0)
	{
		return NULL;
	}
	// BiTree head=(BiTree)malloc(sizeof(BiNode));
	BiTree head = new BiNode;
	head->content=*pre;
	char *p;
	for(p=in;*p!=NULL;p++)
	{
		if(*p==*pre)  
		{	
			break;
		}
	}
	if (*p == NULL)
	{
		return NULL;
	}

	k=(int)(p-in);
	head->left=createBiTree(pre+1,in,k);
	head->right=createBiTree(pre+k+1,p+1,len-k-1);

	return head;
}

int main()
{
	BiTree Tree;
	string preorder,inorder;

/*
	//������Ŀ��д��һ���������������������뷽ʽ����
	cout<<"����ǰ������˳��\n";
	cin>>preorder;
	const char *pre_1=preorder.c_str();
	int len = strlen(pre_1);
	char *pre = new char[ len + 1 ];
	strcpy(pre,pre_1);
	cout<<"������������˳��\n";
	cin>>inorder;
	const char *in_1=inorder.c_str();
	len = strlen(in_1);
	char *in = new char[ len + 1 ];
	strcpy(in,in_1);
*/

	char pre_1[26];
	char in_1[26];

	cout<<"������������ڵ����������ڵ���1��С�ڵ���26����\n";
	int len;
	cin>>len;
	cout<<"����һ��������������ڵ��ǰ��������õ����ո�ָ���\n";
	for (int i=0;i<len;i++)
	{
		cin>>pre_1[i];
	}
	cout<<"����һ��������������ڵ������������õ����ո�ָ���\n";
	for (int i=0;i<len;i++)
	{
		cin>>in_1[i];
	}
	cout<<"��Ӧ�������ĺ�������ǣ�\n";
	Tree=createBiTree(pre_1,in_1,len);	
	show(1,Tree);
	cout<<endl;

	//һ���������
	/*if ((Tree->left==NULL)||(Tree->left==NULL))
	{
		deleteTree(Tree);
		return NULL;
	}*/
	deleteTree(Tree);
//	system("pause");
	return 0;

}
