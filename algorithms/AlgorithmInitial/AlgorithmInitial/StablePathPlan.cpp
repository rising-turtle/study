#include "preheader.h"

typedef enum{Unknown, Block, Floor}Status;

const static int ROW=10;
const static int COL=10;
const static int DIS=3;
const static int UNKNOW=20;

int		W[ROW][COL];
Status  S[ROW][COL];

void GenerateMap(Status S[ROW][COL])
{
	for(int i=0;i<ROW;i++)
		for(int j=0;j<COL;j++)
			S[i][j]=Floor;

	// Block 1 
	for(int i=ROW/4;i<3*ROW/4;i++)
		for(int j=0;j<COL/4;j++)
			S[i][j]=Block;
	// Block 2
	for(int i=4*ROW/5;i<ROW;i++)
		for(int j=COL/3;j<=COL/2;j++)
			S[i][j]=Block;
	//Block 3
	for(int i=0;i<=ROW/3;i++)
		for(int j=3*COL/4;j<COL;j++)
			S[i][j]=Block;
}

void PathWeight(int W[ROW][COL],Status S[ROW][COL])
{
	for(int i=0;i<ROW;i++)
		for(int j=0;j<COL;j++)
		{
			W[i][j]=1;
		}
	for(int i=0;i<ROW;i++)
		for(int j=0;j<COL;j++)
		{
			if(S[i][j]==Block)
			{
				W[i][j]=-1;
			}
			else if(S[i][j]==Unknown)
			{
				W[i][j]+=UNKNOW;
			}
			else if(S[i][j]==Floor) // find nearest BLOCK
			{
				int dis=1;
				bool flag=false;
				for(;dis<=DIS;dis++)
				{
					flag=false;
					for(int l=0;l<=dis;l++)
					{
						int k=dis-l;
						if(i-l>0 && j-k>0 && S[i-l][j-k]==Block){flag=true;break;}
						
						if(i-l>0 && j+k<COL && S[i-l][j+k]==Block) {flag=true;break;}

						if(i+l<ROW && j-k>0 && S[i+l][j-k]==Block) {flag=true;break;}

						if(i+l<ROW && j+k<COL && S[i+l][j+k]==Block) {flag=true;break;}
					}
					if(flag) break;
				}
				W[i][j]+=DIS-dis+1;
			}
		}
}
void Display(Status S[ROW][COL],int W[ROW][COL])
{
	for(int i=0;i<ROW;i++)
	{
		for(int j=0;j<COL;j++)
		{
			switch(S[i][j])
			{
			case Block:
				cout<<"B";
				break;
			case Floor:
				cout<<"F";
				break;
			case Unknown:
				cout<<"U";
				break;
			default:
				break;
			}
		}
		cout<<endl;
	}
	cout<<endl;
	for(int i=0;i<ROW;i++)
	{
		for(int j=0;j<COL;j++)
			cout<<W[i][j];
		cout<<endl;
	}
}
void PathPlaning(Status S[ROW][COL],vector<int>& path)
{
	int Weight[ROW][COL];
	
}
//void main()
//{
//	GenerateMap(S);
//	PathWeight(W,S);
//	Display(S,W);
//}
