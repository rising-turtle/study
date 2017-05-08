#include "preheader.h"
#include "QuickSort.h"
#include <ctime>
#include <stdlib.h>

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

void PathWeightFloor(int W[ROW][COL],Status S[ROW][COL])
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
void PathWeightBlock(int W[ROW][COL],Status S[ROW][COL])
{
	vector<int> vrow;
	vector<int> vcol;
	for(int i=0;i<ROW;i++)
		for(int j=0;j<COL;j++)
		{
			W[i][j]=1;
			if(S[i][j]==Block)
			{
				vrow.push_back(i);
				vcol.push_back(j);
				W[i][j]=-1;
			}
		}
	for(size_t i=0;i<vrow.size();i++)
	{
		int row=vrow[i];
		int col=vcol[i];
		int dis=1;
		for(;dis<=DIS;dis++)
		{
			for(int l=0;l<=dis;l++)
			{
				int k=dis-l;
				if(l==0)
				{
					if( col-k>=0 && S[row][col-k]==Floor){W[row][col-k]=W[row][col-k]+DIS-dis;}
					if( col+k<COL && S[row][col+k]==Floor) {W[row][col+k]=W[row][col+k]+DIS-dis;}
				}
				else if(k==0)
				{
					if( row-l>=0  && S[row-l][col]==Floor) {W[row-l][col]=W[row-l][col]+DIS-dis;}
					if( row+l<ROW  && S[row+l][col]==Floor) {W[row+l][col]=W[row+l][col]+DIS-dis;}
				}
				else 
				{
					if(row-l>=0 && col-k>=0 && S[row-l][col-k]==Floor){W[row-l][col-k]=W[row-l][col-k]+DIS-dis;}	
					if(row-l>=0 && col+k<COL && S[row-l][col+k]==Floor) {W[row-l][col+k]=W[row-l][col+k]+DIS-dis;}
					if(row+l<ROW && col-k>=0 && S[row+l][col-k]==Floor) {W[row+l][col-k]=W[row+l][col-k]+DIS-dis;}
					if(row+l<ROW && col+k<COL && S[row+l][col+k]==Floor) {W[row+l][col+k]=W[row+l][col+k]+DIS-dis;}
				}
			}
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
void Display(int Path[ROW][COL])
{
	cout<<endl;
	for(int i=0;i<ROW;i++)
	{
		for(int j=0;j<COL;j++)
			cout<<Path[i][j]<<" ";
		cout<<endl;
	}
}
void PathPlaning(Status S[ROW][COL],int W[ROW][COL],vector<int>& path,unsigned int sp,unsigned int dp)
{
	unsigned short s_row,s_col;
	queue<int> q_update;
	
	s_row=(sp&0xFFFF0000)>>16;
	s_col=(sp&0x0000FFFF);
	if(s_row<0 || s_row>=ROW || s_col<0 || s_col>=COL)
	{
		cout<<"row="<<s_row<<",col="<<s_col<<"Out"<<endl;
	}
	
	unsigned short d_row,d_col;
	d_row=(dp&0xFFFF0000)>>16;
	d_col=(dp&0x0000FFFF);
	if(d_row<0 || d_row>=ROW || d_col<0 || d_col>=COL)
	{
		cout<<"Destination exceed! row="<<d_row<<",col="<<d_col<<endl;
	}
	
	// 
	int SP[ROW][COL];
	for(int i=0;i<ROW;i++)
		for(int j=0;j<COL;j++)
		{
			if(S[i][j]==Block)
				SP[i][j]=-1;
			else
				SP[i][j]=0;
		}
	SP[s_row][s_col]=1;// Original Point
	q_update.push(sp);
	while(!q_update.empty())
	{
		int sindex=q_update.front();
		q_update.pop();

		unsigned short srow=(sindex&0xFFFF0000)>>16;
		unsigned short scol=(sindex&0x0000FFFF);

		// UP
		if(srow>0)
		  if(SP[srow-1][scol]>=0)
		  if(SP[srow-1][scol]==0 || SP[srow][scol]+W[srow-1][scol]<SP[srow-1][scol])
			{
				SP[srow-1][scol]=SP[srow][scol]+W[srow-1][scol];
				q_update.push(((srow-1)<<16)+scol);
			}
		

		// DOWN
		if(srow<ROW-1)
			if(SP[srow+1][scol]>=0)
			if(SP[srow+1][scol]==0 || SP[srow][scol]+W[srow+1][scol]<SP[srow+1][scol])
			{
				SP[srow+1][scol]=SP[srow][scol]+W[srow+1][scol];
				q_update.push(((srow+1)<<16)+scol);
			}
		// LEFT
		if(scol>0)
			if(SP[srow][scol-1]>=0)
			if(SP[srow][scol-1]==0 || SP[srow][scol]+W[srow][scol-1]<SP[srow][scol-1])
			{
				SP[srow][scol-1]=SP[srow][scol]+W[srow][scol-1];
				q_update.push((srow<<16)+scol-1);
			}
		// RIGHT
		if(scol<COL-1)
			if(SP[srow][scol+1]>=0)
			if(SP[srow][scol+1]==0 || SP[srow][scol]+W[srow][scol+1]<SP[srow][scol+1])
			{
				SP[srow][scol+1]=SP[srow][scol]+W[srow][scol+1];
				q_update.push((srow<<16)+scol+1);
			}
	}
	Display(SP);

	path.clear();
	unsigned short row=d_row;
	unsigned short col=d_col;
	int d_cost=SP[row][col];
	path.push_back((row<<16)+col);
	while(row!=s_row && col!=s_col)
	{
		int d_cost_back=d_cost-W[row][col];
		// UP
		if(row>0 && d_cost==SP[row-1][col]+W[row][col])
		{
			row--;
		}
		// DOWN
		else if(row<ROW-1 && d_cost==SP[row+1][col]+W[row][col])
		{
			row++;
		}
		else if(col>0 && d_cost==SP[row][col-1]+W[row][col])
		{
			col--;
		}
		else if(col<COL-1 && d_cost==SP[row][col+1]+W[row][col])
		{
			col++;
		}
		path.push_back((row<<16)+col);
		d_cost=d_cost_back;
	}
	path.push_back((s_row<<16)+s_col);
	bitset<ROW*COL> bFlag(false);
	for(size_t i=0;i<path.size();i++)
	{
		unsigned short brow=(path[i]&0xFFFF0000)>>16;
		unsigned short bcol=(path[i]&0x0000FFFF);
		bFlag[brow*COL+bcol]=true;
	}
	for(size_t i=0;i<ROW;i++)
	{
		for(size_t j=0;j<COL;j++)
		{
			if(bFlag[i*COL+j])
				cout<<SP[i][j]<<" ";
			else
				cout<<"  ";
		}
		cout<<endl;
	}
	return ;
}

void testQuickSort(int N)
{
	vector<int> s;
	vector<int> index;
	s.resize(N);
	index.resize(N);
	srand((unsigned int)(time(NULL)));
	for(int i=0;i<N;i++)
	{
		s[i]=(rand()%20);
	}
	cout<<"before QuickSort: "<<endl;
	displayVec<int>(s);
	QuickSort(s,index);
	cout<<"after QuickSort: "<<endl;
	displayVec<int>(s,index);
}

//void main()
//{
//
//}
