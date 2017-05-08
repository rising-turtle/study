#include <iostream>
#include <set>
using namespace std;

namespace{
int step(int speed,int dis,int Loop)
{
	int ret = 0;
	int n_step;
	while(1){
		if(dis%speed==0){
			ret+=dis/speed;
			return ret;
		}
		n_step=dis/speed+1;
		ret+=n_step;
		dis = Loop-(speed*n_step-dis);
	}
	return 0;
}

int gcd(int a,int b){
	if(b==0)
		return a;
	return gcd(b,a%b);
}

void read1061(){

	int x,y,m,n,L;
	int f_loc,s_loc,dis;
	int r_step;
	while(cin>>x>>y>>m>>n>>L)
	{
	if(m==n){
		cout<<"Impossible"<<endl;
		continue ;
	}
	else if(m>n){
		dis = x<y?y-x:L+y-x;
		int speed = m-n;
		if(dis%gcd(L,speed)){
			cout<<"Impossible"<<endl;
			continue;
		}
		r_step=step(m-n,dis,L);
	}
	else{
		dis = y<x?x-y:L+x-y;
		int speed = n-m;
		if(dis%gcd(L,speed)){
			cout<<"Impossible"<<endl;
			continue;
		}
		r_step=step(n-m,dis,L);
	}
	cout<<r_step<<endl;
	}
	return ;
}

/************************************************************************/
/* http://hi.baidu.com/newmyl/item/dc6805253305833195f62b0a                                                                    
/* ����a * x + b * y = n�������⡣
��1���ȼ���Gcd(a,b)����n���ܱ�Gcd(a,b)�������򷽳��������⣻�����ڷ�������ͬʱ����Gcd(a,b)���õ��µĲ�������a' * x + b' * y = n'����ʱGcd(a',b')=1;
  2������������˵��ŷ������㷨�������a' * x + b' * y = 1��һ��������x0,y0����n' * x0,n' * y0�Ƿ���a' * x + b' * y = n'��һ�������⣻
  3�����������е���ض����ɵ÷���a' * x + b' * y = n'������������Ϊ��
	x = n' * x0 + b' * t
	y = n' * y0 - a' * t
/************************************************************************/
// ax + by = gcd(a,b)
void exgcd(int a,int b,int& x, int& y){
	if(b==0)
	{
		x=1;
		y=0;
		return ;
	}
	exgcd(b,a%b,x,y);
	int t=x;
	x=y;
	y=t-(a/b)*y; 
}

void read1061e(){
	int x1,y1,m,n,l;
	int a,b,c,x,y;
	while(cin>>x1>>y1>>m>>n>>l){
		a=n-m;
		b=l;
		c=x1-y1;
		int d = gcd(a,b);
		if(c%d){
			cout<<"Impossible"<<endl;
			continue;
		}
		a/=d;
		b/=d;
		c/=d;
		exgcd(a,b,x,y);
		int t=x*c/b;
		x=x*c-b*t;
		if(x<0)
			x+=b;
		cout<<x<<endl;
	}
}
};
int main1061()
{
	//read1061();
	read1061e();
	return 0;
}