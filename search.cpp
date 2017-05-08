#include <math.h>
#include <vector>
#include <string>
#include <iostream>
//#include <cmath>

namespace {
	int abs(int x){
		if(x<0) x = x*-1;
		return x;
	}
}
using namespace std;

class CSearch{
public:
	CSearch(int width = 10, int height = 10);
	bool next(int, int, int,int& ,int&);
	void startSearch(int , int);
	bool nextloop(int, int&, int&);
	bool isValid(int, int);
	int m_w;
	int m_h;
	char m_dir[4];
	int index;
	int m_sx;
	int m_sy;
};

CSearch::CSearch(int width, int height):m_w(width),m_h(height){
	if(m_w<=0  || m_h <=0){
		cout<<"size error!"<<endl;
	}
	m_dir = {'L','U','R','D'};
	index = 0;
}

void CSearch::startSearch(int sx, int sy){
	cout<<"Start point:"<<sx<<","<<sy<<endl;
	int cnt = 0;
	
	m_sx = sx; 
	m_sy = sy;
	int nx = sx+1;
	int ny = sy+1;
	int dis = 1;

	while(1){
		cout<<"loop dis: "<<dis<<endl;
		if(!nextloop(dis,sx,sy))
			break;
		dis++;
		cout<<endl;
	}
	cout<<endl;
}

bool CSearch::isValid(int x, int y){
	return ((x>=0)&&(x<m_w)&&(y>=0)&&(y<m_h));
}

bool CSearch::nextloop(int dis, int& cx, int& cy){
	int sx = cx - 1; // start from the left order
	int sy = cy;
	
	int ix = sx; 
	int iy = sy;
	int nx,ny;
	if(!isValid(ix,iy)){
		return false;
	}
	while(1){
		cout<<" ("<<ix<<","<<iy<<")";
		if(!next(dis,ix,iy,nx,ny))
			return false;
		if(nx == sx && ny == sy){
			cx = ix;
			cy = iy;
			return true;
		}
		ix = nx;
		iy = ny;
	}
}

bool CSearch::next(int dis, int ix, int iy, int& nx, int& ny){
	bool ret = true;
	// index = (index+1)%4;
	int tx = ix;
	int ty = iy;
	bool found = false;
	while(!found){
	switch(m_dir[index]){
	case 'L':
		tx--;
		if(!isValid(tx,ty))
		{
			if(!isValid(tx+1,ty-1))	
				return false;
			else
			{
				tx++;
				index = (index+1)%4;
			}
			continue;
		}
		if(abs(tx-m_sx)<=dis)
			found = true;
		else {
			tx++;
			index = (index+1)%4;
		}
		break;
	case 'U':
		ty--;
		if(!isValid(tx,ty)){
			if(!isValid(tx+1,ty+1)){
				return false;
			}
			else{
				ty++;
				index = (index+1)%4;
			}
			continue;
		}
		if(abs(ty-m_sy)<=dis)
			found = true;
		else{
			ty++;
			index = (index+1)%4;
		}
		break;
	case 'R':
		tx++;
		if(!isValid(tx,ty)){
			if(!isValid(tx-1,ty+1)){
				return false;
			}else{
				tx--;
				index = (index+1)%4;
			}
			continue;
		}
		if(abs(tx-m_sx)<=dis)
			found = true;
		else{
			tx--;
			index = (index+1)%4;
		}
		break;
	case 'D':
		ty++;
		if(!isValid(tx,ty)){
			if(!isValid(tx-1,ty-1))
				return false;
			else{
				ty--;
				index = (index+1)%4;
			}
			continue;
		}
		if(abs(ty-m_sy)<=dis)
			found = true;
		else{
			ty--;
			index = (index+1)%4;
		}
		break;
	}
	}
	nx = tx;
	ny = ty;
	return true;
}


int main(){
	CSearch serach(25,25);
	serach.startSearch(12,12);
	return 0;
}


