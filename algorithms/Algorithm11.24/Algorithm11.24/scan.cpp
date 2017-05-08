#include <vector>

using namespace std;
extern void display(vector<int>& v, string tag="");
void randv(vector<int>& v, int n, int range, int shift = 0);

void swap(vector<int>& a, int i, int j)
{
	if(i==j) return ;
	a[i] = a[i] ^ a[j];
	a[j] = a[i] ^ a[j];
	a[i] = a[i] ^ a[j];
}

// Dutch flag
void scan(vector<int>& iv)
{
	int j1 =0, j2 = 0, j0 = 0; 
	vector<int> v(iv);
	for(int i=0; i<v.size(); i++)
	{
		switch(v[i])
		{
			case 0:
				// color2s = ++color1e;
				swap(v,j1++, i);
				swap(v,j2++, i);
				break;
			case 1:
				swap(v, j2++, i);
				break;
			default:
				break;
		}
	}
	display(iv, "input: ");
	display(v, "output: ");
}

void test_scan()
{
	vector<int> v;	
	randv(v, 10, 3);
	scan(v);
}

// SUM x 
// search two numbers whose sum is 