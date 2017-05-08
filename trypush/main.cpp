#include <vector>
#include <iostream>

using namespace std;
#define N 10
int main(int argc, char* argv[])
{
	vector< vector< float> > vv;
	vector<float> v(N,-1);
	vv.push_back(v);

	return 0;
}
