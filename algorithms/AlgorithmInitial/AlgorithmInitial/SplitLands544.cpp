#include <iostream>

using namespace std;

// Eular V-E+F = 2
// For each vertex, It gains i*(n-i-2) vertexes for i=0...n-2 all vertex = N + N/4*sum(i*(N-i-2)) for i=0...N-2
// For each vertex, It gains i*(n-i-2)+1 edges for i=0...n-2 all edges = N+1+ N/2*sum(i*(N-i-2) + 1) for i=0...N-2
int NSplit(int n)
{
	int V,E;
	int tmp_v =0;
	int tmp_e =0;
	for(size_t i=0;i<=n-2;i++)
	{
		tmp_v += i*(n-i-2);
		tmp_e += i*(n-i-2) + 1;
	}
	V = n + tmp_v*n/4;
	E = n + tmp_e*n/2;
	cout<<"Vertex: "<<V<<endl;
	cout<<"Edge: "<<E<<endl;

	int F = E-V+2;
	cout<<"Face: "<<F<<endl;
	return F;
}


//
//void main()
//{
//	NSplit(4);
//	NSplit(6);
//	getchar();
//	return;
//}