#include <iostream>
#include <vector>


using namespace std;

int main()
{
    vector<int> vc(10); 
    for(int i=0; i<vc.size(); i++)
	vc[i] = i+1; 
    for(int i=vc.size(); i--; )
    {
	cout<<"vc["<<i<<"]: "<<vc[i]<<endl;
    }
    return 1;
}

