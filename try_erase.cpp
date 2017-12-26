#include <vector>
#include <iostream>

using namespace std; 

int main()
{
    vector<int> va{1, 2, 3, 5, 7}; 
    vector<int>::iterator ita = va.begin() + va.size(); 
    --ita; 
    while(ita != va.begin())
    {
	cout <<"*ita = "<<*ita<<" "; 
	if(*ita == 3)
	{
	    ita = va.erase(ita); 
	    cout <<endl<<"now *ita = "<<*ita<<endl; 
	}
	--ita;
    }
    cout <<endl<<"begin() : "<<*ita<<endl; 

    vector<int> va1{1}; 
    vector<int>::iterator it = va1.end();
    --it; 
    it = va1.erase(it); 
    if(it == va1.begin())
    {
	cout <<" it = va1.begin() "<<endl;
    }
    if(it == va1.end())
    {
	cout <<" it = va1.end() "<<endl;
    }
    return 1;
}
