

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

using namespace std; 

void print(string pre, const vector<int>& a)
{
    cout <<pre<<" "<<endl; 
    for(int i=0; i<a.size(); i++)
	cout<<a[i]<<" ";
    cout<<endl;
}


int main()
{
    vector<int> vi{5, 3 ,4, 2, 8, 9, 7,7, 3}; 

    int n = 5; 
    vector<int> keypoints = vi; 
    std::nth_element(keypoints.begin(), keypoints.begin() + n, keypoints.end(),
            [](int& a, int& b) { return a > b; });
    
    print("vi", vi); 
    print("keypoints", keypoints); 
    return 0;
}
