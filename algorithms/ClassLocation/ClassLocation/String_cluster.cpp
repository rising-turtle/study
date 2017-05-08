#include "preheader.h"

// Test whether str2 can be obtained from str1's rotation
bool Isrotation(string& str1, string& str2){
	//string tmpstr(str1.c_str());
	int n = str1.size();
	char * tmpchar = new char[2*str1.size()];

	char * t = new char[2];
	for(size_t i=0;i<20;i++)
	{
		t[i] = 'o';
		cout<<i<<" ";
	}

	for(size_t i=0;i< 2*str1.size();i++){
		tmpchar[i] = str1.data()[i%str1.size()];
	}
	cout<<tmpchar<<endl;
	for(size_t i=0;i<str1.size()*2 - str2.size(); i++)
	{
		bool ret = true;
		for(size_t j=0;j<str2.size();j++){
			if(str2.data()[j] != tmpchar[i+j])
			{
				ret = false;
				break;
			}
		}
		if(ret) {
			delete []tmpchar;		
			return true;
		}
	}
	delete []tmpchar;
	return false;
}



//void main(){
//	string s1("AABCD");
//	string s2("CDAA");
//	if(Isrotation(s1,s2))
//		cout<<"s2 from s1"<<endl;
//	else if(Isrotation(s2,s1))
//		cout<<"s1 from s2"<<endl;
//	getchar();
//	return ;
//}