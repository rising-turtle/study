#include "preheader.h"
#include "myiterator.h"

#include "template.h"

extern void test1016();
extern void test1017(string file);
extern int test1019(int loc);
extern void test1018(string file);
extern int main1019();
extern int main1061();
extern int main1007();
extern int main1016();
extern int main1017();

extern void test_intersection();
extern void test_wordreplace();

extern int Treemain();

vector<string>  strsplit(string str, string delimit);

void main()
{
	//test1016();
	//test1017("inputf\\1017.txt");
	//test1018("inputf\\1018.txt");
	//test1019(20);
	//main1019();
	//main1061();
	//test_intersection();
	//test_wordreplace();
	//main1007();
	//test1016();
	//main1016();
	
	// main1017();
/*using namespace template1;
	cout<<"int: "<<C1<int>::m_sElem<<endl;
	cout<<"str: "<<C1<string>::m_sElem<<endl;*/
	//int a[] = {1,2,3,4,5,6};
	//vector<int> tmpa(a,a+6);
	//copy(tmpa.begin(),tmpa.end(),ostream_iterator<int>(cout," ")); cout<<endl;
	//copy_backward(tmpa.begin(),tmpa.begin()+3,tmpa.end());
	//copy(tmpa.begin(),tmpa.end(),ostream_iterator<int>(cout," ")); cout<<endl;
	
	Treemain();
	getchar();
	return ;
}


vector<string>  strsplit(string str, string delimit)
{
	vector<string> ret;
	int npos=0;
	while(1){
		npos=str.find(delimit.c_str());
		if(npos == -1){
			ret.push_back(str);
			break;
		}
		ret.push_back(str.substr(0,npos));
		str=str.substr(npos+delimit.size(),str.size());
	}
	return ret;
}