#include <iostream>
#include <map>
#include <string>
//#include <pair>

/*
	求逆序数，然后排序
*/

using namespace std;

namespace{
char gl_input[][11]={"AACATGAAGG",
		  "TTTTGGCCAA",
		  "TTTGGCCAAA", 
		  "GATCAGATTT",  
		  "CCCGGGGGGA",
		  "ATCGATGCAT"};

// A,C,G,T A<C<G<T
int getScore(char *seq,int len)
{
	int n = len-1;
	//int ret = 0;
	// record the number of 'A''C''G''T'
	map<char,int> score_record;
	score_record.insert(make_pair('A',0));
	score_record.insert(make_pair('C',0));
	score_record.insert(make_pair('G',0));
	score_record.insert(make_pair('T',0));
	
	int score=0;
	for(int i=n;i>=0;i--){
		switch(seq[i]){
			case 'A':
			 score_record['A']++;
			break;
			case 'C':
			 score+=score_record['A'];
			 score_record['C']++;
			break;
			case 'G':
			 score+=score_record['A'];
			 score+=score_record['C'];
			 score_record['G']++;
			break;
			case 'T':
			 score+=score_record['A'];
			 score+=score_record['C'];
			 score+=score_record['G'];
			 score_record['T']++;
			break;
		}	
	}
	return score;
}


void sort(char** input,int col, int wid){
	multimap<int,int> score_sort;
	for(int i=0;i<wid;i++){
		score_sort.insert(make_pair(getScore(input[i],col),i));
	}
	for(multimap<int,int>::iterator it=score_sort.begin();it!=score_sort.end();it++)	{
		int index=(*it).second;
		for(int i=0;i<10;i++)
		{
			std::cout<<input[index][i];
		}
		std::cout<<endl;
	}
}

void read()
{
	int wid,col;
	cin>>col;
	cin>>wid;
	//  开辟数组
	char** input = (char**)new char*[wid];
	for(int i=0;i<wid;i++)
		input[i]=(char*)new char[col];
	
	for(int i=0;i<wid;i++)
		for(int j=0;j<col;j++)
			cin>>input[i][j];
	
	sort(input,col,wid);

	// 释放数组
	for(int i=0;i<wid;i++)
		delete []input[i];
	delete []input;
}

void init(){
	int wid,col;
	wid = 6;
	col = 10;
	//  开辟数组
	char** input = (char**)new char*[wid];
	for(int i=0;i<wid;i++)
		input[i]=(char*)new char[col];

	for(int i=0;i<wid;i++)
		for(int j=0;j<col;j++)
			input[i][j] = gl_input[i][j];

	sort(input,col,wid);

	// 释放数组
	for(int i=0;i<wid;i++)
		delete []input[i];
	delete []input;
}

}
int main1007()
{
	init();
	//read();
return 0;
}
