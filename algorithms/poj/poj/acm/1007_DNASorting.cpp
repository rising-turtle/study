#include <iostream>
#include <map>
#include <string>
//#include <pair>

using namespace std;

char input[][11]={"AACATGAAGG",\ 
		  "TTTTGGCCAA",\ 
		  "TTTGGCCAAA",\ 
		  "GATCAGATTT",\  
		  "CCCGGGGGGA",\
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
	for(int i=n-1;i>=0;i--){
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

int main()
{
	int N = sizeof(input)/sizeof(input[0]);
	
	multimap<int,int> score_sort;
	for(int i=0;i<N;i++){
		score_sort.insert(make_pair(getScore(input[i],sizeof(input[i])),i));
	}

	for(multimap<int,int>::iterator it=score_sort.begin();it!=score_sort.end();it++)	{
		int index=(*it).second;
		for(int i=0;i<10;i++)
		{
			std::cout<<input[index][i];
		}
	std::cout<<endl;
	}
return 0;
}
