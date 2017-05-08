#include <iostream>
#include <vector>
#include <map>
#include "stdlib.h"
#include "string.h"

using namespace std;

char findFirstSingleChar(const char s[]);

int main(int argc, char* argv[])
{
	while(1)
	{
		string str;
		cout<<"input a string!"<<endl;
		cin>>str;
		cout<<"input: "<<str<<endl;
		if(strcmp(str.c_str(),"exit") == 0)
			break;
		cout<<"first single char is: "<<findFirstSingleChar(str.c_str())<<endl;
	}
	cout<<"exit!"<<endl;
	return 0;
}

const unsigned int PN = 65537;

unsigned int hash(unsigned int query)
{
	return (query%PN);
}

bool lookFortable(vector<char>& table, unsigned int& index, char key)
{
	if(index <0 || index>=table.size())
	{
		cout<<"error index: "<<index<<endl;
		return false;
	}
	if(table[index]==' ')
	{
		table[index] = key;
		return false;
	}else{
		unsigned int j = index;
		while(table[j]!=key && table[j]!=' ')
		{
			j++;
		}
		if(table[j]==' ')
		{
			table[j] = key;
			index = j;
			return false;
		}
	}
	return true;
}

char findFirstSingleChar(const char s[])
{
	int len = strlen(s);
	if(len <= 0) 
	{
		cout<<"input str is invalid: "<<s<<endl;
		return '*';
	}
	// vector<bool> v(len,true);
	vector<char> hash_table(PN,' ');
	vector<int> c_set(len,0);
	map<int, bool> ret;

	for(int i=0;i<len;i++)
	{
		char c = s[i];
		unsigned int index = hash((unsigned int)(c));
		// cout<<"c: "<<c<<" index: "<<index;
		if(lookFortable(hash_table,index,c))
		{
		//	cout<<" matched!"<<endl;
			ret[index] = false;
			// v[i] = false;
		}
		else 
		{
			// c_set.push_back(index);
			ret.insert(make_pair(index,true));
		//	cout<<" unique!"<<endl;
		}
		c_set[i] = index;
	}
	/*
	map<char,bool>::iterator it = ret.begin();
	while(it!=ret.end())
	{
		if(it->second)
		{
			return it->first;
		}
		it++;
	}*/
	for(int i=0;i<c_set.size();i++)
	{
		if(ret[c_set[i]])
			return s[i];
	}

	/*for(int i=0;i<len;i++)
	{	
		if(v[i]) 
		{
			return s[i];
		}
	}*/
	return '*';
}
