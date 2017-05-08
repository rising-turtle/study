#include "preheader.h"

namespace{

	string wordreplace(string content, map<string,string>& rule)
	{
		string ret;
		for(int i=0;i<content.size();i++){
			if(content[i]=='$'){
				string val;
				if(i+1<content.size() && content[i+1]=='$'){
					while(content[++i]=='$') val.push_back('$');
					ret+=val;
					i--;
				}
				else{
					while(++i<content.size() && content[i]!='$') val.push_back(content[i]);
					map<string,string>::iterator it=rule.find(val);
					if(it==rule.end()){
						cout<<"No matched words for: "<<val;
					}else{
						ret+=it->second;	
					}
				}
			}else ret.push_back(content[i]);
		}
		return ret;
	}

	void wordrun(string input, string rulef){
		
		ifstream inf(input.c_str());
		ifstream inr(rulef.c_str());

		if(!inf.is_open() || !inr.is_open()){
			cout<<"failed to open file!"<<endl;
			return ;
		}

		// mapping the rule
		map<string,string > rule;
		char buf[1024],word[100],value[100];
		string keyword, keyvalue;
		while(inr.getline(buf,1024)){
			sscanf(buf,"%s : %s",word,value);
			keyvalue = value;
			keyword = word;
			if(keyword.size() <= 0 || keyvalue.size()<=0)
			{
				cout<<"wrong value for rule!"<<endl;
				return ;
			}
			rule.insert(make_pair(keyword,keyvalue));
		}
		
		// translate the input string
		string content;
		while(inf.getline(buf,1024)){
			//strcat(content.c_str(),inf);
			content+=string(buf);
		}
		
		cout<<"content: "<<content<<endl;
		cout<<"RULES: "<<endl;
		for(map<string,string>::iterator it = rule.begin();it!=rule.end();it++){
			cout<<it->first<<" : "<<it->second<<endl;
		}
		cout<<"result: "<<wordreplace(content,rule)<<endl;
		inf.close();
		inr.close();
	}

}

void test_wordreplace(){
	string input("./files/input.txt");
	string rule("./files/rule.txt");
	wordrun(input,rule);
}