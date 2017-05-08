#include "preheader.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <sstream>

using namespace std;
namespace {
	
	// if Bi>Bj && Ci<Cj then j is invalid item
	void eraseInvalidItem(vector<pair<int,int> >& out)
	{
		vector<pair<int,int>>::iterator it = out.begin();
		while(it!=out.end()){
			bool Is_It_deleted = false;
			vector<pair<int,int>>::iterator it_nxt = it+1;
			while(it_nxt!=out.end()){
				if(it_nxt->first<=it->first && it_nxt->second>=it->second) // erase it_nxt
				{
					it_nxt = out.erase(it_nxt);
					continue;
				}
				if(it_nxt->first>=it->first && it_nxt->second<=it->second) // erase it
				{
					Is_It_deleted = true;
					break;
				}
					it_nxt++;
			}
			if(Is_It_deleted){
				it = out.erase(it);
				continue;
			}
			it++;
		}

	}

	void calValidCombine(vector< vector<pair<int,int> > >& machines, vector<pair<int,int> >& out)
	{
		for(int i=0;i<machines.size();i++)
		{
			vector<pair<int,int> > temp;
			for(int j=0;j<machines[i].size();j++)
			{
				if(out.size()==0)
				{
					temp.push_back(make_pair(machines[i][j].first,machines[i][j].second));
				}
				for(int k=0;k<out.size();k++)
				{
					pair<int,int> cur = out[k];
					cur.first=machines[i][j].first>=cur.first?cur.first:machines[i][j].first;
					cur.second+=machines[i][j].second;
					temp.push_back(cur);
				}	

			}
			eraseInvalidItem(temp);
			out.swap(temp);
		}
	}

	double calMaxBP(vector< vector<pair<int,int> > >& machines)
	{
		vector<pair<int,int> > m_com;
		calValidCombine(machines,m_com);
		double max_score = -1.0;
		for(int i=0;i<m_com.size();i++){
			pair<int,int>& cur_p = m_com[i];
			double tmp_score = (double)((double)cur_p.first/(double)cur_p.second);
			if(tmp_score>max_score) max_score = tmp_score;
		}
		return max_score;
	}
	
	void display(vector< vector<pair<int,int> > >& machines)
	{
		for(int i=0;i<machines.size();i++)
		{
			for(int j=0;j<machines[i].size();j++){
				cout<<machines[i][j].first<<" "<<machines[i][j].second<<" ";
			}
			cout<<endl;
		}
	}

};

void test1018(string file_n){
	ifstream inputf(file_n.c_str());
	if(!inputf.is_open()){
		cout<<"failed to open file!"<<endl;
		return ;
	}
	char buf[1024];
	char p_buf[100];
	inputf.getline(buf,1024);
	int cases,n_machine;
	sscanf(buf,"%d %d",&cases,&n_machine);
	for(int i=0;i<cases;i++){
		vector<vector<pair<int,int> > > machines;
		vector<pair<int,int> > e_mach;
		for(int j=0;j<n_machine;j++){
			inputf.getline(buf,1024);
			int n_;
			stringstream tmp;
			tmp<<buf;
			tmp>>n_;
			for(int k=0;k<n_;k++){
				int p,b;
				tmp>>p;
				tmp>>b;
				e_mach.push_back(make_pair(p,b));
			}
			machines.push_back(e_mach);
			e_mach.clear();
		}
		//display(machines);
		cout<<"max: "<<setprecision(3)<<calMaxBP(machines)<<endl;
	}
}