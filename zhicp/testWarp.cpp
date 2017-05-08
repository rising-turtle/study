#include "ZHIcp_Warpper.h"
#include <stdio.h>
#include "stdlib.h"
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
using namespace std;

string fdir("/home/lyxp/work/mkproj/SVN/PFG/trunk/src/zhicp/data");
string fname("matched_nodes.log");
void testICP_Warp();
void testICP_Quality();
bool getScan(string, vector<float>&, vector<float>&);

int main(int argc, char* argv[])
{
	if(argc >= 2)
	{
		fdir = string(argv[1]);
	}
	testICP_Warp();
	// testICP_Quality();
	return 0;
}

void testICP_Quality(){
	string file = fdir+"/"+fname;
	cout<<file<<endl;
	FILE* f = fopen(file.c_str(),"rb");
	if(f == 0){
		cout<<"failed to open file!"<<endl;
		return ;
	}
	CICPWarpper icp_warpper;
	int n1,n2,num;
	double px,py,pth;
	vector<float> rx,ry;
	vector<float> ax,ay;
	
	// test icp match quality
	ofstream icp_quality("not_matched_quality.log");
	int iter;
	float good;
	float quality;
	vector<int> ref_id;
	vector<int> cur_id;
	

	while(!feof(f)){
		fscanf(f,"%i %i %i %lf %lf %lf\n",&n1,&n2,&num,&px,&py,&pth);
		cout<<n1<<" "<<n2<<" "<<px<<" "<<py<<" "<<pth<<endl;
		ref_id.push_back(n1);
		cur_id.push_back(n2);
	}
	fclose(f);

	int ntimes = 30;
	int t = 0;
	srand(time(NULL));
	while(t++<ntimes){
		int ref = rand()%ref_id.size();
		int cur = rand()%cur_id.size();
		if(ref == cur) continue;
		cout<<"match between "<<ref<<" and "<<cur<<endl;
		stringstream s1,s2;
		s1<<fdir<<"/"<<ref_id[ref]<<".log";
		s2<<fdir<<"/"<<cur_id[cur]<<".log";
		if(!getScan(s1.str(),rx,ry) || !getScan(s2.str(),ax,ay) ){
			cout<<"failed to read scan "<<ref_id[ref]<<" and "<<cur_id[cur]<<" !"<<endl;
			return ;
		}
		icp_warpper.ICPMatch(rx,ry,ax,ay,0,0,0);
		icp_warpper.getMatchQuality(iter,good,quality);
		cout<<"iter: "<<iter<<" good: "<<good<<" quality: "<<quality<<endl;
		icp_quality<<iter<<" "<<good<<" "<<quality<<endl;
	}
}

void testICP_Warp(){
	string file = fdir+"/"+fname;
	cout<<file<<endl;
	FILE* f = fopen(file.c_str(),"rb");
	if(f == 0){
		cout<<"failed to open file!"<<endl;
		return ;
	}
	CICPWarpper icp_warpper;
	int n1,n2,num;
	double px,py,pth;
	vector<float> rx,ry;
	vector<float> ax,ay;
	
	ofstream pair_result("results.log");
	// test icp match quality
	ofstream icp_quality("matched_quality.log");
	int iter;
	float good;
	float quality;
	
	double pinit[3] ={0,};
	double pout[3] = {0,};

	while(!feof(f)){
		fscanf(f,"%i %i %i %lf %lf %lf\n",&n1,&n2,&num,&px,&py,&pth);
		cout<<n1<<" "<<n2<<" "<<px<<" "<<py<<" "<<pth<<endl;
		stringstream s1,s2;
		s1<<fdir<<"/"<<n1<<".log";
		s2<<fdir<<"/"<<n2<<".log";
		// pair_result<<n1<<" "<<n2<<" "<<px<<" "<<py<<" "<<pth<<endl;
		if(!getScan(s1.str(),rx,ry) || !getScan(s2.str(),ax,ay) ){
			cout<<"failed to read scan "<<n1<<" and "<<n2<<" !"<<endl;
			return ;
		}
		// float good1 = icp_warpper.ICPMatch(&rx[0],&ry[0],&ax[0],&ay[0],ax.size(),pinit,pout);
		
		icp_warpper.ICPMatch(rx,ry,ax,ay,px,py,pth);
		icp_warpper.getMatchQuality(iter,good,quality);

		// cout<<"iter: "<<iter<<" good: "<<good<<" quality: "<<quality<<endl;
		icp_quality<<iter<<" "<<good<<" "<<quality<<endl;
		// pair_result<<(icp_warpper.m_pTrans->cov)<<endl;
	}
	fclose(f);
}

bool getScan(string file, vector<float>& fx, vector<float>& fy)
{
	FILE* f = fopen(file.c_str(),"rb");
	if(f==0) {
		cout<<"failed to open file: "<<file<<endl;
		return false;
	}
	float x,y;
	fx.clear();
	fy.clear();
	cout<<"start read file: "<<file<<endl;

	while(!feof(f)){
		fscanf(f,"%f %f\n",&x,&y);
		fx.push_back(x);
		fy.push_back(y);
	}
	fclose(f);
	cout<<"finish read file: "<<file<<endl;
	return true;
}
