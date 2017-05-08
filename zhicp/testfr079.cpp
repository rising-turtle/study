#include <iostream>
#include <fstream>
#include <vector>
#include "ZHIcp_Warpper.h"
#include "mrpt/slam.h"
#include "mrpt/poses.h"
#include <string.h>
#include <string>
#include <stdlib.h>

static const int sick_num = 361; 

typedef float REAL;

bool getScan(string file, vector< vector<REAL> > & r );
bool getOdo(string file, vector<vector<REAL> > & odo);
void projectBeam(vector<REAL>& r, vector<REAL>& rx, vector<REAL>& ry);

using mrpt::poses::CPose2D;

int testfr1(int argc, char* argv[]);
int testfr2(int argc, char* argv[]);

int main(int argc, char* argv[])
{   
    testfr2(argc, argv);   
    return 1;
}

int testfr2(int argc, char* argv[])
{
    string icp_odo("icp_result.txt");
    string icp_scan_pre("icp_scan_prev.txt");
    string icp_scan_cur("icp_scan_curr.txt");
    if(argc >= 4)
    {
        icp_odo = argv[1]; 
        icp_scan_pre = argv[2];
        icp_scan_cur = argv[3];
    }
    
    vector<vector<REAL> > r_pre;
    vector<vector<REAL> > r_cur;
    vector<vector<REAL> > odo;
    if(!getOdo(icp_odo, odo))
        return 0;
    if(!getScan(icp_scan_pre, r_pre)) 
        return 0;
    if(!getScan(icp_scan_cur, r_cur))
        return 0;
    CICPWarpper icp_wrapper;
    icp_wrapper.setDebugWindowOn();
    //icp_wrapper.setDebugWindowOff();
    
    ofstream ouf("icp_test.txt");
    vector<REAL> rx, ry, ax, ay; 
    double input[3], output[3];
    assert(r_pre.size() == r_cur.size()); // == odo.size());
    assert(r_pre.size() == odo.size());
    CPose2D iniPose(4.293107, -1.743848, -3.819356);

    for(int i=0; i< odo.size(); i++)
    {
        projectBeam(r_pre[i], rx, ry);
        projectBeam(r_cur[i], ax, ay);
        input[0] = odo[i][0]; 
        input[1] = odo[i][1]; 
        input[2] = odo[i][2];
        float goodness = icp_wrapper.ICPMatch(&rx[0], &ry[0], &ax[0], &ay[0], sick_num, input, output); 
        cout<<"match "<<i-1<<" with "<<i<<" goodness = "<<goodness<<endl; 
        cout<<"result: "<<output[0]<<" "<<output[1]<<" "<<output[2]<<endl;
        CPose2D rel_Pose = CPose2D(output[0], output[1], output[2]); 
        iniPose = iniPose + rel_Pose;

        ouf<<std::fixed<<input[0]<<" "<<input[1]<<" "<<input[2]<<" ";
        ouf<<iniPose[0]<<" "<<iniPose[1]<<" "<<iniPose[2]<<" "<<goodness<<endl;
        // ouf<<output[0]<<" "<<output[1]<<" "<<output[2]<<" "<<goodness<<endl;
    }
    return 1;
}

int testfr1(int argc, char* argv[])
{
    string icp_scan("icp_scan.txt"); 
    string icp_odo("icp_result.txt"); 
    if(argc >=3)
    {
        icp_scan = argv[1]; 
        icp_odo = argv[2];
    }
    vector<vector<REAL> > r; 
    vector<vector<REAL> > odo;
    if(!getOdo(icp_odo, odo))
        return 0;
    if(!getScan(icp_scan, r)) 
        return 0;
    CICPWarpper icp_wrapper;
    // icp_wrapper.setDebugWindowOn();
    icp_wrapper.setDebugWindowOff();
    
    ofstream ouf("icp_test.txt");

    vector<REAL> rx, ry, ax, ay; 
    double input[3], output[3];
    CPose2D last_p; 
    CPose2D curr_p;
    for(int i=0; i<icp_odo.size(); i++)
    {
        if(i==0)
        {
            projectBeam(r[i], rx, ry); 
            last_p = CPose2D(odo[i][0], odo[i][1], odo[i][2]);
            continue; 
        }
        projectBeam(r[i], ax, ay); 
        curr_p = CPose2D(odo[i][0], odo[i][1], odo[i][2]);
        //  CPose2D rel = inverseComposeFrom(last_p,curr_p); 
        CPose2D rel = last_p - curr_p;
        input[0] = rel[0]; input[1] = rel[1]; input[2] = rel[2];
        float goodness = icp_wrapper.ICPMatch(&rx[0], &ry[0], &ax[0], &ay[0], sick_num, input, output); 
        cout<<"match "<<i-1<<" with "<<i<<" goodness = "<<goodness<<endl; 
        cout<<"result: "<<output[0]<<" "<<output[1]<<" "<<output[2]<<endl;
        
        ouf<<std::fixed<<curr_p[0]<<" "<<curr_p[1]<<" "<<curr_p[2]<<" ";
        ouf<<output[0]<<" "<<output[1]<<" "<<output[2]<<" "<<goodness<<endl;

        rx.swap(ax); 
        ry.swap(ay);
        last_p = curr_p;
    }
    return 1;
}


bool getScan(string file, vector< vector<REAL> > & r )
{
    ifstream inf(file.c_str());
    if(!inf.is_open())
    {
        cout<<"failed to open "<<file<<endl;
        return false;
    }
    char line[4096*8];
    string delim(" ,\t");
    int num = sick_num; 
    vector<REAL> cur_r(num);
    while(inf.getline(line, sizeof(line)))
    {
        cur_r[0] = atof(strtok(line, delim.c_str()));
        for(int i=1; i<num; i++) 
            cur_r[i] = atof(strtok(NULL, delim.c_str()));
        r.push_back(cur_r);
    }
    cout<<"succeed to load "<<r.size()<<" scans!"<<endl;
    return true;
}

bool getOdo(string file, vector<vector<REAL> > & odo)
{
    ifstream inf(file.c_str());
    if(!inf.is_open())
    {
        cout<<"failed to open "<<file<<endl;
        return false;
    }
    char line[4096*8];
    string delim(" ,\t");
    int num = 3; 
    vector<REAL> cur_o(num);
    while(inf.getline(line, sizeof(line)))
    {
        cur_o[0] = atof(strtok(line, delim.c_str()));
        for(int i=1; i<num; i++) 
            cur_o[i] = atof(strtok(NULL, delim.c_str()));
        odo.push_back(cur_o);
    }
    cout<<"succeed to load "<<odo.size()<<" odos!"<<endl;
    return true;

}
void projectBeam(vector<REAL>& r, vector<REAL>& rx, vector<REAL>& ry)
{
    static REAL minAngle = -M_PI/2.; 
    static REAL maxAngle = M_PI/2.;
    static int N_BEAMS = sick_num;
    static bool ini = false; 
    static vector<REAL> cosA(N_BEAMS); 
    static vector<REAL> sinA(N_BEAMS); 
    if(!ini)
    {
        REAL incr = (maxAngle - minAngle)/(REAL)(N_BEAMS-1);
        REAL angle;
        for(int i=0; i< N_BEAMS; i++)
        {
            angle = minAngle + incr*i;
            cosA[i] = cosf(angle); 
            sinA[i] = sinf(angle);
        }
    }
    rx.resize(N_BEAMS); 
    ry.resize(N_BEAMS); 
    for(int i=0; i<N_BEAMS; i++)
    {
        rx[i] = r[i]*cosA[i]; 
        ry[i] = r[i]*sinA[i];
    }
    return ;
}

