/*
    run check in projectionFactor 

*/

#include "projection_quat.h" 
#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <fstream>

using namespace std; 
using namespace QUATERNION_VIO; 

#define SQ(x) ((x)*(x))

static double para_pose[2][7]; 
static double ex_para_pose[1][7]; 
static double feat_dis[10000][1];     

namespace Eigen{
    typedef Matrix<double, 6, 1> Vector6d; // roll, pitch, yaw, x, y, z 
    typedef Matrix<double, 7, 1> Vector7d; // x, y, z, qx, qy, qz, qw
}

struct ip_M
{
    typedef enum{NO_DEPTH =0, DEPTH_MES, DEPTH_TRI, INVALID} DPT_TYPE;
    float ui, vi, uj, vj, s, sj; // s responds to Xi = [ui,vi,1] * si
    int ind;
    DPT_TYPE v; 
};

void test();
void test_ceres(); 

string log_file("iprelations.log"); 
void read_iprelation_log(vector<ip_M>& vip, string log_file); 
void random_iprelation(vector<ip_M>& vip, Eigen::Matrix<double, 7, 1>& pose); 
void vo_pnp(vector<ip_M>& vip, Eigen::Matrix<double, 7, 1>& );

void test_plane(); 

int main(int argc, char* argv[])
{
    // test(); 
    
    if(argc >= 2)
	log_file = string(argv[1]); 

    test_ceres(); 
    // test_plane(); 
    return 1; 
}

void test_ceres()
{
    // 1. read features 
    vector<ip_M> vip ;
    read_iprelation_log(vip, log_file); 
    if(0)
    {
	Eigen::Matrix<double, 7, 1> pose; 
	pose << 0.2, 0.5, -0.3, 0.23, 0.1, -0.3, sqrt(1-SQ(0.23)-SQ(0.1)-SQ(0.3));
	cout<<"true pose: "<<endl<<pose<<endl;
	random_iprelation(vip, pose);    
    }
    Eigen::Matrix<double, 7, 1> pnp_vo;
    vo_pnp(vip, pnp_vo);
    cout<<"pnp pose: "<<endl<<pnp_vo<<endl;

     // 2. ceres 
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    // these variables needs to be global or static 
    Eigen::Matrix<double, 7, 1> inipose, tmppose; 
    inipose<< 0, 0, 0, 0, 0, 0, 1; 
    tmppose<< 1, 1, 1, 0, 0, 0, 1;
    int N = vip.size(); 

    cout <<"start to construct structure! N = "<<N<<endl; 
    {
	ceres::LocalParameterization *local_param_1 = new PoseLocalPrameterization; 
	cout <<"local_param_1->GlobalSize(): "<<local_param_1->GlobalSize()<<endl; 
	problem.AddParameterBlock(para_pose[0], 7, local_param_1); 
	cout <<"after add parameter block "<<endl; 
	problem.SetParameterBlockConstant(para_pose[0]);
    }
    {
	cout<<"what? arrive here?"<<endl;
	ceres::LocalParameterization *local_param_2 = new PoseLocalPrameterization; 
	cout <<"local_param_2->GlobalSize(): "<<local_param_2->GlobalSize()<<endl; 
	problem.AddParameterBlock(para_pose[1], 7, local_param_2);
    }
    {
	ceres::LocalParameterization *local_param_3 = new PoseLocalPrameterization; 
	problem.AddParameterBlock(ex_para_pose[0], 7, local_param_3); 
	problem.SetParameterBlockConstant(ex_para_pose[0]); 
    }


    for(int i=0; i<7; i++)
    {
	para_pose[0][i] = inipose[i]; 
	para_pose[1][i] = inipose[i]; // pnp_vo[i]; // pose[i]; // tmppose[i]; 
	ex_para_pose[0][i] = inipose[i]; 
    }

    // problem.AddParameterBlock(para_pose[0], 6); 
    
    const int INIT_DIS = 20; 

    for(int i=0; i<N; i++)
    {
	ip_M& pt = vip[i]; 
	Eigen::Vector3d p1(pt.ui, pt.vi, 1.); 
	Eigen::Vector3d p2(pt.uj, pt.vj, 1.); 
	Eigen::Vector3d np1 = p1/p1[2]; 
	Eigen::Vector3d np2 = p2/p2[2];
	Eigen::Vector6d J; 

	if(pt.v == ip_M::NO_DEPTH)
	{
	    feat_dis[i][0] = INIT_DIS; 
	    ProjectionFactor_Y2 * f= new ProjectionFactor_Y2(np1, np2); 
	    problem.AddResidualBlock(f, loss_function, para_pose[0], para_pose[1], ex_para_pose[0], feat_dis[i]); 
	}else if(pt.v == ip_M::DEPTH_MES || pt.v == ip_M::DEPTH_TRI)
	{
//	    np1[2] = pt.s; 
//	    ProjectionFactor_Y3 * f3 = new ProjectionFactor_Y3(np1, np2); 
//	    problem.AddResidualBlock(f3, loss_function, para_pose[0]); 
//
//	    ProjectionFactor_Y4 * f4 = new ProjectionFactor_Y4(np1, np2); 
//	    problem.AddResidualBlock(f4, loss_function, para_pose[0]); 
	    feat_dis[i][0] = 1./pt.s; 
	    ProjectionFactor * f = new ProjectionFactor(np1, np2); 
	    f->sqrt_info = 240 * Eigen::Matrix2d::Identity(); 
	    problem.AddResidualBlock(f, loss_function, para_pose[0], para_pose[1], ex_para_pose[0], feat_dis[i]);
	    problem.SetParameterBlockConstant(feat_dis[i]);
	}	
    }   

    ceres::Solver::Options options; 
    options.linear_solver_type = ceres::DENSE_QR; // ceres::DENSE_SCHUR; 
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT; // ceres::DOGLEG; 
    // options.minimizer_progress_to_stdout = true; 
    options.max_num_iterations = 150; 
    ceres::Solver::Summary summary; 
    ceres::Solve(options, &problem, &summary); 
    std::cout<<summary.BriefReport()<<endl;

    Eigen::Vector7d vo_p; 
    for(int i=0; i<7; i++)
	vo_p[i] = para_pose[1][i]; 
    
    Eigen::Quaterniond q(vo_p[6], vo_p[3], vo_p[4], vo_p[5]); 
    q.normalize(); 
    vo_p[3] = q.x(); vo_p[4] = q.y(); vo_p[5] = q.z(); vo_p[6] = q.w();
    cout<<"vo_p: "<<endl<<vo_p<<endl;
    
    /*
    cout <<"pose0: "<<endl;
    for(int i=0; i<7; i++)
	cout <<para_pose[0][i]<<" ";
    cout<<endl;
    
    cout <<"paraPose: "<<endl;
    for(int i=0; i<7; i++)
	cout <<ex_para_pose[0][i]<<" ";
    cout<<endl;
    */

    return ; 

}

void test_plane()
{
    Eigen::Matrix<double, 4, 1> l_plane(-1, 0.1, 0.2, 5); 
    Eigen::Matrix<double, 4, 1> g_plane(-1.1, 0.2, 0.3, 5.4);
    PlaneFactor_P1 * f = new PlaneFactor_P1(g_plane, l_plane);
    double ** para = new double*[1]; 
    para[0] = new double[7]; // pose_i 
    double qx =  0.1; 
    double qy =  -0.2; 
    double qz =  0.5; 
    double qw = sqrt(1 - qx*qx - qy*qy - qz*qz); 

    Eigen::Vector3d Pi(1, -2, 3); 
    para[0][0] = Pi(0); para[0][1] = Pi(1); para[0][2] = Pi(2); 
    para[0][3] = qx; para[0][4] = qy; para[0][5] = qz; para[0][6] = qw; 
    f->check(para); 
    
}

void test()
{
    Eigen::Vector3d pts_i(3, 2, 1.); 
    Eigen::Vector3d pts_j(20, 20, 1.); 
    double ** para = new double*[4]; 
    double qx = 0.1; 
    double qy = -0.2; 
    double qz = 0.5; 
    double qw = sqrt(1 - qx*qx - qy*qy - qz*qz); 

    para[0] = new double[7]; // pose_i 
    Eigen::Vector3d Pi(1, 2, 3); 
    Eigen::Quaterniond Qi(qw, qx, qy, qz); 
    para[0][0] = 1; para[0][1] = 2; para[0][2] = 3; 
    para[0][3] = qx; para[0][4] = qy; para[0][5] = qz; para[0][6] = qw; 
    
    para[1] = new double[7]; // pose_j
    para[1][0] = 1.2; para[1][1] = 2.2; para[1][2] = 3.2; 
    qx+=0.1; qy+=0.1; 
    qw = sqrt(1 - qx*qx - qy*qy - qz*qz); 
    Eigen::Vector3d Pj(1.2, 2.2, 3.2); 
    Eigen::Quaterniond Qj(qw, qx, qy, qz); 
    para[1][3] = qx; para[1][4] = qy; para[1][5] = qz; para[1][6] = qw; 
    
    para[2] = new double[7]; // T_I2C
    Eigen::Vector3d tic(0.1, -0.2, 0.05); 
    Eigen::Quaterniond qic(0.1, 0.2, -0.1, sqrt(1-0.1*0.1 - 0.2*0.2 - 0.1*0.1)); 
    para[2][0] = tic(0); para[2][1] = tic(1); para[2][2] = tic(2); 
    para[2][3] = qic.x(); para[2][4] = qic.y(); para[2][5] = qic.z(); para[2][6] = qic.w(); 
    
    para[3] = new double[1]; // depth 
    double inv_dep = 0.5; 
    para[3][0] = 0.5;

    // para[4] = new double[1]; // count
    // para[4][0] = 7.0;
    double count = 7.0; 

    Eigen::Vector3d pts_camera_i = pts_i/inv_dep; 
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic; 
    Eigen::Vector3d pts_w = Qi*pts_imu_i + Pi; 
    Eigen::Vector3d pts_imu_j = Qj.inverse()*(pts_w - Pj); 
    Eigen::Vector3d pts_cam_j = qic.inverse() * (pts_imu_j - tic); 
    double dep_j = pts_cam_j.z();
    Eigen::Vector3d noise(0.01, 0.02, -0.01); 
    pts_j = (pts_cam_j/dep_j); 
    pts_j += noise; 
    ProjectionFactor_Y2 * f1 = new ProjectionFactor_Y2(pts_i, pts_j); 
    cout <<"f1->size: "<<f1->parameter_block_sizes().size()<<endl; 
    // ProjectionFactor::sqrt_info.setIdentity();
    // WeightProjectionFactor * f = new WeightProjectionFactor(pts_i, pts_j, count); 
    // WeightProjectionFactor::sqrt_info.setIdentity(); 
    
    f1->check(para); 
    return ;
}

void vo_pnp(vector<ip_M>& vip, Eigen::Matrix<double, 7, 1>& pose)
{
    // compute PnP error 
    Eigen::MatrixXd SRC_PTS;
    Eigen::MatrixXd DPT_PTS;
    int N = vip.size()/2;
    SRC_PTS.resize(3, N); 
    DPT_PTS.resize(3, N);
    for(int i=0; i<N; i++)
    {
	ip_M& m = vip[i];
	DPT_PTS.col(i) = Eigen::Vector3d(m.ui*m.s, m.vi*m.s, m.s); 
	SRC_PTS.col(i) = Eigen::Vector3d(m.uj*m.sj, m.vj*m.sj, m.sj);
    } 
    
    Eigen::Matrix<double, 4, 4> T = Eigen::umeyama(SRC_PTS, DPT_PTS, false); // scaling = false 
    Eigen::Matrix<double, 3, 3> R = T.block(0, 0, 3, 3); 
    Eigen::Matrix<double, 3, 1> t = T.block(0, 3, 3, 1); 
    pose(0) = t(0); pose(1) = t(1); pose(2) = t(2); 
    Eigen::Quaterniond q(R); 
    q.normalize(); 
    pose(3) = q.x(); pose(4) = q.y(); pose(5) = q.z(); pose(6) = q.w(); 
    return ;
}

void random_iprelation(vector<ip_M>& vip, Eigen::Matrix<double, 7, 1>& pose)
{
    // generate two point sets 
    static std::random_device rd{};
    static std::mt19937 gen{rd()};

    static std::uniform_real_distribution<> uni_dis(-1.0, 1.0);
    static std::uniform_real_distribution<> uni_z(1., 5.); 
    static std::normal_distribution<> noise{0,0.02};
    static std::uniform_real_distribution<> rangle(-5., 5.); 
    static std::uniform_real_distribution<> rdis(-0.2, 0.2); 
    
    int NUM = 40;
    Eigen::Quaterniond q(pose(6), pose(3), pose(4), pose(5));
    Eigen::Vector3d t(pose(0), pose(1), pose(2));

    for(int k = 0; k<NUM;)
    {
	Eigen::Vector3d p1; 
	p1 << uni_dis(gen), uni_dis(gen), 1.0; 
	p1 = p1 * (uni_z(gen) + 1.); 
	Eigen::Vector3d p0 = q * p1 + t; 
	p0(0) += noise(gen); 
	p0(1) += noise(gen); 
	p0(2) += noise(gen); 
	if(p1(2) <= 0.3 || p0(2) <= 0.3)
	    continue; 
	ip_M m; 
	m.ui = p0.x()/p0.z(); m.vi = p0.y()/p0.z(); m.s = p0.z(); 
	m.uj = p1.x()/p1.z(); m.vj = p1.y()/p1.z(); m.sj = p1.z(); 
	if(k > NUM/2)
	    m.v = ip_M::NO_DEPTH;
	else
	    m.v = ip_M::DEPTH_MES;
	k++;
	vip.push_back(m);
    }
}

void read_iprelation_log(vector<ip_M>& vip, string log_file)
{
    ifstream inf(log_file.c_str());
    while(!inf.eof())
    {
	string s;
	getline(inf, s);
	if(!s.empty())
	{
	    float v; 
	    ip_M ip; 
	    stringstream ss; 
	    ss << s; 
	    ss >> ip.ui >> ip.vi >> ip.uj >> ip.vj >> ip.s >> v; 
	    if(v == 0.){ ip.v = ip_M::NO_DEPTH; }
	    else if(v == 1.){ip.v = ip_M::DEPTH_MES; }
	    else if(v == 2.){ip.v = ip_M::DEPTH_TRI; }
	    else ip.v = ip_M::INVALID; 
	    vip.push_back(ip); 
	}
    }
} 

