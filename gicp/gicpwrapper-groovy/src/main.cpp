#include "globaldef.h"
#include "ros/ros.h"
#include "qtros.h"
#include <iostream>
#include <fstream>
// #include <random>
#include "OctoVizWrapper.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <octomap/Pointcloud.h>
#include <QtGui>
#include <QApplication>
#include <QObject>
// #include <octovis/ViewerGui.h>
#include "GicpWrapper.h"
#include "FileReader.h"
#include "RandomImpl.h"
#include <octomap/ScanGraph.h>
#include <vector>
#include "bag_listener.h"
#include "ColorOctreeImpl.h"

using namespace std;

void testGicp1(COctVizWrapper&);
void testGicp2(COctVizWrapper&);
void testGicp3(COctVizWrapper&);
void testGT1(COctVizWrapper&); // obtain ground truth for the conference room
void readGraph(COctVizWrapper&, const char* );
void fromGraph2Pcd(const char*); // convert graph to a pcd 
void readPose(const char* fs, octomap::pose6d& pose);
void readLog(const char* logf, vector<string>& pcdf, vector<string>& posef);

int main(int argc, char* argv[])
{
    // ros::init(argc,argv,"gicp");
    // ros::NodeHandle nh;
    QtROS qtRos(argc, argv, "gicp");
    QApplication app(argc, argv);
    COctVizWrapper viz;
    CBagListener listen;
    if(argc >= 2)
    {
        // readGraph(viz, argv[1]);   
        fromGraph2Pcd(argv[1]);
    }else
    {
        point_cloud_ptr pc(new point_cloud);
        pcl::io::loadPCDFile("person.pcd",*pc);
        cout<<"point cloud width: "<<pc->width<<" height: "<<pc->height<<" size: "<<pc->points.size()<<endl;
       // octomap::point3d ini_p;
       // octomap::Pointcloud* oct_pc(new octomap::Pointcloud);
       // fromPCL2OctoPC(*pc, *oct_pc, ini_p);
        viz.addPC(*pc);
        // testGicp2(viz);
        // testGT1(viz);
        // testGicp1(viz);
        // testGicp3(viz);
        // QObject::connect(&listen, SIGNAL(sendCameraImgs(sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfoConstPtr)),&viz, SLOT(receCameraImgs(sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfoConstPtr)),Qt::DirectConnection);
        // QObject::connect(&listen, SIGNAL(sendColorPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)), &viz, SLOT(receColorPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)), Qt::DirectConnection);
    }
    viz.show();

    //If one thread receives a exit signal from the user, signal the other thread to quit too
    QObject::connect(&app, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));
    QObject::connect(&qtRos, SIGNAL(rosQuits()), &app, SLOT(quit()));

    qtRos.start();// Run main loop.
    app.exec();
    return 0;
}

void normal_noise(octomap::pose6d& pose, double sigma_t, double sigma_r)
{
    static CRandomImpl rng;
    pose.x() += rng.normal(sigma_t);
    pose.y() += rng.normal(sigma_t);
    pose.z() += rng.normal(sigma_t);

    octomath::Vector3 euler = pose.rot().toEuler();
    euler(0) += rng.normal(sigma_r);
    euler(1) += rng.normal(sigma_r);
    euler(2) += rng.normal(sigma_r);

    pose.rot() = octomath::Quaternion(euler);
}

void calSqrErrorPose(octomap::pose6d& p1, octomap::pose6d& p2, double& t, double& r)
{
    octomap::pose6d trans = p1.inv()*p2;
    float x = trans.x();
    float y = trans.y();
    float z = trans.z();
    
    float roll = R2D(trans.roll());
    float pitch = R2D(trans.pitch());
    float yaw = R2D(trans.yaw());

    t = SQ(x) + SQ(y) + SQ(z);
    r = SQ(roll) + SQ(pitch) + SQ(yaw);
}

void testGicp1(COctVizWrapper& viz)
{
    vector<string > pcdf;
    vector<string > posef;
    readLog("record.log", pcdf, posef);
    
    int times = 50;
    int step = 5;
    int adj = 2;
    double sigma_t = 0.04;
    double sigma_r = D2R(4);
    point_cloud_ptr targetPC(new point_cloud);
    point_cloud_ptr queryPC(new point_cloud);

    CGicpWrapper gicp;

    vector<octomap::pose6d> gt_pose;
    vector<octomap::pose6d> gicp_pose;
    octomap::pose6d targetPose, queryPose, transPose, tmp, gt_trans, ftrans;
    Eigen::Matrix4f iniTrans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f finalTrans = Eigen::Matrix4f::Identity();
    double t_err, r_err, time_cost;
    ofstream err_log("4cm4d.log");
    for(int i=0, j=0; i< times; i++, j+=step)
    {
        pcl::io::loadPCDFile(pcdf[j],*targetPC);
        pcl::io::loadPCDFile(pcdf[j+adj], *queryPC);
        
        readPose(posef[j].c_str(),targetPose);
        readPose(posef[j+adj].c_str(),queryPose);

        /*if(i==0)
        {
            gt_pose.push_back(targetPose);
            gicp_pose.push_back(targetPose);
        }*/
        // normal noise with (sigma_t, sigma_r) to trans
        transPose = targetPose.inv()*queryPose; // ground truth trans
        gt_trans = transPose;
        ROS_INFO_STREAM("ground truth: "<<transPose);
        normal_noise(transPose, sigma_t, sigma_r);
        // ROS_INFO_STREAM("with noise: "<<transPose);
        
        // gicp trans
        fromPose6d2Eigen(iniTrans, transPose);
        StartTiming();
        gicp.match_it(*targetPC, *queryPC, finalTrans, iniTrans);
        time_cost = StopTiming();
        fromEigen2Pose6d(finalTrans, tmp);
        ftrans = transPose * tmp;
        ROS_INFO_STREAM("gicp trans: "<<ftrans);
        
        calSqrErrorPose(gt_trans, ftrans, t_err, r_err);
        err_log<<t_err<<"\t"<<r_err<<"\t"<<time_cost<<endl;
        
        // ROS_INFO_STREAM("gicp pose: "<<tmp);
        // tmp = gicp_pose[gicp_pose.size()-1]*ftrans;
        gt_pose.push_back(gt_trans);
        gicp_pose.push_back(ftrans);
    }   
    // record the result
    ofstream comp1("gt_log");
    ofstream comp2("gicp_log");
    for(int i=0;i<gt_pose.size();i++)
    {
        comp1<<gt_pose[i]<<endl;
        comp2<<gicp_pose[i]<<endl;
    }
    ROS_WARN("HAHA, FINISHED!");
}

void testGicp2(COctVizWrapper& viz)
{
    // 1 obtain pcs and poses
    vector<point_cloud_ptr> pcs;
    vector<octomap::pose6d> poses;
    vector<octomap::pose6d> icp_pose;
    readPcdAndPose(gl_pcd_file_dir, pcs, poses);

    // 2 GICP dead-reckon
    CGicpWrapper gicp;
    Eigen::Matrix4f iniTrans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f finalTrans = Eigen::Matrix4f::Identity();
    static octomap::ScanGraph gf;
    octomap::point3d ini_p;
    octomap::pose6d trans;
    octomap::pose6d cur_pose;
    for(int i=0;i<pcs.size();i++)
    {
        point_cloud_ptr pc = pcs[i];
        octomap::Pointcloud* oct_pc(new octomap::Pointcloud);
        fromPCL2OctoPC(*pc, *oct_pc, ini_p);
        if(i==0) // the first scan
        {
            icp_pose.push_back(poses[0]);
            gf.addNode(oct_pc, icp_pose[0]);
        }else
        {
            // match with previous scan
            gicp.match_it(*pcs[i-1],*pcs[i],finalTrans, iniTrans);
            fromEigen2Pose6d(finalTrans, trans);
            cur_pose = icp_pose[i-1]*trans;
            gf.addNode(oct_pc, cur_pose);
            gf.addEdge(gf.size()-2, gf.size()-1);
            icp_pose.push_back(cur_pose);
        }
    }
    // 3, show the result of GICP
    gf.writeBinary("../external/octomap/mapdata/conf_gicp.graph");
    viz.prepareGraph( &gf, true);
    // 4, output the trajectory
    ofstream gt_("conf_gt.log");
    ofstream gicp_("conf_gicp.log");
    if(icp_pose.size()!=poses.size())
    {
        ROS_ERROR("ERROR icp.size()!=gt.size()");
    }
    for(int i=0;i<icp_pose.size();i++)
    {
        gt_<<poses[i]<<endl;
        gicp_<<icp_pose[i]<<endl;
    }
}

void testGicp3(COctVizWrapper& viz)
{
    point_cloud_ptr targetPC(new point_cloud);
    pcl::io::loadPCDFile("person.pcd",*targetPC);
    point_cloud_ptr queryPC(new point_cloud(*targetPC));
    // pcl::io::loadPCDFile("person.pcd",*queryPC);
    CGicpWrapper gicp;
    Eigen::Matrix4f iniTrans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f finalTrans = Eigen::Matrix4f::Identity();
    octomap::pose6d t1(0.3,-0.1,1,0.2,0.3,-0.1);
    octomap::pose6d t2,t3;
    fromPose6d2Eigen(iniTrans, t1);
    gicp.match_it(*targetPC, *queryPC, finalTrans, iniTrans);
    fromEigen2Pose6d(finalTrans, t2);
    t3 = t1*t2;
    cout<<"INI: "<<t1<<endl;
    cout<<"RESULT: "<<t2<<endl;
    cout<<"INI*RES: "<<t3<<endl;
    viz.addPC(*targetPC);
    viz.addPC(*queryPC, t3, getGreen());
}
/*
void readPose(const char* fs, octomap::pose6d& pose)
{
    FILE* f = fopen(fs,"rb");
    if(f!=NULL)
    {
        double x,y,z,q0,q1,q2,q3;
        fscanf(f,"%lf %lf %lf %lf %lf %lf %lf",&x,&y,&z,&q0,&q1,&q2,&q3);
        pose = octomap::pose6d(octomath::Vector3(x,y,z), octomath::Quaternion(q0,q1,q2,q3));
        cout<<"read pose: "<<pose<<endl;
    }
    fclose(f);
}

void readPcdAndPose(const char* fs, vector<point_cloud_ptr>& pcs, vector<octomap::pose6d>& poses, int step)
{
    CFileReader files;
    files.readDir(fs);
    string pcdf;
    string posef;
    octomap::pose6d pose;
    for(int i=files.m_firstnum; i<= files.m_lastnum; i+=step)
    {
        point_cloud_ptr pc(new point_cloud);
        // 1, get pcd & pose file
        files.getPoseFile(i, posef);
        files.getPcdFile(i, pcdf);
        // 2, get point cloud & pose6d 
        pcl::io::loadPCDFile(pcdf, *pc);
        pcs.push_back(pc);
        readPose(posef.c_str(), pose);
        poses.push_back(pose);
    }
}
*/

void fromGraph2Pcd(const char* graph_file)
{
    static octomap::ScanGraph pg;
    pg.readBinary(graph_file);
    octomap::OcTree* tree = new octomap::OcTree(0.05);

    octomap::ScanGraph::iterator it;
    octomap::ScanGraph::iterator m_nextScanToAdd = pg.end();
    unsigned numScans = pg.size();
    unsigned currentScan = 1;

    point_cloud_ptr outPC(new point_cloud);
    bool first = true;
    for (it = pg.begin(); it != m_nextScanToAdd; it++) 
    {
        tree->insertPointCloud(**it, -1);
        if(first)
        {
            Eigen::Vector4f& pose = outPC->sensor_origin_;
            pose[0] = (*it)->pose.x();
            pose[1] = (*it)->pose.y();
            pose[2] = (*it)->pose.z();
            pose[3] = 1.0;
            first = false;
        }
        // fprintf(stderr, "generateOctree:: inserting scan node with %d points, origin: %.2f  ,%.2f , %.2f.\n",
        //        (unsigned int) (*it)->scan->size(), (*it)->pose.x(), (*it)->pose.y(), (*it)->pose.z()  );
        std::cout << " S ("<<currentScan<<"/"<<numScans<<") " << std::flush;
        currentScan++;
        // if(currentScan > 1) break;
    }

    int maxDepth = 16;
    for(octomap::OcTree::leaf_iterator it = tree->begin(maxDepth), end=tree->end(); it!= end; ++it) 
    {
        if(!tree->isNodeOccupied(*it))
        {
            octomap::point3d pt = it.getCoordinate();
            outPC->points.push_back(point_type(pt(0),pt(1),pt(2)));
       //     list_iterator.push_back(OcTreeVolume(it.getCoordinate(), it.getSize()));
        }
    }
    outPC->width = outPC->points.size();
    outPC->height = 1;
    pcl::io::savePCDFile("outPC.pcd", *outPC);
    cout<<"FINISHED!"<<endl;
}

void readGraph(COctVizWrapper& viz, const char* graph_file)
{
    static octomap::ScanGraph pg; // = new octomap::ScanGraph;
    pg.readBinary(graph_file);
    viz.prepareGraph(&pg, true);
}

void testGT1(COctVizWrapper& viz)
{
    CFileReader files;
    files.readDir(gl_pcd_file_dir);
    string pcdf;
    string posef;
    point_cloud_ptr pc(new point_cloud);
    octomap::pose6d pose;
    octomap::point3d ini_p;
    static octomap::ScanGraph graph;
    for(int i=files.m_firstnum; i<= files.m_lastnum; i+=5)
    {
        // 1, get pcd & pose file
        files.getPoseFile(i, posef);
        files.getPcdFile(i, pcdf);
        // 2, get point cloud & pose6d 
        pcl::io::loadPCDFile(pcdf, *pc);
        octomap::Pointcloud* oct_pc(new octomap::Pointcloud);
        fromPCL2OctoPC(*pc, *oct_pc, ini_p);
        readPose(posef.c_str(), pose);
        graph.addNode(oct_pc, pose);
        if(graph.size()>1){
            graph.addEdge(graph.size()-2,graph.size()-1);
        }
    }
    graph.writeBinary("../external/octomap/mapdata/conf_gt.graph");
    // viz.m_scanGraph = &graph;
    viz.prepareGraph(&graph, false);
    cout<<"finished testGT1"<<endl;
}

void readLog(const char* logf, vector<string>& pcdf, vector<string>& posef)
{
    char buf[1024];
    ifstream inf(logf);
    while(inf.getline(buf,1024))
    {
        pcdf.push_back(buf);
        inf.getline(buf,1024);
        posef.push_back(buf);
    }
}





