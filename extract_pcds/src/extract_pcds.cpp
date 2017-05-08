#include "extract_pcds.h"
#include <stdlib.h>
#include <pcl/registration/transforms.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cmath>

#define D2R ((M_PI)/180.)

CExtractPCDs::CExtractPCDs(int s_port, int e_port): 
s_port_(s_port),
e_port_(e_port)
{
    initTrans();
}

CExtractPCDs::~CExtractPCDs()
{

}

void CExtractPCDs::initTrans()
{
    //transform from left and right to front
    // vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > vCamToBase; // up-front, up-left, up-right, down-front, down-left, down-right
    Eigen::Affine3f affine;
    pcl::getTransformation (0, -0.61, 0, 0., 0.*D2R, 0., affine);
    vCamToBase.push_back(affine.matrix());
    pcl::getTransformation (-0.13, -0.61, -0.21, 0., -90.*D2R, 0., affine);
    vCamToBase.push_back(affine.matrix());
    pcl::getTransformation (0.16, -0.61, -0.18, 0., 90.*D2R, 0., affine);
    vCamToBase.push_back(affine.matrix());

    pcl::getTransformation (0., 0., 0, 0., 0.*D2R, 0., affine);
    vCamToBase.push_back(affine.matrix());
    pcl::getTransformation (-0.13, 0, -0.21, 0., -90.*D2R, 0., affine);
    vCamToBase.push_back(affine.matrix());
    pcl::getTransformation (0.16, 0, -0.18, 0., 90.*D2R, 0., affine);
    vCamToBase.push_back(affine.matrix());
}

// data: idx = [t_odo], value=[x, y, z, qx, qy, qz, qw]
bool CExtractPCDs::readPoseAndPngs(const char* path, std::map<double, vector<double> > & poses, map<double, vector<double> >& pngs)
{
    ifstream inf(path);
    if(!inf.is_open() )
    {
        cout<<"failed to open file: "<<path<<endl;
        return false;
    }   

    double t_odo_cur, t_odo;
    vector<double> v_pose_tmp;
    v_pose_tmp.resize(7);

    vector<double> v_png_tmp;
    v_png_tmp.resize(6);

    char line[255];
    string delim(" \t");
    while(inf.good())
    {   
        inf.getline(line, 255);
        if(inf.eof())
            break;
        //  
        t_odo_cur = atof(strtok(line,delim.c_str())); // false odo time in current system time

        v_pose_tmp[0] = atof(strtok(NULL,delim.c_str()));
        v_pose_tmp[1] = atof(strtok(NULL,delim.c_str()));
        v_pose_tmp[2] = atof(strtok(NULL,delim.c_str()));
        v_pose_tmp[3] = atof(strtok(NULL,delim.c_str()));
        v_pose_tmp[4] = atof(strtok(NULL,delim.c_str()));
        v_pose_tmp[5] = atof(strtok(NULL,delim.c_str()));
        v_pose_tmp[6] = atof(strtok(NULL,delim.c_str()));

        //real odo timestamp
        // t_odo_cur = atof(strtok(NULL, delim.c_str())); // again: false odo time in current time
        // t_odo = atof(strtok(NULL, delim.c_str())); // real odo time in current time
        for(int i=0;i<v_png_tmp.size();i++)
            v_png_tmp[i] = atof(strtok(NULL, delim.c_str())); 
        
        // data[t_odo] = v_pose_tmp;
        poses[t_odo_cur] = v_pose_tmp;
        pngs[t_odo_cur] = v_png_tmp;
    }   
    inf.close();
    printf("Loaded %d pose data \n", poses.size());
    return true;
}

void CExtractPCDs::reconstructPcd(const char* path, const int port, const double timestamp,
        const Eigen::Matrix4f& trans, color_cloud_t::Ptr& pcd)
{
    char rgbName[255];
    char depthName[255];
    sprintf(rgbName, "%s/raw_data/xtion_%d/rgb/%f.png", path, port, timestamp);
    sprintf(depthName, "%s/raw_data/xtion_%d/depth/%f.png", path, port, timestamp);

    cv::Mat rgb_img = cv::imread(rgbName, -1);
    cv::Mat depth_img = cv::imread(depthName, -1);

    if(!rgb_img.data || !depth_img.data )                              // Check for invalid input
    {
        printf("No Image Found, but Continue! \n");
        return;
    }

    //convert depth_img from 16uc1 to 32fc1
    cv::Mat float_img;
    depth_img.convertTo(float_img, CV_32FC1, 0.001, 0);//From mm to m(scale of depth_img matters)
    depth_img = float_img;

    //reconstruct
    // pcd = cloud_t::Ptr( createXYZPointCloud(depth_img,rgb_img));
    pcd = color_cloud_t::Ptr( createXYZRGBPointCloud(depth_img, rgb_img));
    pcl::transformPointCloud ( *pcd, *pcd, trans);
}

bool CExtractPCDs::operator()(string path_in, string path_out)
{
    // 1 read poses and pngs 
    map<double , vector< double> > poses;
    map<double , vector< double> > pngs; 
    string idx_path = path_in + "/pose_idx.txt";
    if(!readPoseAndPngs(idx_path.c_str(), poses, pngs))
    {
        cout<<"extract_pcds.cpp: failed to parse file: "<<idx_path<<endl;
        return false;
    }
    assert(poses.size() == pngs.size());
    // 2 save pngs to pcds in path_out
    map<double, vector<double> >::iterator it = pngs.begin();
    map<double, vector<double> >::iterator it_pose = poses.begin();

    // for debug 
    Eigen::Matrix4f iden_mat = Eigen::Matrix4f::Identity();
    while(it != pngs.end())
    {
        color_cloud_t::Ptr curr_pcd(new color_cloud_t());
        for(int i=s_port_ ; i<= e_port_; i++)
        {
            int index = i - 9009; // 9009 is the start index
            if(it->second[index] >=0)
            {
                color_cloud_t::Ptr tmp_pcd(new color_cloud_t());    
                reconstructPcd(path_in.c_str(), i, it->second[index], iden_mat, tmp_pcd);
                *curr_pcd += *tmp_pcd;
            }
        }
        if(curr_pcd->points.size() > 0)
        {
            // add pose to this pcd 
            curr_pcd->sensor_origin_.x() = it_pose->second[0]; // x
            curr_pcd->sensor_origin_.y() = it_pose->second[1]; // y
            curr_pcd->sensor_origin_.z() = it_pose->second[2]; // z
            curr_pcd->sensor_orientation_.x() = it_pose->second[3]; // qx
            curr_pcd->sensor_orientation_.y() = it_pose->second[4]; // qy
            curr_pcd->sensor_orientation_.z() = it_pose->second[5]; // qz
            curr_pcd->sensor_orientation_.w() = it_pose->second[6]; // qw
        
            // save to disk
            char outname[255];
            sprintf(outname, "%s/%.6lf.pcd", path_out.c_str(), it->first);
            
            color_cloud_t::Ptr tmpPCD(new color_cloud_t);
            convert2Std(curr_pcd, tmpPCD);
            pcl::io::savePCDFile(outname, *tmpPCD, true);
        }
        ++it;
        ++it_pose;
    }
}

