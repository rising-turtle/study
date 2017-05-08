/*
    Author: David Z, 2014.2.14 

    To extract pcds from 6 xtion! 
    input: sysed pose-related (img, dpt) 
    output: pcds + poses, with name = timestamp
*/

#ifndef EXTRACT_PCDS_H
#define EXTRACT_PCDS_H

#include <string>
#include <map>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include "pcl_dec.h"
#include <boost/make_shared.hpp>

// typedef pcl::PointXYZ point_type;
// typedef pcl::PointCloud<point_type> pointcloud_type;

using namespace std;

class CExtractPCDs
{
public:
    CExtractPCDs(int s_port=9012, int e_port=9014);
    ~CExtractPCDs();
    bool operator()(string path_in, string path_out);
protected:
    bool readPoseAndPngs(const char* path, std::map<double, vector<double> > & poses, map<double, vector<double> >&);
    void reconstructPcd(const char* path, const int port, const double timestamp, const Eigen::Matrix4f& trans, color_cloud_t::Ptr& pcd);
    
    template<typename PointT>
    bool convert2Std(boost::shared_ptr<pcl::PointCloud<PointT> >& , boost::shared_ptr<pcl::PointCloud<PointT> >& );

    // up-front, up-left, up-right, down-front, down-left, down-right
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > vCamToBase; 
   
    string in_path_; // input path 
    string ou_path_; // output path
    int s_port_;
    int e_port_;
private:
    void initTrans();
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename PointT>
bool CExtractPCDs::convert2Std(boost::shared_ptr<pcl::PointCloud<PointT> >& in, boost::shared_ptr<pcl::PointCloud<PointT> >& out)
{
    static const unsigned int HEIGHT = 480;
    static const unsigned int WIDTH = 640;
    static const unsigned int DenseN = HEIGHT*WIDTH;
    if(in->width * in->height != DenseN)
    {
        cout<<"extract_pcds.cpp: input pcd cannot be dense! with points: "<<in->width*in->height<<endl;
        return false;
    }
    // out = boost::make_shared<pcl::PointCloud<PointT> >();
    out->is_dense = false; // dense means delete the useless points 
    out->width = WIDTH;
    out->height = HEIGHT;
    out->points.reserve(WIDTH * HEIGHT);
    out->sensor_origin_ = in->sensor_origin_;
    out->sensor_orientation_ = in->sensor_orientation_;
    out->points.insert(out->points.begin(), in->points.begin(), in->points.end());
    return true;
}

#endif

