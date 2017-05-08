#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "pcl_dec.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using namespace std; 

bool read( const char* path, vector<double>& index, int num = -1, int step = 1);
bool record(const char* path, int num = -1, int step = 1);

template <typename PointT>
bool convert2Std(boost::shared_ptr<pcl::PointCloud<PointT> >& in, boost::shared_ptr<pcl::PointCloud<PointT> >& out)
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


int main(int argc, char* argv[])
{
    if(argc < 2 )
    {
        cout<<"xxx path [num] [step] "<<endl;
        return -1;
    }
    int n = atoi(argv[2]);
    int step = 1;
    if(argc > 3)
        step = atoi(argv[3]);
    if(record(argv[1], n, step))
    {
        cout<<"succeed to record pcds!"<<endl;
    }else{
        cout<<"failed to record pcds!"<<endl;
    }
    return 0; 
}

bool record(const char* path, int num, int step)
{
     vector<double> index; 
     if(!read(path, index, num, step)) 
         return false;
     cout<<"retrive records: "<<index.size()<<endl;
     for(int i=0; i< index.size(); i++)
     {
         double v = index[i];
         char dptPath[255]; 
         char rgbPath[255];
         sprintf(dptPath, "%s/depth/%lf.png", path, v); 
         sprintf(rgbPath, "%s/rgb/%lf.png", path, v);
         
         cv::Mat dptM = cv::imread(dptPath, -1); 
         cv::Mat rgbM = cv::imread(rgbPath, -1);
              
         if(!dptM.data || !rgbM.data)
         {
            cout<<"No image found, with v: "<<v<<endl; 
            return false;
         }
         //convert depth_img from 16uc1 to 32fc1
         cv::Mat float_img;
         dptM.convertTo(float_img, CV_32FC1, 0.001, 0);//From mm to m(scale of depth_img matters)
         dptM = float_img;

         //reconstruct
         // pcd = cloud_t::Ptr( createXYZPointCloud(depth_img,rgb_img));
         color_cloud_t::Ptr pcd = color_cloud_t::Ptr( createXYZRGBPointCloud(dptM, rgbM));
         if(pcd->size() > 0)
         {
            char pcdPath[255]; 
            sprintf(pcdPath, "%s/pcds/%f.pcd", path, v);
            color_cloud_t::Ptr tmpPCD(new color_cloud_t);
            convert2Std(pcd, tmpPCD);
            if(tmpPCD->size()<=0)
            {
                cout<<"tmpPCD has no data!"<<endl;
                continue;
            }
            pcl::io::savePCDFile(pcdPath, *tmpPCD, true);
         }else{
            cout<<"pcd size = 0: "<<endl; 
        }
     }
    return true;
}

bool read(const char* path, vector<double>& index, int num, int step)
{
    string fname = string(path) + string("/rgbd.txt");
    ifstream inf(fname.c_str());
    if(!inf.is_open())
    {
        cout<<"failed to read file: "<<path<<endl;
        return false;
    }
    
    char line[4096];
    int cnt = 0 ;
    int r_cnt = 0;
    while(inf.getline(line, 4096))
    {
        if(cnt++ % step !=0) continue;
        double index_v;
        sscanf(line, "%lf ", &index_v); 
        index.push_back(index_v);
        if(num >0 && ++r_cnt >= num)
        {
            break; 
        }   
    }
    return true;
}

