

#ifndef PCL_DEF_H
#define PCL_DEF_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

typedef pcl::PointXYZ point_t;
typedef pcl::PointCloud<point_t> cloud_t;

typedef pcl::PointXYZRGBA color_point_t;
typedef pcl::PointCloud<color_point_t> color_cloud_t;

typedef union
{
    struct /*anonymous*/
    {   
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;
    };  
    float float_value;
    long long_value;
} RGBValue;


cloud_t* createXYZPointCloud( const cv::Mat& depth_img, const cv::Mat& rgb_img);
// depth_img must be 32fc1
color_cloud_t* createXYZRGBPointCloud (const cv::Mat& depth_img, const cv::Mat& rgb_img);


#endif
