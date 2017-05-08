#include "Submap.h"
#include <string>
#include <sstring>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "misc.h"
using namespace std;

int main()
{
    string path("./data1/");
    std_msgs::Header header;
    for(int i=1;i<=6;i++)
    {
        stringstream s1,s2;
        s1<<i<<"depth.jpg";
        s2<<i<<"image.jpg";
        cv::Mat img, dpt, dpt_8mono;
        cv::imread(s1.str().c_str(), dpt);
        cv::imread(s2.str().c_str(), img);
        depthToCV8UC1(dpt, dpt_8mono);
        cv_bridge::CvImage dpt_pImg(header, "mono8", dpt_8mono);
        cv_bridge::CvImage img_pImg(header, "rgb8", img);
        sensor_msgs::ImagePtr fdpt = dpt_pImg->toImageMsg();
        sensor_msgs::ImagePtr fimg = img_pImg->toImageMsg();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = 
        // color_pc_ptr pc = createXYZRGBPointCloud()
    }
    return 0;
}
