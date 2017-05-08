#ifndef BAG_LISTENER_H
#define BAG_LISTENER_H

#include "globaldef.h"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
// #include <tf/tfMessage.h>
// #include <tf/tf.h>
// #include <tf/transform_broadcaster.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <bitset>
#include <vector>
#include <QObject>

using namespace std;

class CBagListener : public QObject
{
    Q_OBJECT
public:
    enum SYN {COLOR=0, DEPTH, CAMERA, TF};
    const static int n_signals = 4;
    CBagListener(int stack = 10)
    {
        m_bSyn.reset();
        m_bSyn.set(TF);
        // ros::NodeHandle nh_;
        // ros::Subscriber sub1 = nh_.subscribe<sensor_msgs::Image>("/camera/depth/image",10, &CBagListener::depth_cb, this);
        // ros::Subscriber sub2 = nh_.subscribe<sensor_msgs::Image>("/camera/rgb/image_color",10, &CBagListener::visual_cb,this);
        depth_sub = nh_.subscribe<sensor_msgs::Image>(gl_bag_depth_name, stack, &CBagListener::depth_cb, this);
        visual_sub = nh_.subscribe<sensor_msgs::Image>(gl_bag_color_name, stack, &CBagListener::visual_cb,this);
        // ros::Subscriber sub3 = nh_.subscribe<tf::tfMessage>(gl_bag_tf_msg, stack, &CBagListener::tf_cb, this);
        camera_sub = nh_.subscribe<sensor_msgs::CameraInfo>(gl_bag_camera_info, stack, &CBagListener::camera_cb, this);
        ROS_INFO("After setting callbacks!");
    }
    typedef union
    {
        struct
        {
            unsigned char Blue;
            unsigned char Green;
            unsigned char Red;
            unsigned char Alpha;
        };
        float float_value;
        uint32_t long_value;
    } RGBValue;

    void depth_cb(const sensor_msgs::ImageConstPtr& depth_img)
    {
        static unsigned int depth_cout = 0;
        m_vDpts.push_back(depth_img);
        ROS_INFO_STREAM("Receive "<<++depth_cout<<" Depth Images!");
        setBit(DEPTH);
    }
    void visual_cb(const sensor_msgs::ImageConstPtr& visual_img)
    {
        static unsigned int visual_cout = 0;
        m_vImgs.push_back(visual_img);
        ROS_INFO_STREAM("Receive "<<++visual_cout<<" Visual Images!");
        setBit(COLOR);
    }
   /* void tf_cb(const tf::tfMessage& tf_msg)
    {
        static unsigned int tf_cout = 0;
        ROS_INFO_STREAM("Receive "<<++depth_cout<<" tf Message!");
        setBit(TF);
    }*/

    void camera_cb(const sensor_msgs::CameraInfoConstPtr& camera_info)
    {
        static unsigned int depth_cout = 0;
        m_vCams.push_back(camera_info);
        ROS_INFO_STREAM("Receive "<<++depth_cout<<" Depth Images!");
        setBit(CAMERA);
    }
Q_SIGNALS:
    void sendCameraImgs(sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfoConstPtr);
    void sendColorPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);
private:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr createXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg, 
        const sensor_msgs::ImageConstPtr& rgb_msg,
        const sensor_msgs::CameraInfoConstPtr& cam_info);
    inline void setBit(SYN sig)
    {
        m_bSyn[sig] = true;
        static unsigned int syn_num = 0;
        if(m_bSyn.count() == m_bSyn.size())
        {
            // send out this pair data
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc = createXYZRGBPointCloud(m_vDpts[syn_num], m_vImgs[syn_num], m_vCams[syn_num]);
            sendColorPointCloud(pc);
            sendCameraImgs(m_vImgs[syn_num], m_vDpts[syn_num], m_vCams[syn_num]);
            // delete count msgs
            m_vImgs[syn_num].reset();
            m_vDpts[syn_num].reset();
            m_vCams[syn_num].reset();
            ++syn_num;
            ROS_INFO_STREAM("SYN: "<<syn_num<<" all signals!");
            m_bSyn.reset();
            m_bSyn.set(TF);
        }
    }
    std::bitset<n_signals> m_bSyn;
    vector<sensor_msgs::ImageConstPtr> m_vImgs;
    vector<sensor_msgs::ImageConstPtr> m_vDpts;
    vector<sensor_msgs::CameraInfoConstPtr> m_vCams; 
    ros::NodeHandle nh_;
    ros::Subscriber visual_sub;
    ros::Subscriber depth_sub;
    ros::Subscriber camera_sub;
};







#endif
