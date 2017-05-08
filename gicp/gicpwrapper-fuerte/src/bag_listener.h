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

class CBagListener
{
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
    void depth_cb(const sensor_msgs::ImageConstPtr& depth_img)
    {
        static unsigned int depth_cout = 0;
        ROS_INFO_STREAM("Receive "<<++depth_cout<<" Depth Images!");
        setBit(DEPTH);
    }
    void visual_cb(const sensor_msgs::ImageConstPtr& visual_img)
    {
        static unsigned int visual_cout = 0;
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
        ROS_INFO_STREAM("Receive "<<++depth_cout<<" Depth Images!");
        setBit(CAMERA);
    }
private:
    ros::NodeHandle nh_;
    void setBit(SYN sig)
    {
        m_bSyn[sig] = true;
        static unsigned int syn_num = 0;
        if(m_bSyn.any())
        {
            ROS_INFO_STREAM("SYN: "<<++syn_num<<" all signals!");
            m_bSyn.reset();
            m_bSyn.set(TF);
        }
    }
    std::bitset<n_signals> m_bSyn;
    ros::Subscriber visual_sub;
    ros::Subscriber depth_sub;
    ros::Subscriber camera_sub;
};


#endif
