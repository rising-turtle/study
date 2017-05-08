#include "bag_listener.h"

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CBagListener::createXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg, 
        const sensor_msgs::ImageConstPtr& rgb_msg,
        const sensor_msgs::CameraInfoConstPtr& cam_info) 
{
    // struct timespec starttime, finish; 
    // double elapsed; 
    // clock_gettime(CLOCK_MONOTONIC, &starttime);
    // pointcloud_type* cloud (new pointcloud_type() );
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloud->header.stamp     = depth_msg->header.stamp;
    cloud->header.frame_id  = rgb_msg->header.frame_id;
    cloud->is_dense         = false;

    float cx, cy, fx,fy;//principal point and focal lengths
    unsigned color_step, color_skip;

    cloud->height = depth_msg->height;
    cloud->width = depth_msg->width;
    cx = cam_info->K[2]; //(cloud->width >> 1) - 0.5f;
    cy = cam_info->K[5]; //(cloud->height >> 1) - 0.5f;
    fx = 1.0f / cam_info->K[0]; 
    fy = 1.0f / cam_info->K[4]; 
    int pixel_data_size = 0;

    if(rgb_msg->encoding.compare("mono8") == 0) pixel_data_size = 1;
    if(rgb_msg->encoding.compare("rgb8") == 0) pixel_data_size = 3;

    ROS_ERROR_COND(pixel_data_size == 0, "Unknown image encoding: %s!", rgb_msg->encoding.c_str());
    color_step = pixel_data_size * rgb_msg->width / cloud->width;
    color_skip = pixel_data_size * (rgb_msg->height / cloud->height - 1) * rgb_msg->width;

    cloud->points.resize (cloud->height * cloud->width);

    const float* depth_buffer = reinterpret_cast<const float*>(&depth_msg->data[0]);
    const uint8_t* rgb_buffer = &rgb_msg->data[0];

    // depth_msg already has the desired dimensions, but rgb_msg may be higher res.
    int color_idx = 0, depth_idx = 0;
    // double depth_scaling = ParameterServer::instance()->get<double>("depth_scaling_factor");
    double depth_scaling = 1.0; // default in rgbdslam

    pcl::PointCloud<pcl::PointXYZRGBA>::iterator pt_iter = cloud->begin ();
    for (int v = 0; v < (int)cloud->height; ++v, color_idx += color_skip)
    {
        for (int u = 0; u < (int)cloud->width; ++u, color_idx += color_step, ++depth_idx, ++pt_iter)
        {
            pcl::PointXYZRGBA& pt = *pt_iter;
            float Z = depth_buffer[depth_idx] * depth_scaling;

            // Check for invalid measurements
            if (std::isnan (Z))
            {
                pt.x = pt.y = pt.z = Z;
            }
            else // Fill in XYZ
            {
                pt.x = (u - cx) * Z * fx;
                pt.y = (v - cy) * Z * fy;
                pt.z = Z;
            }

            // Fill in color
            CBagListener::RGBValue color;
            if(pixel_data_size == 3){
                color.Red   = rgb_buffer[color_idx];
                color.Green = rgb_buffer[color_idx + 1];
                color.Blue  = rgb_buffer[color_idx + 2];
            } else {
                color.Red   = color.Green = color.Blue  = rgb_buffer[color_idx];
            }
            color.Alpha = 0;
            pt.rgb = color.float_value;
        }
    }

    // clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    return cloud;
}

