#include <iostream>
#include <fstream>
#include <string>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

int main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

 // if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/jin/Desktop/pointcloud/data/lab2.pcd", *point_cloud_ptr) == -1) //* load the file
 // if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/davidz/work/study/testPcl/data/lab2.pcd", *point_cloud_ptr) == -1) //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/davidz/work/study/testPcl/data/lab3.pcd", *point_cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }


  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (point_cloud_ptr);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // viewer->setBackgroundColor (0, 0, 0);
  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
  // viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "sample cloud");

  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb, "sample cloud");

  std::string idx;

  double buffer, x,y,z,qx,qy,qz,qw;
  double x_tj,y_tj,z_tj;
  double x_tj_old,y_tj_old,z_tj_old;

  x_tj  = 0;
  y_tj  = 0;          
  z_tj  = 0;

  pcl::PointXYZRGB now_point;
  pcl::PointXYZRGB last_point;

 // ifstream input( "/home/jin/Desktop/trajectory/6678_2_Yes/CameraTrajectory.txt" );
 // ifstream input( "/home/davidz/work/study/testPcl/data/6678_2_Yes/CameraTrajectory.txt" );
 ifstream input( "/home/davidz/work/study/testPcl/data/6678_3_Yes/CameraTrajectory.txt" );

  int count = 0;
  int res = 1;

  while(!input.eof())
  {
    count ++;
    res++;
    if (res<1){
      continue;
    }
    else
    {
    res = 0;
    cout << count <<endl;
    
    input>>buffer>>x>>y>>z>>qx>>qy>>qz>>qw;

    last_point = now_point;
    std::stringstream ss;
    ss<<"line_"<<count;

    x_tj  = x;
    y_tj  = y;          
    z_tj  = z;

    now_point.x = x_tj;
    now_point.y = y_tj;
    now_point.z = z_tj;

    cout << count <<endl; 
    viewer->addLine(last_point,now_point,0,255,0, ss.str());  
    }
  }

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
