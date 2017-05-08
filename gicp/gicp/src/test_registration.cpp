#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>

int main(int argc , char** argv)
{
  using namespace std;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, cloud_target, cloud_aligned;
  cloud_source.reset (new pcl::PointCloud<pcl::PointXYZ> ());
  cloud_target.reset (new pcl::PointCloud<pcl::PointXYZ> ());
  cloud_aligned.reset (new pcl::PointCloud<pcl::PointXYZ> ());
  if (pcl::io::loadPCDFile (argv[1], *cloud_source) < 0)
  {
    std::cout << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  if (pcl::io::loadPCDFile (argv[2], *cloud_target) < 0)
  {
    std::cout << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  // Eigen::Matrix3f R;
  // R = Eigen::AngleAxisf(M_PI/4, Eigen::Vector3f::UnitZ());
  // Eigen::Matrix4f RT;
  // RT.topLeftCorner<3, 3> () = R;
  // RT(0,3) = 0; RT(1,3) = 0; RT(2,3) = 0; RT(3,3) = 1;
//  pcl::GeneralizedIterativeClosestPointDouble<pcl::PointXYZ, pcl::PointXYZ> icp;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud (cloud_source);
  icp.setInputTarget (cloud_target);
  icp.align (*cloud_aligned);
  cout << "final transform from gicp" << endl;
  cout << icp.getFinalTransformation() << endl;
  // cout << "transformation difference from gicp" << endl;
  // cout << icp.getFinalTransformation() - RT << endl;
}
