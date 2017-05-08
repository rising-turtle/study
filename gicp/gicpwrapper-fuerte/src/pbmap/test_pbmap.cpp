#include <iostream>
#include <mrpt/pbmap.h> // precomp. hdr
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "globaldef.h"
#include "FileReader.h"
#include <vector>

using namespace std;
using namespace mrpt::pbmap;

// string path("/home/edu/Libraries/mrpt-svn/share/mrpt/datasets/pbmap-demos/");

typedef pcl::PointXYZRGBA PointT;

string path(gl_pcd_file_dir);

void testPbMapConstruction(const string &config_file)
{
  // Reconstructed PointCloud
  pcl::PointCloud<PointT> globalCloud;

  PbMapMaker pbmap_maker(config_file);

  unsigned N = 3; // Read the 5 sample pairs pointClouds+Pose
  string cloudFile, poseFile;

  vector<point_cloud_ptr> m_pcs;
  vector<octomap::pose6d> m_poses;
  readPcdAndPose(path.c_str(), m_pcs, m_poses, N);
  for(int i=0;i<N;i++)
  {
     frameRGBDandPose cloudAndPose;
     // cloudAndPose.cloudPtr.reset(new pcl::PointCloud<PointT>);
     cloudAndPose.cloudPtr = m_pcs[i];
     fromPose6d2Eigen(cloudAndPose.pose, m_poses[i]);
     
     // Detect planes and build PbMap
     pbmap_maker.frameQueue.push_back(cloudAndPose);

     pcl::PointCloud<PointT>::Ptr alignedCloudPtr(new pcl::PointCloud<PointT>);
     pcl::transformPointCloud(*(cloudAndPose.cloudPtr),*alignedCloudPtr,cloudAndPose.pose);
     globalCloud += *alignedCloudPtr;
  }

  // Serialize PbMap
  mrpt::utils::CFileGZOutputStream serialize_pbmap("test.pbmap");
  serialize_pbmap << pbmap_maker.getPbMap();
  serialize_pbmap.close();

  // Save reconstructed point cloud
  pcl::io::savePCDFile("reconstructed_cloud.pcd", globalCloud);

  double total_area = 0.0;
  for(unsigned i=0; i < pbmap_maker.getPbMap().vPlanes.size(); i++)
    total_area += pbmap_maker.getPbMap().vPlanes[i].areaHull;
  cout << "This PbMap contains " << pbmap_maker.getPbMap().vPlanes.size() << " planes, covering a total area of " << total_area << " m2" << endl;

}

int main(int argc, char **argv)
{
  try
  {
		bool showHelp    = argc>1 && !os::_strcmp(argv[1],"--help");

		// Process arguments:
		if (argc<2 || showHelp )
		{
			printf("Usage: %s <config_file.ini>\n\n",argv[0]);
			if (!showHelp)
			{
				// mrpt::system::pause();
                
				return -1;
			}
			else return 0;
		}

    const string INI_FILENAME = string( argv[1] );

    testPbMapConstruction(INI_FILENAME);

    return 0;

  } catch (exception &e)
  {
    cout << "MRPT exception caught: " << e.what() << endl;
    return -1;
  }
  catch (...)
  {
    printf("Another exception!!");
    return -1;
  }
}
