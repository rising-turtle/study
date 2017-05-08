#include "preheader.h"
#include "globaldefinition.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>


int showTriangulation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.2);

	// Set typical values for the parameters
	gp3.setMu (1.5);
	gp3.setMaximumNearestNeighbors (100);
	//gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	initViewer(viewer);
	AddPolygonMesh2Viewer(viewer,triangles,"triangles");
	showViewer(viewer);

	// Finish
	return (0);
}

void deleteInfinite( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_pc)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (size_t cp = 0; cp < m_pc->points.size (); ++cp)
	{

		if (!pcl_isfinite (m_pc->points[cp].x) ||
			!pcl_isfinite (m_pc->points[cp].y) ||
			!pcl_isfinite (m_pc->points[cp].z)){
				continue;
		}
		tmp_pc->points.push_back(m_pc->points[cp]);
	}
	m_pc->points.swap(tmp_pc->points);
}

int fast_greedy_triangulation (string file_name)
{
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	//sensor_msgs::PointCloud2 cloud_blob;
	pcl::io::loadPCDFile (file_name.c_str(), *cloud/*cloud_blob*/);
	//pcl::fromROSMsg (cloud_blob, *cloud);
	//* the data should be available in cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sparsecloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.02f, 0.02f, 0.02f);
	sor.filter (*sparsecloud);

	deleteInfinite(sparsecloud);

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
	tree->setInputCloud (sparsecloud);
	n.setInputCloud (sparsecloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures
	return showTriangulation(sparsecloud,normals);
}