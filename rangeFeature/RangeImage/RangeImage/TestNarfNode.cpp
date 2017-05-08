#include "preheader.h"
#include "globaldefinition.h"
#include "NarfNode.h"


void testNarfNode()
{
	CNarfNode node_tar("D:\\MyProjects\\pcl\\RangeImage\\Debug\\1.pcd");
	CNarfNode node_src("D:\\MyProjects\\pcl\\RangeImage\\Debug\\1.pcd");
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_DisPC(new pcl::PointCloud<pcl::PointXYZ>);
	DisplayPT(m_DisPC);

}

void testDisNarfNode()
{
	CNarfNode node_tar("D:\\MyProjects\\pcl\\RangeImage\\Debug\\1.pcd");
	//CNarfNode node_src("D:\\MyProjects\\pcl\\RangeImage\\Debug\\2.pcd");
	
	// find matched features
	//vector<cv::DMatch> matchedf;
	//node_src.FindPairsFlann(&node_tar,&matchedf);

	//// --------------------------------------------
	//// -----Open 3D viewer and add point cloud-----
	//// --------------------------------------------
	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.setBackgroundColor (1, 1, 1);
	viewer.addCoordinateSystem (1.0f);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> point_cloud_color_handler (node_tar.m_pRangeImage, 0, 0, 0);
	viewer.addPointCloud (node_tar.m_pRangeImage, point_cloud_color_handler, "target point cloud");

	// -------------------------------------
	// -----Show keypoints in 3D viewer-----
	// -------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
	keypoints.points.resize (node_tar.m_keypoint_indices.size ());

	pcl::RangeImage& range_image = *node_tar.m_pRangeImage; 

	for (size_t i=0; i<node_tar.m_keypoint_indices.size (); ++i)
		keypoints.points[i].getVector3fMap () = range_image.points[node_tar.m_keypoint_indices[i]].getVector3fMap ();

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "narfkeypoints");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "narfkeypoints");

	// see whether pfh features are at the same points
	keypoints.points.resize(node_tar.m_pfhKeyPoints->points.size());
	for(size_t i=0;i<node_tar.m_pPFHDest->points.size();i++)
	{
		keypoints.points[i].getVector3fMap() =node_tar.m_pfhKeyPoints->points[i].getVector3fMap ();
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler2 (keypoints_ptr, 255, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler2, "pfhkeypoints");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "pfhkeypoints");

	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer.wasStopped ())
	{
		//	range_image_widget.spinOnce ();  // process GUI events
		viewer.spinOnce ();
		pcl_sleep(0.01);
	}
}

void testNarfMatchFeatures()
{
	CNarfNode node_tar("D:\\MyProjects\\pcl\\RangeImage\\Debug\\1.pcd");
	CNarfNode node_src("D:\\MyProjects\\pcl\\RangeImage\\Debug\\2.pcd");
	
	// find matched features
	vector<cv::DMatch> matchedf;
	//node_src.FindPairsFlann(&node_tar,&matchedf,false);
	//node_src.FindMultiPairsFlann(&node_tar,&matchedf,2);
	node_src.FindPairsFlann2(&node_tar,&matchedf,5);
	node_src.DeleteMultiMatches(&matchedf);

	// RANSAC method to find actual matched pairs
	vector<cv::DMatch> inliers;
	float rmse;
	Eigen::Matrix4f resulting_transformation;
	bool bInliers = node_src.getRelativeTransformationTo(&node_tar,&matchedf, resulting_transformation, rmse, inliers);
	if(bInliers==false)
	{
		cout<<"failed to find matches using RANSAC!"<<endl;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1=node_tar.m_pPC;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2=node_src.m_pPC;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_left(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_right(new pcl::PointCloud<pcl::PointXYZRGB>);

	//pcl::PointCloud<pcl::Narf36>::Ptr keypoints1=node_tar.m_pNarfDest;
	//pcl::PointCloud<pcl::Narf36>::Ptr keypoints2=node_src.m_pNarfDest;

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints1_back=node_tar.m_pfhKeyPoints;//(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints2_back=node_src.m_pfhKeyPoints;//(new pcl::PointCloud<pcl::PointXYZ>);

	/*pcl::PointXYZ sp;
	for(size_t i=0;i<keypoints1->points.size();i++)
	{
		sp.x=keypoints1->points[i].x;
		sp.y=keypoints1->points[i].y;
		sp.z=keypoints1->points[i].z;
		keypoints1_back->points.push_back(sp);
	}
	for(size_t i=0;i<keypoints2->points.size();i++)
	{
		sp.x=keypoints2->points[i].x;
		sp.y=keypoints2->points[i].y;
		sp.z=keypoints2->points[i].z;
		keypoints2_back->points.push_back(sp);
	}*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_left(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_right(new pcl::PointCloud<pcl::PointXYZ>);

	// Shift the first clouds' points to the left
	//const Eigen::Vector3f translate (0.0, 0.0, 0.3);
	const Eigen::Vector3f translate (2.0, 0.0, 0.0);
	const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
	pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
	pcl::transformPointCloud (*keypoints1_back, *keypoints_left, -translate, no_rotation);

	// Shift the second clouds' points to the right
	pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
	pcl::transformPointCloud (*keypoints2_back, *keypoints_right, translate, no_rotation);

	// Add the clouds to the vizualizer
	pcl::visualization::PCLVisualizer viz;
	viz.setBackgroundColor (1, 1, 1);
	viz.addCoordinateSystem (1.0f);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> point_cloud_color_handler_left (points_left, 0, 0, 0);
	viz.addPointCloud (points_left, point_cloud_color_handler_left, "target point cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> point_cloud_color_handler_right (points_right, 0, 0, 0);
	viz.addPointCloud (points_right, point_cloud_color_handler_right, "source point cloud");
	//viz.addPointCloud (points_left, "points_left");
	//viz.addPointCloud (points_right, "points_right");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler_left (keypoints_left, 0, 255, 0);
	viz.addPointCloud<pcl::PointXYZ> (keypoints_left, keypoints_color_handler_left, "keypoints_left");
	viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints_left");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler_right (keypoints_right, 255, 0, 0);
	viz.addPointCloud<pcl::PointXYZ> (keypoints_right, keypoints_color_handler_right, "keypoints_right");
	viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints_right");

	std::vector<float> correspondence_scores;
	correspondence_scores.resize(matchedf.size());
	for(int i=0;i<matchedf.size();i++)
	{
		correspondence_scores[i]=matchedf[i].distance;
	}

	// Compute the median correspondence score
	std::vector<float> temp (correspondence_scores);
	std::sort (temp.begin (), temp.end ());
	float median_score = temp[temp.size ()/2];

	// Draw lines between the best corresponding points
	for (size_t i = 0; i < inliers.size()/*matchedf.size()*//*keypoints_left->points.size()*/; ++i)
	{
		//if (correspondence_scores[i] > median_score)
		//{
		//	continue; // Don't draw weak correspondences
		//}
		//if(i>10) break;

		// Get the pair of points
		/*
		const pcl::PointXYZ & p_left = keypoints_left->points[matchedf[i].trainIdx];
				const pcl::PointXYZ & p_right = keypoints_right->points[matchedf[i].queryIdx];*/
		const pcl::PointXYZ & p_left = keypoints_left->points[inliers[i].trainIdx];
		const pcl::PointXYZ & p_right = keypoints_right->points[inliers[i].queryIdx];
		/*pcl::PointXYZ p1,p2;
		p1.x=p_left.x; p1.y=p_left.y; p1.z=p_left.z;
		p2.x=p_right.x; p2.y=p_right.y; p2.z=p_right.z;*/

		// Generate a random (bright) color
		double r = (rand() % 100);
		double g = (rand() % 100);
		double b = (rand() % 100);
		double max_channel = std::max (r, std::max (g, b));
		r /= max_channel;
		g /= max_channel;
		b /= max_channel;

		// Generate a unique string for each line
		std::stringstream ss ("line");
		ss << i;

		// Draw the line
		viz.addLine (p_left, p_right, r, g, b, ss.str ());
	}

	// Give control over to the visualizer
	viz.spin ();

}