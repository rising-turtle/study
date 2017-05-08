#include "preheader.h"
#include "globaldefinition.h"
#include "stdlib.h"

double rand_noise(){
	srand((unsigned int)(time(NULL)));
	double randf=(rand()%1000);
	return (randf*0.00005)-0.025;
}

template<>
void DisplayPoint(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
				  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& m_disPC,int _pointProperty)
{
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(m_disPC);
	viewer->addPointCloud<pcl::PointXYZRGB> (m_disPC, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _pointProperty, "sample cloud");
	viewer->addCoordinateSystem (2.0);
	viewer->initCameraParameters ();

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

template<>
void AddPT2Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
						 boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >& m_disPC,string name,int point_size)
{
	// 对于特殊点
	if(point_size!=2)
	{
		static int color_index=1;
		COLOR ran_c = colorindex(++color_index%g_color_number);
		for(int i=0;i<m_disPC->points.size();i++)
		{
			pcl::PointXYZRGB& sp=m_disPC->points[i];
			sp.r = ran_c.r; sp.g=ran_c.g; sp.b=ran_c.b;
		}
	}
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(m_disPC);
	viewer->addPointCloud<pcl::PointXYZRGB> (m_disPC, rgb, name.c_str());
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name.c_str());
}

void DisplayPTAndNormal(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& pt, boost::shared_ptr<pcl::PointCloud<pcl::Normal> >& normal)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pt);
	viewer->addPointCloud<pcl::PointXYZRGB> (pt, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (pt, normal, 10, 0.2, "normals");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.0,0.0,0.0,"normals");

	viewer->addCoordinateSystem (2.0);
	viewer->initCameraParameters ();
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void AddPTNV2Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
						   boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >& m_disPC, \
						   boost::shared_ptr<pcl::PointCloud<pcl::Normal> >& m_normal,string pcname,string nvname)
{
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(m_disPC);
	viewer->addPointCloud<pcl::PointXYZRGB> (m_disPC, rgb, pcname.c_str());
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, pcname.c_str());
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,golden().r,golden().g,golden().b,pcname.c_str());
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (m_disPC, m_normal, 1, 0.2, nvname.c_str());
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.0,0.0,0.0,nvname.c_str());
}

void initViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
{
	viewer->setBackgroundColor (0, 0, 0);
}
void showViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
{
	viewer->addCoordinateSystem (2.0);
	viewer->initCameraParameters ();
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void AddPolygonMesh2Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
						   pcl::PolygonMesh &poly_mesh,std::string id)
{
	//static double psize = 2.0, opacity = 1.0, linesize =1.5;
	viewer->addPolygonMesh(poly_mesh,id);
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, id.c_str());
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id.c_str());
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, id.c_str());
}