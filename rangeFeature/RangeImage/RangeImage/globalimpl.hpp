#include "boost/thread/thread.hpp"

template<typename PT>
void SparsePC(boost::shared_ptr< pcl::PointCloud<PT> >& m_densePC,float leafsize){
	boost::shared_ptr< pcl::PointCloud<PT> > m_tmpPC(new pcl::PointCloud<PT>);
	pcl::VoxelGrid<PT> sor;
	sor.setInputCloud (m_densePC);
	sor.setLeafSize (leafsize, leafsize, leafsize);
	sor.filter (*m_tmpPC);
	m_densePC.swap(m_tmpPC);
}

template<typename PT>
void DisplayPoint(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
				  boost::shared_ptr< pcl::PointCloud<PT> >& m_disPC,int _pointProperty)
{
	viewer->setBackgroundColor (0, 0, 0);
	//pcl::visualization::PointCloudColorHandlerRGBField<PT> rgb(m_disPC);
	//viewer->addPointCloud<PT> (m_disPC, rgb, "sample cloud");
	viewer->addPointCloud<PT> (m_disPC, "sample cloud");
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
extern void DisplayPoint(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
						 boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& m_disPC,int _pointProperty);

template<typename PT>
void DisplayPT(boost::shared_ptr<PT>& m_pt)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	DisplayPoint(viewer,m_pt,2);
}

extern void DisplayPTAndNormal(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& pt,\
								boost::shared_ptr<pcl::PointCloud<pcl::Normal> >& normal);

template<typename PT>
void AddPT2Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
				  boost::shared_ptr< pcl::PointCloud<PT> >& m_disPC, string name, int point_size=2)
{
	viewer->addPointCloud<PT> (m_disPC, name.c_str());
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name.c_str());
}
template<>
extern void AddPT2Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
						 boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >& m_disPC, string name, int point_size);

extern void AddPTNV2Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
						   boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >& m_disPC, \
					boost::shared_ptr<pcl::PointCloud<pcl::Normal> >& m_normal,string pcname, string nvname);

extern void initViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);
extern void showViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);

extern void AddPolygonMesh2Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
								  pcl::PolygonMesh &poly_mesh,std::string id);