#include "edge_contour_extraction.h"

void initialize(){
	pcl::EdgeContourExtraction<pcl::PointXYZRGB>::Ptr m_pc(new pcl::EdgeContourExtraction<pcl::PointXYZRGB>);
	return ;
}