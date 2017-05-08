#include "preheader.h"
#include "globaldefinition.h"
#include "edge_contour_extraction.h"

void testEdgeContourExtraction(string file_name)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr oriPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tarPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(pcl::io::loadPCDFile(file_name,*oriPC)==-1)
	{
		cout<<"failed to open file: "<<file_name<<endl;
		return;
	}
	
	pcl::EdgeContourExtraction<pcl::PointXYZRGB> edgeExtractor;
	edgeExtractor.setInputCloud(oriPC);
	edgeExtractor.setEigMeanK(20);
	edgeExtractor.setPrincEigThresh(0.76);
	edgeExtractor.filter(*tarPC);
	DisplayPT(tarPC);
}