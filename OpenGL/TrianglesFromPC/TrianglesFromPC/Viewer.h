#ifndef VIEWER_H
#define VIEWER_H
#pragma once

#define NOMINMAX

#include "preheader.h"
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#define RX 1200 // X [-6 6]
#define RY 400	// Y [-2 2]
#define RZ 1200 // Z [-6 6]

#define L_RX RX/200
#define L_RY RY/200
#define L_RZ RZ/200

#define S_RX RX/2
#define S_RY RY/2
#define S_RZ RZ/2

#define CELLSIZE 4
#define X_CELL RX/CELLSIZE // 300
#define Y_CELL RY/CELLSIZE // 100
#define Z_CELL RZ/CELLSIZE // 300

#define X_STEP Y_CELL*Z_CELL // 120*400
#define Y_STEP Z_CELL // 400

#define ALL_CELLS X_CELL*Y_CELL*Z_CELL // 


#define IOTGUI_SLAM_SIZE 19200000  //400*400*120
#define IOTGUI_SLAM_TRIBLESIZE 57600000  //400*400*120
#define IOTGUI_SLAM_SIXTHSIZE 230400000  //400*400*120

class MapBuilder{
		
public:
	// to ues this type in .cpp
	typedef pcl::PointXYZRGB point_type;
	typedef pcl::PointCloud<point_type> pointcloud_type;

	MapBuilder():global_map(new pointcloud_type),num_of_ver(0){
		MapbuilderInit();
	}
	~MapBuilder(){}
	void loadpcfromfile(std::string& filename){
		pcl::io::loadPCDFile (filename, *global_map);
	}
	// Init Mapbuilder params
	void MapbuilderInit();
	void MapbuilderUnInit();

	// Push data to OpenGL
	void Push2OpenGL(unsigned char* pimg, float* pver);

	// Index from (x,y,z)
	inline int getIndexCell(float& x,float& y, float& z){
		if(_isnan(x) || _isnan(y) || _isnan(z))
			return -1;
		if(fabs(x) >= L_RX || fabs(z) >= L_RZ || fabs(y)>= L_RY )
			return -1;
		int lx = ( x*100 + S_RX); //> >2;/// CELLSIZE;
			lx>>=2;
		int ly = ( y*100 + S_RY); //> >2;/// CELLSIZE;
			ly>>=2;
		int lz = ( z*100 + S_RZ); //> >2;/// CELLSIZE;
			lz>>=2;
		return (lx*X_STEP + ly*Y_STEP + lz);
	}

	// Copy from CPointCloud to Cells
	void fromPCtoCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud);
	// Show globalCells
	void ShowGlobalCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud);
	// From Cell to RenderList
	int CreateRenderList();
	void drawVertex(boost::shared_ptr<pcl::PointXYZRGB>& p);

	// From Cell to RenderArrayElement
	void drawVertex(boost::shared_ptr<pcl::PointXYZRGB> p[3],unsigned char *pucImg,float *pfVertex);
	// From Cells to Triangles pucImg->Color Array, pfVertex->Vertex Array
	void FromCell2Triangles(unsigned char *pucImg,float *pfVertex);

	// this is interface for rendering 
	//void RenderPoints();
	void RenderTriangle();
	void RenderTriangle2();

	// number of points to be rendered
	size_t num_of_ver;
	GLubyte *m_pucSLAMColor;
	GLfloat *m_pfSLAMPC;
	
public:
	
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map; // global map
	 std::vector<int> weighted; // use MRPT weighted point Alignment
	 float min_points_distance; // determine whether these two points are the same point

	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cell_map; // to display cells
	 std::vector<boost::shared_ptr<pcl::PointXYZRGB> > global_cells; // all the discrete cells 5*5*5
	
	 std::vector<GLint> cloud_list_indices; // show list
	 float * pVertex; // pointer to vertex-array
	 unsigned char * pImg; // pointer to color-array
};

#endif