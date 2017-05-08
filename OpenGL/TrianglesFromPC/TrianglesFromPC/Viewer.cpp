#include "Viewer.h"

using namespace std;

#define COLOR_RANGE 255.0

void MapBuilder::MapbuilderInit()
{
	global_cells.resize(ALL_CELLS);
	m_pucSLAMColor=new GLubyte[IOTGUI_SLAM_TRIBLESIZE];
	m_pfSLAMPC=new GLfloat[IOTGUI_SLAM_TRIBLESIZE];
}

void MapBuilder::Push2OpenGL(unsigned char* pimg, float* pver)
{
	memcpy(m_pucSLAMColor,pimg,num_of_ver*3);
	memcpy(m_pfSLAMPC,pver,num_of_ver*12);
}
//// Render for OpenGL
//void MapBuilder::RenderPoints(){
//	// Now we try to render to 
//	glBegin(GL_POINTS);
//	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator it = global_map->points.begin();
//	for(size_t i=0;i<global_map->points.size(); i++,it++){
//			if (_isnan((*it).x) || 
//				_isnan((*it).y) || 
//				_isnan((*it).z))
//				continue;
//			float fr = (float)(*it).r/COLOR_RANGE;
//			float fg = (float)(*it).g/COLOR_RANGE;
//			float fb = (float)(*it).b/COLOR_RANGE;
//		glColor3f(fr,fg,fb);
//		glVertex3f((*it).x,(*it).y,(*it).z);
//	}
//	glEnd();
//}
void MapBuilder::RenderTriangle(){
	
	for(size_t i=0;i<cloud_list_indices.size();i++)
		glCallList(cloud_list_indices[i]);
}
 // Copy from CPointCloud to Cells
void MapBuilder::fromPCtoCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud){
	
	int N = pCloud->points.size();
	vector<bool> index_point(N,false);
	vector<int> index_set;

	for(size_t i=0;i<pCloud->points.size();i++)
	{
		pcl::PointXYZRGB& sp = pCloud->points[i];
		int index = getIndexCell(sp.x,sp.y,sp.z);
		if(index >=0 )
		{
			if(global_cells[index].get()!=NULL) // this is already painted
				continue; // not flush this point using the new point	
			boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
			p->x = sp.x; p->y = sp.y; p->z = sp.z; 
			p->r = sp.r; p->g = sp.g; p->b = sp.b;
			
			index_point[i] = true;
			index_set.push_back(index);
			
			//global_cells[index] = p;
			//global_cell_map->points.push_back(sp);
			/*global_cells[index].r = sp.r;
			global_cells[index].g = sp.g;
			global_cells[index].b = sp.b;
			global_cells[index].x = sp.x;
			global_cells[index].y = sp.y;
			global_cells[index].z = sp.z;
			global_cells[index]._unused = 1;*/
		}
	}
	for(size_t i=0,j=0;i<N;i++)
		if(index_point[i])
		{
			pcl::PointXYZRGB& sp = pCloud->points[i];
			boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
			p->x = sp.x; p->y = sp.y; p->z = sp.z; 
			p->r = sp.r; p->g = sp.g; p->b = sp.b;
			int index_cell = index_set[j++];
			if(global_cells[index_cell].get()== NULL){
				global_cells[index_cell/*index_set[j++]*/] = p;
//				global_cell_map->points.push_back(sp);
			}
		}
}
// Show globalCells
void MapBuilder::ShowGlobalCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud){
	for(size_t i=0;i<global_cells.size();i++){
		boost::shared_ptr<pcl::PointXYZRGB> sp = global_cells[i];
		if(sp.get() == NULL)
			continue;
		pCloud->points.push_back(*sp);
	}
}

void MapBuilder::drawVertex(boost::shared_ptr<pcl::PointXYZRGB>& p){
	glColor3ub(p->r,p->g,p->b);
	glVertex3f(p->x,p->y,p->z);
}

// From Cell to RenderList
int MapBuilder::CreateRenderList()
{
	static double color_similar = 3;
	bool d_left,d_behind,d_down;
	boost::shared_ptr<pcl::PointXYZRGB> pself;
	boost::shared_ptr<pcl::PointXYZRGB> pP[3];
	int d_n = 0;

	// create ShowList
	GLuint cloud_list_index = glGenLists(1);
	if(!cloud_list_index) {
		cout<<"No display list could be created"<<endl;
		return -1;
	}
	glNewList(cloud_list_index, GL_COMPILE);
	cloud_list_indices.push_back(cloud_list_index);

	  for(int lx=0;lx<X_CELL;lx++) // traverse along x-axis
		  for(int ly=0;ly<Y_CELL;ly++) // traverse along y-axis
			  for(int lz=0;lz<Z_CELL;lz++) // traverse along z-axis
			  {
				  // initilization 
				  d_n = 0;

				  // find point (lx,ly,lz) 
				  int index_self = lx*X_STEP + ly*Y_STEP + lz; 
				  pself = global_cells[index_self];
				  pP[d_n++] = pself;//global_cells[index_self];
				  if(pself.get()==NULL){
					continue;
				  }
				  
				  // on xoy plane
				  glBegin(GL_TRIANGLE_STRIP);
				  drawVertex(pself);

				  bool draw_xoy = true;
				  int ll = lx-1;
				  int ld = ly-1;
				  int lb = lz-1;
				  if( ll> 0){ 
					d_left = true;
					int ll_index = ll*X_STEP + ly*Y_STEP + lz;
					pP[d_n] = global_cells[ll_index];
					if(pP[d_n].get() ==NULL)
					{
						draw_xoy = false;
						glEnd();
					}
					else if(fabs(pP[d_n]->rgb - pP[d_n-1]->rgb) < color_similar)
					{
						//d_n++;
						drawVertex(pP[d_n++]);
					}
					if(draw_xoy && ld > 0){
						int dd_index = lx*X_STEP + ld*Y_STEP + lz;
						pP[d_n] = global_cells[dd_index];
						if(pP[d_n].get()==NULL)
						{
							draw_xoy = false;
							glEnd();
						}
						else if(fabs(pP[d_n]->rgb - pP[d_n-1]->rgb) < color_similar)
						{
							//d_n++;
							drawVertex(pP[d_n++]);
						}

						if(draw_xoy){
							int ld_index = ll*X_STEP + ld*Y_STEP + lz;
							d_n = 0;
							pP[d_n] = global_cells[ld_index];
							if(pP[d_n].get() == NULL)
							{
								draw_xoy = false;
								 glEnd();
							}
							else if(fabs(pP[d_n]->rgb - pP[d_n+1]->rgb) < color_similar && \
								fabs(pP[d_n]->rgb - pP[d_n+2]->rgb) < color_similar)// draw this point
							{
								drawVertex(pP[d_n]);
							}
						}
					}
				  }
				if(draw_xoy) glEnd();
				
				// on xoz plane
				d_n = 0;
				glBegin(GL_TRIANGLE_STRIP);
				drawVertex(pself);
				pP[d_n++] = pself;

				bool draw_xoz = true;
				/*int ll = lx-1;
				int ld = ly-1;
				int lb = lz-1;*/
				if( lb> 0){ 
					int lb_index = lx*X_STEP + ly*Y_STEP + lb;
					pP[d_n] = global_cells[lb_index];
					if(pP[d_n].get() ==NULL)
					{
						draw_xoz = false;
						glEnd();
					}
					else if(fabs(pP[d_n]->rgb - pP[d_n-1]->rgb) < color_similar)
					{
						//d_n++;
						drawVertex(pP[d_n++]);
					}
					if(draw_xoz && ld > 0){
						int dd_index = lx*X_STEP + ld*Y_STEP + lz;
						pP[d_n] = global_cells[dd_index];
						if(pP[d_n].get()==NULL)
						{
							draw_xoz = false;
							glEnd();
						}
						else if(fabs(pP[d_n]->rgb - pP[d_n-1]->rgb) < color_similar)
						{
							//d_n++;
							drawVertex(pP[d_n++]);
						}

						if(draw_xoz){
							int bd_index = lx*X_STEP + ld*Y_STEP + lb;
							d_n = 0;
							pP[d_n] = global_cells[bd_index];
							if(pP[d_n].get() == NULL)
							{
								draw_xoz = false;
								glEnd();
							}
							else if(fabs(pP[d_n]->rgb - pP[d_n+1]->rgb) < color_similar && \
								fabs(pP[d_n]->rgb - pP[d_n+2]->rgb) < color_similar)// draw this point
							{
								drawVertex(pP[d_n]);
							}
						}
					}
				}
				if(draw_xoz) glEnd();
				

				// on yoz plane
				d_n = 0;
				glBegin(GL_TRIANGLE_STRIP);
				drawVertex(pself);
				pP[d_n++] = pself;

				bool draw_yoz = true;
				/*int ll = lx-1;
				int ld = ly-1;
				int lb = lz-1;*/
				if( ll> 0){ 
					int ll_index = ll*X_STEP + ly*Y_STEP + lz;
					pP[d_n] = global_cells[ll_index];
					if(pP[d_n].get() ==NULL)
					{
						draw_yoz = false;
						glEnd();
					}
					else if(fabs(pP[d_n]->rgb - pP[d_n-1]->rgb) < color_similar)
					{
						//d_n++;
						drawVertex(pP[d_n++]);
					}
					if(draw_yoz && lb > 0){
						int bb_index = lx*X_STEP + ly*Y_STEP + lb;
						pP[d_n] = global_cells[bb_index];
						if(pP[d_n].get()==NULL)
						{
							draw_yoz = false;
							glEnd();
						}
						else if(fabs(pP[d_n]->rgb - pP[d_n-1]->rgb) < color_similar)
						{
							//d_n++;
							drawVertex(pP[d_n++]);
						}

						if(draw_yoz){
							int lb_index = ll*X_STEP + ly*Y_STEP + lb;
							d_n = 0;
							pP[d_n] = global_cells[lb_index];
							if(pP[d_n].get() == NULL)
							{
								draw_yoz = false;
								glEnd();
							}
							else if(fabs(pP[d_n]->rgb - pP[d_n+1]->rgb) < color_similar && \
								fabs(pP[d_n]->rgb - pP[d_n+2]->rgb) < color_similar)// draw this point
							{
								drawVertex(pP[d_n]);
							}
						}
					}
				}
				if(draw_yoz) glEnd();
			  }	

		  glEndList(); 
		  return cloud_list_index;
}

void MapBuilder::drawVertex(boost::shared_ptr<pcl::PointXYZRGB> p[3],unsigned char *pucImg,float *pfVertex){
	for(size_t i=0;i<3;i++)
	{
		*(pucImg++) = p[i]->r;
		*(pucImg++) = p[i]->g;
		*(pucImg++) = p[i]->b;
		*(pfVertex++) = p[i]->x;
		*(pfVertex++) = p[i]->y;
		*(pfVertex++) = p[i]->z;
	}
	num_of_ver+=3;
	//glColor3ub(p->r,p->g,p->b);
	//glVertex3f(p->x,p->y,p->z);
}

void MapBuilder::RenderTriangle2(){
	
	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);

	glColorPointer(3,GL_UNSIGNED_BYTE,0,m_pucSLAMColor);
	glVertexPointer(3,GL_FLOAT,0,m_pfSLAMPC);
	
	glBegin(GL_TRIANGLES);
	//glBegin(GL_POINTS);
	for(size_t i=0;i<num_of_ver; i++)
		glArrayElement(i);
	glEnd();
	glFlush();
	Sleep(10);

}
// From Cells to Triangles pucImg->Color Array, pfVertex->Vertex Array
void MapBuilder::FromCell2Triangles(unsigned char *pucImg,float *pfVertex)
{
	static double color_similar = 3;
	bool d_left,d_behind,d_down;
	boost::shared_ptr<pcl::PointXYZRGB> pself;
	boost::shared_ptr<pcl::PointXYZRGB> pP[3];
	int d_n = 0;
	unsigned char *pImg = pucImg;
	float * pVertex = pfVertex;
	// create ShowList
	/*GLuint cloud_list_index = glGenLists(1);
	if(!cloud_list_index) {
	cout<<"No display list could be created"<<endl;
	return -1;
	}*/
	//glNewList(cloud_list_index, GL_COMPILE);
	//cloud_list_indices.push_back(cloud_list_index);


	double s_t = GetTickCount();
	for(int lx=0;lx<X_CELL;lx++) // traverse along x-axis
		for(int ly=0;ly<Y_CELL;ly++) // traverse along y-axis
			for(int lz=0;lz<Z_CELL;lz++) // traverse along z-axis
			{
				// initilization 

				// find point (lx,ly,lz) 
				int index_self = lx*X_STEP + ly*Y_STEP + lz; 
				pself = global_cells[index_self];
				pP[0] = pself;//global_cells[index_self];
				if(pself.get()==NULL){
					continue;
				}

				// on xoy plane
				//glBegin(GL_TRIANGLE_STRIP);
				//drawVertex(pself,pImg,pVertex);

				bool draw_xoy = true;
				int ll = lx-1; // left 
				int ld = ly-1; // down 
				int lb = lz-1; // before

				int left_p = ll*X_STEP + ly*Y_STEP + lz;
				int before_p = lx*X_STEP + ly*Y_STEP + lb;
				int down_p = lx*X_STEP + ld*Y_STEP + lz;

				// down point on xoy
				if( ll>=0 && ld >=0)
				{
					pP[1] = global_cells[ll*X_STEP + ld*Y_STEP + lz]; // down-left point
					pP[2] = global_cells[down_p]; // down point
				}
				else
					draw_xoy = false;
				if(draw_xoy && pP[1].get() != NULL && pP[2].get() != NULL && fabs(pP[1]->rgb - pP[0]->rgb) < color_similar)
				{
					drawVertex(pP,pImg,pVertex);
					pImg +=9;
					pVertex +=9;
				}
				else
					draw_xoy = false;
				if(draw_xoy)
				{
					pP[2] = global_cells[left_p]; // left point
					if(pP[2].get()!=NULL)
					{
						drawVertex(pP,pImg,pVertex);
						pImg +=9;
						pVertex +=9;
					}
				}


				// on xoz plane
				bool draw_xoz = true;
				if( lb>= 0 && ll>=0){ 
					pP[1] = global_cells[ll*X_STEP + ly*Y_STEP + lb]; // left-before point
					pP[2] = global_cells[left_p]; // left point
				}
				else
					draw_xoz = false;
				if(draw_xoz && pP[1].get() != NULL && pP[2].get()!= NULL && fabs(pP[1]->rgb - pP[0]->rgb) < color_similar)
				{
					drawVertex(pP,pImg,pVertex);		
					pImg +=9;
					pVertex +=9;
				}
				else
					draw_xoz = false;
				if(draw_xoz){
					pP[2] = global_cells[before_p]; // before point
					if(pP[2].get() != NULL)
					{
						drawVertex(pP,pImg,pVertex);
						pImg +=9;
						pVertex +=9;
					}
				}

				// on yoz plane
				bool draw_yoz = true;
				if(lb >=0 && ld>=0)
				{
					pP[1] = global_cells[lx*X_STEP + ld*Y_STEP + lb]; // before-down point
					pP[2] = global_cells[down_p];						// down point
				}
				else
					draw_yoz = false;
				if(draw_yoz && pP[1].get()!=NULL && pP[2].get()!=NULL && fabs(pP[1]->rgb - pP[0]->rgb)<color_similar)
				{
					drawVertex(pP,pImg,pVertex);
					pImg +=9;
					pVertex +=9;
				}
				else
					draw_yoz = false;
				if(draw_yoz)
				{
					pP[2] = global_cells[before_p]; // before point
					if(pP[2].get()!=NULL)
					{
						drawVertex(pP,pImg,pVertex);
						pImg +=9;
						pVertex +=9;
					}
				}
			}
			double e_t=GetTickCount();
			cout<<"Normal time consume: "<<e_t-s_t<<endl;
			return;
}
			