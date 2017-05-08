#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h> //caculate normal

#include "plane_segmentation.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense> 
#include <iostream>
#include <ctime>


#ifdef _WIN32
# define sleep(x) sleep((x)*1000)
#endif

using namespace pcl;
using namespace std;
using namespace Eigen;

class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}
	//�ص�����
	void cloud_cb_ (const PointCloud<PointXYZRGBA>::ConstPtr &rawCloud)
	{
		/////////////////////////////////////////////////////
		////////do something with rawCloud//////////////////
		////////////////////////////////////////////////////
		//1 �������
		clock_t tStart=clock();
		PointCloud<PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<PointXYZRGBA>);
		vector<int> indice;
		for(int i=0;i<rawCloud->size();i++)
		{if(i/rawCloud->width%2==0 && i%rawCloud->width%2==0){indice.push_back(i);}}
		copyPointCloud(*rawCloud,indice,*cloud);	
		cloud->width=rawCloud->width/2;cloud->height=rawCloud->height/2;cloud->resize(cloud->width*cloud->height);

		//2 ���㷨����
		clock_t normalStart=clock();
		PointCloud<Normal>::Ptr normals (new pcl::PointCloud<Normal>);
		IntegralImageNormalEstimation<PointXYZRGBA, Normal> ne;
		ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
		ne.setMaxDepthChangeFactor(0.1f);
		ne.setNormalSmoothingSize(8.0f);
		ne.setInputCloud(cloud);
		ne.compute(*normals);
		//clock_t normalEnd=clock();
		
		//cout<<"Time for caculating normal: "<<(double)(normalEnd-normalStart)/CLOCKS_PER_SEC<<" S"<<endl;

		//3 ��������
		//////change data structure  from  PointXYZRGBA to double [][]
		float ** cloudArray = new float *[cloud->size()];
		float ** normalsArray = new float *[normals->size()];
		int * classFlag = new int[cloud->size()];
		for(int i=0;i<cloud->size();i++)
		{
			cloudArray[i] = new float[3];
			normalsArray[i] = new float[3];
		}
		for(int i=0;i<cloud->size();i++)
		{
			if(finite(cloud->points[i].x) && finite(cloud->points[i].y) && finite(cloud->points[i].z))
			{
				cloudArray[i][0]=cloud->points[i].x;
				cloudArray[i][1]=cloud->points[i].y;
				cloudArray[i][2]=cloud->points[i].z;
			}
			else
			{
				cloudArray[i][0]=cloudArray[i][1]=cloudArray[i][2]=std::numeric_limits<float>::quiet_NaN ();
			}
			if(finite(normals->points[i].normal_x) && finite(normals->points[i].normal_y) && finite(normals->points[i].normal_z))
			{
				normalsArray[i][0]=normals->points[i].normal_x;
				normalsArray[i][1]=normals->points[i].normal_y;
				normalsArray[i][2]=normals->points[i].normal_z;
			}
			else
			{
				normalsArray[i][0]=normalsArray[i][1]=normalsArray[i][2]=std::numeric_limits<float>::quiet_NaN ();
			}
		}
		//MODULE��PlaneSegmentation  �����������ƽ����Ϣ  
		// 1.PlaneModel ��ax+by+cz+d=0  ���ÿ��ƽ���a,b,c,d
		// 2.classFlag  : �����е��Ƶ�һ�����λ��<0��ʾ��ƽ��㣬>=0�����ֱ�ʾ�õ���ݵ�ƽ����ţ���Ӧequi�е�ƽ����ţ�
		PlaneSegmentation seg;
		seg.segmentOrganized(cloudArray,normalsArray,cloud->width,cloud->height,classFlag);
		//��������seg�Ĳ�������,���õ�Ĭ�ϲ���
		seg.setMaxVisDistance(5);     ////�Ӿ�ԽԶ��ƽ����ȡ��׼ȷ��Խ�ͣ�2����׼ȷ�Ƚϸ�
		seg.setPlaneThreshold(100,5);
		vector<double> equi[4];
		seg.gettPlaneModel(equi);

		/*for(int i=0;i<equi[0].size();i++)
		{
			cout<<"Plane #"<<i+1<<" equition:"<<endl;
			cout<<equi[0][i]<<" * x  +  "<<equi[1][i]<<" * y  +  "<<equi[2][i]<<" * z  +  "<<equi[3][i]<<"  = 0"<<endl<<endl;
		}*/

		//draw
		int red[8]={30,60,90,120,150,180,210,240};
		int grn[8]={30,60,90,120,150,180,210,240};
		int blu[8]={30,60,90,120,150,180,210,240};
		for(int i=0;i<cloud->size();i++)
		{
			if(classFlag[i]>=0)
			{
				cloud->points[i].r=red[classFlag[i]*3%8];
				cloud->points[i].g=grn[(classFlag[i]+2)*5%8];
				cloud->points[i].b=blu[(classFlag[i]+5)*7%8];
			}
			else
			{
				cloud->points[i].r=0;
				cloud->points[i].g=0;
				cloud->points[i].b=255;
			}
		}
		//visulization
		if (!viewer.wasStopped())
		{
			viewer.showCloud (cloud);
		}
		time_t tEnd=clock();
		cout<<"Planes count in current Frame: "<<equi->size()<<endl;
		cout<<"Total Time For Frame: "<<(double)(tEnd-tStart)/CLOCKS_PER_SEC<<" S"<<endl<<endl;
		char txt_front[1024];
		sprintf(txt_front, "%ld.pcd", tStart);
		//pcl::io::savePCDFile(txt_front, *rawCloud);

		for(int i=0;i<cloud->size();i++)
		{
			delete [] cloudArray[i];
			delete [] normalsArray[i];
		}
		delete [] cloudArray;
		delete [] normalsArray;
		delete [] classFlag;
	}

	void run ()
	{
		Grabber* interface = new OpenNIGrabber();   //����һ��OpenNIGrabber�ӿ�

		boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
			boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

		interface->registerCallback (f);
		interface->start();
		while (!viewer.wasStopped())//�رջص�����󣬻�Ҫ�˳�ѭ�����ſ��Լ�����������
		{
			boost::this_thread::sleep(boost::posix_time::seconds (2));
		}
		interface->stop ();
	}

	visualization::CloudViewer viewer;
};

int main ()
{
	SimpleOpenNIViewer v;
	//SimpleOpenNIProcessor v;
	v.run ();
	return 0;
}
