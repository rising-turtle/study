#include <iostream>
#include <fstream>  //��ȡtxt�ļ�
#include <vector>
#include <ctime>

#include <Eigen/Dense>  //eigen���ھ�������

//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>  //cloud_viewer
#include <pcl/features/normal_3d.h>   //���㷨����
#include <pcl/segmentation/sac_segmentation.h> //�ָ�
#include <pcl/filters/extract_indices.h>   //ƽ����ȡ
//#include <pcl/ModelCoefficients.h>

using namespace pcl;
using namespace std;
using namespace Eigen;


// calculate normals for each point and the intersection with z-axis: Jieju[]
void caculateNormals(PointCloud<PointXYZRGBA>::Ptr cloud,double normals[][3],const double radious,const double roboPose[3],double Jieju[])
{
	//r����
	NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normObj;  //�����������������
    normObj.setInputCloud (cloud);                  //�����������
    search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());   
    normObj.setSearchMethod (tree);
    normObj.setRadiusSearch (radious); //normObj.setRadiusSearch (0.1);
	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>); //����������
    normObj.compute (*cloud_normals); //���㷨����

	for(int i=0;i<cloud->points.size();i++)
	{
		double t =  (roboPose[0]-cloud->points[i].x) * cloud_normals->points[i].normal_x
			    +(roboPose[1]-cloud->points[i].y) * cloud_normals->points[i].normal_y
				+(roboPose[2]-cloud->points[i].z) * cloud_normals->points[i].normal_z; //����㼯
		if(t<0) //�������� ����������
		{
			cloud_normals->points[i].normal_x *= -1;
			cloud_normals->points[i].normal_y *= -1;
			cloud_normals->points[i].normal_z *= -1;
		}
		//Jieju[i] = (0 - cloud_normals->points[i].normal_x*cloud->points[i].x          //�˽ؾ�Ϊ�㵽��ľ���
		//			  - cloud_normals->points[i].normal_y*cloud->points[i].y
		//			  - cloud_normals->points[i].normal_z*cloud->points[i].z)
		//			  / sqrt(cloud_normals->points[i].normal_x*cloud_normals->points[i].normal_x
		//			        +cloud_normals->points[i].normal_y*cloud_normals->points[i].normal_y
		//					+cloud_normals->points[i].normal_z*cloud_normals->points[i].normal_z);
		Jieju[i] = cloud_normals->points[i].normal_x/cloud_normals->points[i].normal_z*cloud->points[i].x   //�˽ؾ�Ϊƽ����Z��ؾ�
			+ cloud_normals->points[i].normal_y/cloud_normals->points[i].normal_z*cloud->points[i].y
			+ cloud->points[i].z;
		normals[i][0] = cloud_normals->points[i].normal_x;
		normals[i][1] = cloud_normals->points[i].normal_y;
		normals[i][2] = cloud_normals->points[i].normal_z;
	}
	
}

// calculate the ground equation by 
// 1. get all normal near to (0,0,1)
// 2. ransac find the largest adjacent one
// 3. extracted these points, and caluculate its plane function 
void getGroundEquition(PointCloud<PointXYZRGBA>::Ptr cloud,double prior[5],double groundVal[4])  //prior[5] = a,b,c,groundHeight,groungError
{
	//2 ��������淶Χ��
	vector<int> groundCloudIndex;
	for(int i=0;i<cloud->points.size();i++)
	{
		if(    prior[0]*cloud->points[i].x + prior[1]*cloud->points[i].y + prior[2]*cloud->points[i].z + prior[3]+prior[4] > 0 
			&& prior[0]*cloud->points[i].x + prior[1]*cloud->points[i].y + prior[2]*cloud->points[i].z + prior[3]-prior[4] < 0)
		{groundCloudIndex.push_back(i);}
	}
	PointCloud<PointXYZ>::Ptr groundCloud(new PointCloud<PointXYZ>);
	groundCloud->width = groundCloudIndex.size();
	groundCloud->height=1;
	groundCloud->is_dense=false;
	groundCloud->resize(groundCloud->width*groundCloud->height);
	for(int i=0;i<groundCloudIndex.size();i++)
	{
		groundCloud->points[i].x = cloud->points[groundCloudIndex[i]].x;
		groundCloud->points[i].y = cloud->points[groundCloudIndex[i]].y;
		groundCloud->points[i].z = cloud->points[groundCloudIndex[i]].z;
	}
	//ͨ��RANSAC��ȡƽ��
	PointCloud<PointXYZ>::Ptr planeCloud(new PointCloud<PointXYZ>); 
	SACSegmentation<PointXYZ> seg;// �����ָ����
	seg.setOptimizeCoefficients (true);  //�Թ��Ƶ�ģ�Ͳ��������Ż����� // ��ѡ 
	seg.setModelType (SACMODEL_PLANE);  //���÷ָ�ģ�͵����   ƽ��// ��ѡ
	seg.setMethodType (SAC_RANSAC);     //������������Ĺ��Ʒ���   ransac
	seg.setMaxIterations (1000);        //��������������
	seg.setDistanceThreshold (0.005);    //�����ж��Ƿ�Ϊģ���ڵ�ľ�����ֵ //������ȡ��ƽ����
	ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
	PointIndices::Ptr inliers (new PointIndices ());
	ExtractIndices<pcl::PointXYZ> extract;  //����������ȡ����
	seg.setInputCloud (groundCloud);
	seg.segment (*inliers, *coefficients);
	if (inliers->indices.size () == 0){cout << "Could not estimate a planar model for the given dataset." << endl;return;}
	extract.setInputCloud (groundCloud);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*planeCloud);
	//4����ƽ�淽��
	MatrixXd plane(planeCloud->points.size(),3);
	for(int i=0;i<planeCloud->points.size();i++)
	{
		plane(i,0) = planeCloud->points[i].x;
		plane(i,1) = planeCloud->points[i].y;
		plane(i,2) = planeCloud->points[i].z;
	}
	MatrixXd planeMean = plane.colwise().mean();
	//����Э�������cov
	plane = plane.rowwise()-plane.colwise().mean();
	MatrixXd cov = plane.transpose()*plane/(plane.rows()-1);
	//�Խǻ�cov����÷�����normalVectoru
	SelfAdjointEigenSolver<Matrix3d> eig(cov); 
	if(eig.info()!=Success){cout<<eig.info(); abort();}
	MatrixXd P = eig.eigenvectors(); 
	groundVal[0]=P(0,0);
	groundVal[1]=P(1,0);
	groundVal[2]=P(2,0);
	groundVal[3]=-planeMean(0,0)*P(0,0)-planeMean(0,1)*P(1,0)-planeMean(0,2)*P(2,0);
	if(groundVal[2]<0)
	{
		groundVal[0]*=-1;  groundVal[1]*=-1;  groundVal[2]*=-1;  groundVal[3]*=-1;
	}
}

void viewerOneOff (visualization::PCLVisualizer& viewer)
{viewer.setBackgroundColor (0, 0, 0);}

int main(int argc,char**argv)
{
	clock_t t1,t2;
	double totaltime;
	t1 =clock();
	//��ȡ�������ݵ�data����
	ifstream fin;
	fin.open("roboLab.txt");
	if(!fin.is_open())
	{cout<<"���ļ�ʧ�ܣ�\n";return 0;}
	else
	{cout<<"���ļ��ɹ���\n";}
	//��ȡ����
	cout<<"��ȡ����..."<<endl;
	MatrixXf data(200000,3);
	int pointNum=0;
	for(int i=0;i<200000;i++)
	{
		if(fin.eof()){pointNum=i-1;break;}
		for(int j=0;j<3;j++)
		{
			fin>>data(i,j);
		}
//		double t;
		//fin>>t;fin>>t;fin>>t;  //��ȥrgb����
	}
	fin.close();
	data.conservativeResize(pointNum,3);

	//������д��cloud������
	//PointCloud<PointXYZRGBA> cloud;  //�˴���cloudΪһ������
	PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>);  //�˴���cloudΪһ������
	cloud->width=pointNum; //������ƣ���ĸ���
	cloud->height=1;//������Ʊ����ó�1
	cloud->is_dense=false;
	cloud->points.resize(cloud->width*cloud->height); //���õ��ƵĴ�С
	for(int i=0;i<cloud->points.size();i++)
	{
		cloud->points[i].x=data(i,0);
		cloud->points[i].y=data(i,1);
		cloud->points[i].z=data(i,2);
		cloud->points[i].r=255;
		cloud->points[i].g=255;
		cloud->points[i].b=255;
	}
	
	cout<<"���㷨����..."<<endl;
	double cloudNormal[200000][3];
	const double roboPose[3]={9,5,0.2};
	double Jieju[200000];
	caculateNormals(cloud,cloudNormal,0.1,roboPose,Jieju);

	cout<<"����KD��..."<<endl;
	KdTreeFLANN<PointXYZRGBA>kdtree;    //����KD��
	kdtree.setInputCloud (cloud);   //����KD�����������
	
	cout<<"�����������淽��..."<<endl;
	/////prior information
	VectorXd groundNormal(3); groundNormal<<0,0,1;
	double groundNormalError = 0.96;  //cos(errorTheta)
	double groundHigh = 0, houseHeight= 2;
	double groundHighError = 0.1;
	
	double prior[5];
	prior[0]=groundNormal[0];prior[1]=groundNormal[1];prior[2]=groundNormal[2];prior[3]=groundHigh;prior[4]=groundHighError;
	double groundVal[4];
	getGroundEquition(cloud,prior,groundVal);
	////////////////////////////////////////////////////////////////////////////////////////////////
	///////region growing//////////////////////
	int* classFlag = new int[cloud->points.size()];
	for(int i=0;i<cloud->points.size();i++){classFlag[i]=0;} //������������������ʼ��  0:δ������  -1����չʧ�ܵĵ�  1��2��3...����չ��ƽ���
	//////threshold/////////////
	double cosLevel=0.996;//0.9995;
	int minPlaneCnt =100; //����ƽ�����
	int searchK = 10;
	double pointDense[200000]={0};
	struct equition
	{
		double a;
		double b;
		double c;
		double d;   //ax+by+cz+d=0;
	}eqGround;
	eqGround.a=groundVal[0]; eqGround.b=groundVal[1]; eqGround.c=groundVal[2]; eqGround.d=groundVal[3];
	groundNormal<<groundVal[0],groundVal[1],groundVal[2];

	cout<<"ƽ����չ..."<<endl;
	int ite=1; //��������
	for(int i=0;i<cloud->points.size();i++)
	{
		vector<int> pointList;//�洢��ǰ��չ���ĵ�
		int listIdx=0;  //���list��׼����һ��ƽ����չ
		if(classFlag[i]==0)   //�ҵ�һ�����ӵ㣬��ʼһ���µ���չ
		{
			//���ӵ��ѡȡ����/////��֦/////////////////////////////////////////////////////////////////////////
			//�޸Ĵ˴�����ѡ���������棬ǽ�棬����ƽ��
			//groundHigh =  (-eqGround.a*cloud->points[i].x-eqGround.b*cloud->points[i].y-eqGround.d)/eqGround.c;
			//if(abs(cloud->points[i].z-groundHigh)>groundHighError && (cloud->points[i].z-groundHigh)<houseHeight){continue;}          //�����Լ������
			//Vector3d pointNormal;
			//pointNormal<<cloudNormal[i][0],cloudNormal[i][1],cloudNormal[i][2];
			//if(abs(pointNormal.dot(groundNormal)/(pointNormal.norm()*groundNormal.norm()))<groundNormalError){continue;}
			//Vector3d pointNormal;                            //ǽ���Լ������
			//pointNormal<<cloudNormal[i][0],cloudNormal[i][1],cloudNormal[i][2];
			//if(abs(pointNormal.dot(groundNormal)/(pointNormal.norm()*groundNormal.norm()))>0.05){continue;}
			///////////////////////////////////////////////////////////////////////////////////////////////////
			pointList.push_back(i); classFlag[i]=ite;  //�������ӵ�
			while(listIdx != pointList.size())  //��������ƽ��
			{
				vector<int> pointIdx;
				vector<float> pointSquaredDistance;
				PointXYZRGBA searchPoint;
				searchPoint.x = cloud->points[pointList[listIdx]].x;  //������������� pointList[listIdx]
				searchPoint.y = cloud->points[pointList[listIdx]].y;
				searchPoint.z = cloud->points[pointList[listIdx]].z;
				int searchCnt=kdtree.nearestKSearch(searchPoint,searchK, pointIdx, pointSquaredDistance);
				//�����ĳ��̶ܳ�
				if(searchCnt>0 && pointDense[pointList[listIdx]]==0)
				{
					for(int j=0;j<pointSquaredDistance.size();j++)
					{pointDense[pointList[listIdx]]+=pointSquaredDistance[j];}
					pointDense[pointList[listIdx]]/=pointSquaredDistance.size();
				}
				if(pointDense[pointList[listIdx]]>0.005 && pointDense[pointList[listIdx]]<0.1)
				{searchCnt=kdtree.nearestKSearch(searchPoint,searchK*2, pointIdx, pointSquaredDistance);}
				if(searchCnt>0 && pointDense[pointList[listIdx]]<0.1)
				{
					//���ݳ��̶ܳ��޸���ֵ
					double addCosLevel=0;
	            /*	if(pointDense[pointList[listIdx]]<0.0001){addCosLevel=0.0036;}
					else if(pointDense[pointList[listIdx]]>0.005){addCosLevel=-0.1;}*/
					for(int j=0;j<pointIdx.size();j++) //���������İ뾶�ڵ���з������ж�  
					{
						/////K���ڵ���չ����/////////////////////////////////////////////////////////////////////////////////////
						//condition 1:������Ѿ��жϹ��ĵ� �Ͳ���Ҫ���ж���
						if(classFlag[pointIdx[j]] != 0) {continue;}  
						//condition 2:�н�����С��ĳһ��ֵ��������
						Vector3d n1,n2,n3,n4,n5;
						n1<<cloudNormal[pointList[listIdx]][0], //������ķ�����
							cloudNormal[pointList[listIdx]][1],
							cloudNormal[pointList[listIdx]][2];
						n2<<cloudNormal[pointIdx[j]][0],        //��չ��ķ�����
							cloudNormal[pointIdx[j]][1],
							cloudNormal[pointIdx[j]][2];
						n3<<cloud->points[pointIdx[j]].x-searchPoint.x, //������->��չ��ķ�����
							cloud->points[pointIdx[j]].y-searchPoint.y,
							cloud->points[pointIdx[j]].z-searchPoint.z;
						n4 = n1.cross(n3);
						n5 = n2.cross(n3);

                        // seed growth search for plane points, the heart of this algorithm lies here
                        // 1, NV direction : n1.n2
                        // 2, NV.dot(point1-point2) lie in the same plane: n1.n3 < 0.25
                        // 3, n4.n5 is set to enhancement for 1,2 
						if(   n1.dot(n2)/(n1.norm()*n2.norm())>(cosLevel+addCosLevel)
						   && abs(n1.dot(n3)/(n1.norm()*n3.norm())) < 0.25
						   && abs(n2.dot(n3)/(n2.norm()*n3.norm())) < 0.25     
						   && n4.dot(n5)/(n4.norm()*n5.norm())>(cosLevel+addCosLevel))   //,��ʾ�ҵ��������,push��vector pointList
						{
							pointList.push_back(pointIdx[j]);
							classFlag[pointIdx[j]]=ite;
							//cloud->points[pointIdx[j]].r = 255;cloud->points[pointIdx[j]].g = 0;cloud->points[pointIdx[j]].b = 0;
						}
						/////////////////////////////////////////////////////////////////////////////////////////////////////////////
					}
				}
				listIdx++;  //���listIdx�������������Ѿ���չ����
			}//end of if  ��չ���pointList[listIdx]�������			
		}
		////������չ���ƽ������ж�///////////////////////////////
		//����1 ���������̫�٣�������������ȫ��д�����
		if(pointList.size()<minPlaneCnt)
		{
			for(int k=0;k<pointList.size();k++)
			{classFlag[pointList[k]]=-1;}
		}
		else
		{
			ite++;
			////����ƽ�����
			//MatrixXd plane(pointList.size(),3);
			//for(int j=0;j<pointList.size();j++)
			//{
			//	plane(j,0) = cloud->points[pointList[j]].x;
			//	plane(j,1) = cloud->points[pointList[j]].y;
			//	plane(j,2) = cloud->points[pointList[j]].z;
			//}	
			//MatrixXd planeMean = plane.colwise().mean();
			//plane = plane.rowwise()-plane.colwise().mean();
			//MatrixXd cov = plane.transpose()*plane/(plane.rows()-1);
			//SelfAdjointEigenSolver<Matrix3d> eig(cov); 
			//if(eig.info()!=Success){cout<<eig.info(); abort();}
			//MatrixXd P = eig.eigenvectors(); 
			//VectorXd eigValue = eig.eigenvalues();
			//if(eigValue(0)/eigValue.sum()>0.05)  ///����㼯����һ������Ϊ��ƽ��
			//{
			//	for(int k=0;k<pointList.size();k++){classFlag[pointList[k]]=-1;}
			//}
			//else
			//{
			//	//cout<<eigValue(0)<<endl;
			//	struct equition
			//	{
			//		double a;
			//		double b;
			//		double c;
			//		double d;   //ax+by+cz+d=0;
			//	}planeEq;
			//	planeEq.a=P(0,0);
			//	planeEq.b=P(1,0);
			//	planeEq.c=P(2,0);
			//	planeEq.d=-planeMean(0,0)*P(0,0)-planeMean(0,1)*P(1,0)-planeMean(0,2)*P(2,0);
			//	//cout<<"ƽ�淽�̣�"<<planeEq.a<<"*x + "<<planeEq.a<<"*y + "<<planeEq.c<<"*z = "<<planeEq.d<<endl;
			//	ite++;
			//}
		}
		///////////////////////////////////////////////////////////
	}

	cout<<"ƽ������"<<ite-1<<endl; //��չ��ƽ��count

	//t1 =clock();
	//totaltime=(double)(t1-t2)/CLOCKS_PER_SEC;
	//cout<<"��ʱ��"<<totaltime<<" S"<<endl;

	//Ϳɫ
	for(int i=0;i<cloud->points.size();i++)
	{
	
		if(classFlag[i]>0 && classFlag[i]<ite)
		{
		int colorStep = 255*255*255/(ite+3);
		int a = classFlag[i]*colorStep/(255*255)%255;;
		int b = classFlag[i]*colorStep/255%255;
		int c = classFlag[i]*colorStep%(255*255);
		cloud->points[i].r=a;cloud->points[i].g=b;cloud->points[i].b=c;
		}

	}

	///visualization
	visualization::CloudViewer viewer_origin("ʵ���ҵ���"); 
	viewer_origin.showCloud(cloud);
	viewer_origin.runOnVisualizationThreadOnce (viewerOneOff);
	while(!viewer_origin.wasStopped())
	{}
	

	return(0);
}




