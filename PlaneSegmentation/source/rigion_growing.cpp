#include <iostream>
#include <fstream>  //读取txt文件
#include <vector>
#include <ctime>

#include <Eigen/Dense>  //eigen用于矩阵运算

//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>  //cloud_viewer
#include <pcl/features/normal_3d.h>   //计算法向量
#include <pcl/segmentation/sac_segmentation.h> //分割
#include <pcl/filters/extract_indices.h>   //平面提取
//#include <pcl/ModelCoefficients.h>

using namespace pcl;
using namespace std;
using namespace Eigen;


// calculate normals for each point and the intersection with z-axis: Jieju[]
void caculateNormals(PointCloud<PointXYZRGBA>::Ptr cloud,double normals[][3],const double radious,const double roboPose[3],double Jieju[])
{
	//r领域
	NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normObj;  //创建法向量计算对象
    normObj.setInputCloud (cloud);                  //设置输入点云
    search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());   
    normObj.setSearchMethod (tree);
    normObj.setRadiusSearch (radious); //normObj.setRadiusSearch (0.1);
	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>); //创建法向量
    normObj.compute (*cloud_normals); //计算法向量

	for(int i=0;i<cloud->points.size();i++)
	{
		double t =  (roboPose[0]-cloud->points[i].x) * cloud_normals->points[i].normal_x
			    +(roboPose[1]-cloud->points[i].y) * cloud_normals->points[i].normal_y
				+(roboPose[2]-cloud->points[i].z) * cloud_normals->points[i].normal_z; //计算点集
		if(t<0) //如果是锐角 法向量反向
		{
			cloud_normals->points[i].normal_x *= -1;
			cloud_normals->points[i].normal_y *= -1;
			cloud_normals->points[i].normal_z *= -1;
		}
		//Jieju[i] = (0 - cloud_normals->points[i].normal_x*cloud->points[i].x          //此截距为点到面的距离
		//			  - cloud_normals->points[i].normal_y*cloud->points[i].y
		//			  - cloud_normals->points[i].normal_z*cloud->points[i].z)
		//			  / sqrt(cloud_normals->points[i].normal_x*cloud_normals->points[i].normal_x
		//			        +cloud_normals->points[i].normal_y*cloud_normals->points[i].normal_y
		//					+cloud_normals->points[i].normal_z*cloud_normals->points[i].normal_z);
		Jieju[i] = cloud_normals->points[i].normal_x/cloud_normals->points[i].normal_z*cloud->points[i].x   //此截距为平面与Z轴截距
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
	//2 分离出地面范围点
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
	//通过RANSAC提取平面
	PointCloud<PointXYZ>::Ptr planeCloud(new PointCloud<PointXYZ>); 
	SACSegmentation<PointXYZ> seg;// 创建分割对象
	seg.setOptimizeCoefficients (true);  //对估计的模型参数进行优化处理 // 可选 
	seg.setModelType (SACMODEL_PLANE);  //设置分割模型的类别   平面// 必选
	seg.setMethodType (SAC_RANSAC);     //设置随机参数的估计方法   ransac
	seg.setMaxIterations (1000);        //设置最大迭代次数
	seg.setDistanceThreshold (0.005);    //设置判断是否为模型内点的距离阈值 //设置提取的平面厚度
	ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
	PointIndices::Ptr inliers (new PointIndices ());
	ExtractIndices<pcl::PointXYZ> extract;  //创建点云提取对象
	seg.setInputCloud (groundCloud);
	seg.segment (*inliers, *coefficients);
	if (inliers->indices.size () == 0){cout << "Could not estimate a planar model for the given dataset." << endl;return;}
	extract.setInputCloud (groundCloud);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*planeCloud);
	//4计算平面方程
	MatrixXd plane(planeCloud->points.size(),3);
	for(int i=0;i<planeCloud->points.size();i++)
	{
		plane(i,0) = planeCloud->points[i].x;
		plane(i,1) = planeCloud->points[i].y;
		plane(i,2) = planeCloud->points[i].z;
	}
	MatrixXd planeMean = plane.colwise().mean();
	//计算协方差矩阵cov
	plane = plane.rowwise()-plane.colwise().mean();
	MatrixXd cov = plane.transpose()*plane/(plane.rows()-1);
	//对角化cov，求得法向量normalVectoru
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
	//读取点云数据到data数组
	ifstream fin;
	fin.open("roboLab.txt");
	if(!fin.is_open())
	{cout<<"打开文件失败！\n";return 0;}
	else
	{cout<<"打开文件成功！\n";}
	//读取数据
	cout<<"读取数据..."<<endl;
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
		//fin>>t;fin>>t;fin>>t;  //舍去rgb分量
	}
	fin.close();
	data.conservativeResize(pointNum,3);

	//将数据写到cloud对象中
	//PointCloud<PointXYZRGBA> cloud;  //此处的cloud为一个对象
	PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>);  //此处的cloud为一个对象
	cloud->width=pointNum; //无序点云，点的个数
	cloud->height=1;//无序点云被设置成1
	cloud->is_dense=false;
	cloud->points.resize(cloud->width*cloud->height); //设置点云的大小
	for(int i=0;i<cloud->points.size();i++)
	{
		cloud->points[i].x=data(i,0);
		cloud->points[i].y=data(i,1);
		cloud->points[i].z=data(i,2);
		cloud->points[i].r=255;
		cloud->points[i].g=255;
		cloud->points[i].b=255;
	}
	
	cout<<"计算法向量..."<<endl;
	double cloudNormal[200000][3];
	const double roboPose[3]={9,5,0.2};
	double Jieju[200000];
	caculateNormals(cloud,cloudNormal,0.1,roboPose,Jieju);

	cout<<"建立KD树..."<<endl;
	KdTreeFLANN<PointXYZRGBA>kdtree;    //创建KD树
	kdtree.setInputCloud (cloud);   //设置KD树的输入点云
	
	cout<<"由先验计算地面方程..."<<endl;
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
	for(int i=0;i<cloud->points.size();i++){classFlag[i]=0;} //创建聚类索引，并初始化  0:未搜索点  -1：拓展失败的点  1，2，3...：拓展的平面点
	//////threshold/////////////
	double cosLevel=0.996;//0.9995;
	int minPlaneCnt =100; //最少平面点数
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

	cout<<"平面拓展..."<<endl;
	int ite=1; //迭代次数
	for(int i=0;i<cloud->points.size();i++)
	{
		vector<int> pointList;//存储当前拓展到的点
		int listIdx=0;  //清空list，准备新一个平面拓展
		if(classFlag[i]==0)   //找到一个种子点，开始一次新的拓展
		{
			//种子点的选取条件/////剪枝/////////////////////////////////////////////////////////////////////////
			//修改此处可以选择搜索地面，墙面，任意平面
			//groundHigh =  (-eqGround.a*cloud->points[i].x-eqGround.b*cloud->points[i].y-eqGround.d)/eqGround.c;
			//if(abs(cloud->points[i].z-groundHigh)>groundHighError && (cloud->points[i].z-groundHigh)<houseHeight){continue;}          //地面的约束条件
			//Vector3d pointNormal;
			//pointNormal<<cloudNormal[i][0],cloudNormal[i][1],cloudNormal[i][2];
			//if(abs(pointNormal.dot(groundNormal)/(pointNormal.norm()*groundNormal.norm()))<groundNormalError){continue;}
			//Vector3d pointNormal;                            //墙面的约束条件
			//pointNormal<<cloudNormal[i][0],cloudNormal[i][1],cloudNormal[i][2];
			//if(abs(pointNormal.dot(groundNormal)/(pointNormal.norm()*groundNormal.norm()))>0.05){continue;}
			///////////////////////////////////////////////////////////////////////////////////////////////////
			pointList.push_back(i); classFlag[i]=ite;  //存入种子点
			while(listIdx != pointList.size())  //搜索整个平面
			{
				vector<int> pointIdx;
				vector<float> pointSquaredDistance;
				PointXYZRGBA searchPoint;
				searchPoint.x = cloud->points[pointList[listIdx]].x;  //带搜索点的索引 pointList[listIdx]
				searchPoint.y = cloud->points[pointList[listIdx]].y;
				searchPoint.z = cloud->points[pointList[listIdx]].z;
				int searchCnt=kdtree.nearestKSearch(searchPoint,searchK, pointIdx, pointSquaredDistance);
				//计算点的稠密程度
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
					//根据稠密程度修改阈值
					double addCosLevel=0;
	            /*	if(pointDense[pointList[listIdx]]<0.0001){addCosLevel=0.0036;}
					else if(pointDense[pointList[listIdx]]>0.005){addCosLevel=-0.1;}*/
					for(int j=0;j<pointIdx.size();j++) //对搜索到的半径内点进行法向量判断  
					{
						/////K近邻点拓展条件/////////////////////////////////////////////////////////////////////////////////////
						//condition 1:如果是已经判断过的点 就不需要再判断了
						if(classFlag[pointIdx[j]] != 0) {continue;}  
						//condition 2:夹角余弦小于某一阈值，法向量
						Vector3d n1,n2,n3,n4,n5;
						n1<<cloudNormal[pointList[listIdx]][0], //搜索点的法向量
							cloudNormal[pointList[listIdx]][1],
							cloudNormal[pointList[listIdx]][2];
						n2<<cloudNormal[pointIdx[j]][0],        //拓展点的法向量
							cloudNormal[pointIdx[j]][1],
							cloudNormal[pointIdx[j]][2];
						n3<<cloud->points[pointIdx[j]].x-searchPoint.x, //搜索点->拓展点的法向量
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
						   && n4.dot(n5)/(n4.norm()*n5.norm())>(cosLevel+addCosLevel))   //,表示找到了区域点,push到vector pointList
						{
							pointList.push_back(pointIdx[j]);
							classFlag[pointIdx[j]]=ite;
							//cloud->points[pointIdx[j]].r = 255;cloud->points[pointIdx[j]].g = 0;cloud->points[pointIdx[j]].b = 0;
						}
						/////////////////////////////////////////////////////////////////////////////////////////////////////////////
					}
				}
				listIdx++;  //如果listIdx到达最后，则表明已经拓展完了
			}//end of if  拓展完点pointList[listIdx]的邻域点			
		}
		////对于拓展完的平面进行判断///////////////////////////////
		//条件1 ：如果个数太少，则舍弃；否则全部写入分类
		if(pointList.size()<minPlaneCnt)
		{
			for(int k=0;k<pointList.size();k++)
			{classFlag[pointList[k]]=-1;}
		}
		else
		{
			ite++;
			////计算平面参数
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
			//if(eigValue(0)/eigValue.sum()>0.05)  ///如果点集中在一起，则不认为是平面
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
			//	//cout<<"平面方程："<<planeEq.a<<"*x + "<<planeEq.a<<"*y + "<<planeEq.c<<"*z = "<<planeEq.d<<endl;
			//	ite++;
			//}
		}
		///////////////////////////////////////////////////////////
	}

	cout<<"平面数量"<<ite-1<<endl; //扩展的平面count

	//t1 =clock();
	//totaltime=(double)(t1-t2)/CLOCKS_PER_SEC;
	//cout<<"用时："<<totaltime<<" S"<<endl;

	//涂色
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
	visualization::CloudViewer viewer_origin("实验室点云"); 
	viewer_origin.showCloud(cloud);
	viewer_origin.runOnVisualizationThreadOnce (viewerOneOff);
	while(!viewer_origin.wasStopped())
	{}
	

	return(0);
}




