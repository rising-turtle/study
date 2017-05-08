#include "preheader.h"
#include "globaldefinition.h"
#include "SeedHueSegmentation.hpp"
#include "VoxelGridAndNormal.h"
#include "pcl/features/normal_3d.h"
#include "pcl/point_types.h"
#include "TriangleMesh.h"
#include "CPose3D.h"
#include "boost/shared_array.hpp"

void testSegment(string file_name)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(pcl::io::loadPCDFile(file_name,*m_pc)==-1)
	{
		cout<<"failed to load file: "<<file_name<<endl;
		return ;
	}

	pcl::SeededHueSegmentation SHSegment;
	SHSegment.setInputCloud(m_pc);
	SHSegment.setDeltaHue(0.02);
	SHSegment.setClusterTolerance(0.03);
	pcl::PointIndices in_dices;
	pcl::PointIndices out_dices;
	//SHSegment.segment()

}

void testVoxelSparse(string file_name)
{
	CVoxelGridAndNormal tmpVoxelSparse(file_name);
	DisplayPT(tmpVoxelSparse.m_pOriPC);
//	DisplayPT(tmpVoxelSparse.m_pFilterPC);
}
void testSparseNormal(string file_name)
{
	CVoxelGridAndNormal tmpVS(file_name);
	cout<<"sparse size: "<<tmpVS.m_pFilterPC->points.size()<<endl;
	cout<<"normal size: "<<tmpVS.m_pNormals->points.size()<<endl;
	DisplayPTAndNormal(tmpVS.m_pFilterPC,tmpVS.m_pNormals);
}

void testNormalSegment(string file_name)
{
	CVoxelGridAndNormal tmpVS(file_name);
	tmpVS.SegmentToSurfacePatches(200);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	tmpVS.GetPatchSurfaces(tmpPC);
	DisplayPT(tmpPC);
}

void testHueSegment(string file_name)
{
	CVoxelGridAndNormal tmpVS(file_name);
	tmpVS.SegmentToSurfacePatches(200,CVoxelGridAndNormal::HUE);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	tmpVS.GetPatchSurfaces(tmpPC);
	DisplayPT(tmpPC);
}


void testHueNormalSegment(string file_name)
{
	CVoxelGridAndNormal tmpVS(file_name);
	tmpVS.SegmentToSurfacePatches(50,CVoxelGridAndNormal::HUENORMAL);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	tmpVS.GetPatchSurfaces(tmpPC);
	DisplayPT(tmpPC);
}

void testPlaneInfo(string file_name)
{
	CVoxelGridAndNormal tmpVS(file_name);
	tmpVS.SegmentToSurfacePatches(50,CVoxelGridAndNormal::HUENORMAL);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	tmpVS.GetPatchSurfaces(tmpPC);

	// 计算并获取每个patch的属性
	tmpVS.CalculateSurfaceAttribute();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr patch_centorid(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr patch_normal(new pcl::PointCloud<pcl::Normal>);
	tmpVS.GetSurfaceInfo(patch_centorid,patch_normal);

	// 显示所有的patch和其属性
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	AddPT2Viewer(viewer,tmpPC,"patch_raw_PC");
	AddPTNV2Viewer(viewer,patch_centorid,patch_normal,"patch_centorid","patch_normal");
	showViewer(viewer);
}

void testFusePlanes(string file_name)
{
	CVoxelGridAndNormal tmpVS(file_name);
	tmpVS.SegmentToSurfacePatches(30,CVoxelGridAndNormal::HUE);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nonplanePC(new pcl::PointCloud<pcl::PointXYZRGB>);

	tmpVS.CalculateSurfaceAttribute();			// 计算并获取每个patch的属性：重心+法向量
	tmpVS.FindPlanes(tmpVS.m_plane_index);		// 从patch找到相应的面
	tmpVS.BlendPlanes(tmpVS.m_plane_index);		// 合并相近的面
	tmpVS.Push2Planes();						// 将靠近面的杂点加入面中
	//tmpVS.BlendPlanes(tmpVS.m_plane_index);	// 合并相近的面
	tmpVS.DelelteSmallPlane();					// 删除很小的面
	tmpVS.AdjustPtInPlanes();					// 调整面上点的位置

	DisplayPTAndNormal(tmpVS.m_pFilterPC,tmpVS.m_pNormals);

	CPose3D pose(1,1,1,1.1,0,-0.3);
	Eigen::Matrix4f HM;
	pose.getHomogeneousMatrix(HM);
	tmpVS.TransformPatches(HM);

	DisplayPTAndNormal(tmpVS.m_pFilterPC,tmpVS.m_pNormals);
}

void getImageDepth(string file_name)
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	if(pcl::io::loadPCDFile(file_name,*m_pc)==-1){
		cout<<"failed to load file!"<<endl;
		return ;
	}

	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvGrayImage(480, 640,CV_8UC1);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// Get OpenNI visual image
	static unsigned rgb_array_size = 0;
	static boost::shared_array<unsigned char> rgb_array(0);
	static unsigned char* rgb_buffer = 0;
	rgb_array_size = 480*640*3; // size of each frame
	rgb_array.reset(new unsigned char [rgb_array_size]);
	rgb_buffer = rgb_array.get();

	// Get OpenNI depth image
	static unsigned depth_array_size = 0;
	static boost::shared_array<unsigned char> depth_array(0);
	static unsigned char* depth_buffer = 0;
	depth_array_size = 480*640*2;
	depth_array.reset(new unsigned char [depth_array_size]);
	depth_buffer = depth_array.get();

	getImagesandDepthMetaData(m_pc,rgb_buffer,depth_buffer);
	memcpy(cvRGBImage.data,rgb_buffer /*xnImd.Data()*/, 640*480*3);
	memcpy(cvDepthImage.data, depth_buffer, 640*480*2);

	cv::imwrite("d:/1.png",cvRGBImage);
	cv::imwrite("d:/2.png",cvDepthImage);

	cv::imshow("image",cvRGBImage);
	cv::imshow("depth",cvDepthImage);
	cout<<"saved!"<<endl;
	return ;

}

// 为了实验能否提高面的融合度
void testExperiment()
{
	string file_dir("D:\\MyProjects\\KinectDeviation\\cornerloc\\exper2\\");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	
		// 获取文件名
	stringstream imfs,defs;
	imfs<<file_dir<<"i"<<28<<".png";
	defs<<file_dir<<"d"<<28<<".png";

	string imf,def;
	imfs>>imf;
	defs>>def;

	// 读取数据
	cv::Mat image;
	cv::Mat depth;
	image=cv::imread(imf);
	depth=cv::imread(def,2);

	// 合成点云数据
	calPCfromImageAndDepth(image,depth,m_tmpPC);

	CVoxelGridAndNormal tmpVS(m_tmpPC);

	// 显示原始数据
	DisplayPT(tmpVS.m_pFilterPC);

	tmpVS.SegmentToSurfacePatches(30,CVoxelGridAndNormal::HUE);
	tmpVS.CalculateSurfaceAttribute();			// 计算并获取每个patch的属性：重心+法向量
	tmpVS.FindPlanes(tmpVS.m_plane_index);		// 从patch找到相应的面
	tmpVS.BlendPlanes(tmpVS.m_plane_index);		// 合并相近的面
	tmpVS.Push2Planes();						// 将靠近面的杂点加入面中
	tmpVS.BlendPlanes(tmpVS.m_plane_index);		// 合并相近的面
	//tmpVS.DelelteSmallPlane();					// 删除很小的面
	tmpVS.AdjustPtInPlanes();					// 调整面上点的位置
	tmpVS.ReduceNonPatchPoints();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nonplanePC(new pcl::PointCloud<pcl::PointXYZRGB>);
	tmpVS.GetPlanePoints(planePC,nonplanePC);
	
	// 显示面和非面的点云信息
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	AddPT2Viewer(viewer,planePC,"planePC");
	AddPT2Viewer(viewer,nonplanePC,"nonplanePC");
	showViewer(viewer);
}

void testFindPlanes(string file_name)
{
	CVoxelGridAndNormal tmpVS(file_name);
	//tmpVS.SegmentToSurfacePatches(100,CVoxelGridAndNormal::HUENORMAL);
	tmpVS.SegmentToSurfacePatches(30,CVoxelGridAndNormal::HUE);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nonplanePC(new pcl::PointCloud<pcl::PointXYZRGB>);

	// 显示每一个patch
	//cout<<"patch number: "<<tmpVS.m_surfaces.size()<<endl;
	//for(int i=0;i<tmpVS.m_surfaces.size();i++){
	//	cout<<"display patch: "<<i<<endl;
	//	tmpVS.GetPatchPoints(planePC,i);
	//	DisplayPT(planePC);
	//	//planePC->points.clear();
	//}
	//DisplayPT(tmpVS.m_pFilterPC);
	//return ;

	// 显示原始数据
	DisplayPT(tmpVS.m_pFilterPC);

	tmpVS.CalculateSurfaceAttribute();			// 计算并获取每个patch的属性：重心+法向量
	tmpVS.FindPlanes(tmpVS.m_plane_index);		// 从patch找到相应的面
	tmpVS.BlendPlanes(tmpVS.m_plane_index);		// 合并相近的面
	tmpVS.Push2Planes();						// 将靠近面的杂点加入面中
	tmpVS.BlendPlanes(tmpVS.m_plane_index);	// 合并相近的面
	//tmpVS.DelelteSmallPlane();					// 删除很小的面
	//tmpVS.AdjustPtInPlanes();					// 调整面上点的位置
	tmpVS.ReduceNonPatchPoints();
	tmpVS.GetPlanePoints(planePC,nonplanePC);
	
	//for(int i=5;i<tmpVS.m_surfaces.size();i++){
	//	if(tmpVS.m_plane_index[i]){
	//		cout<<"display patch: "<<i<<endl;
	//		tmpVS.GetPatchPoints(planePC,i);
	//		DisplayPT(planePC);
	//	}
	//	planePC->points.clear();
	//}
	//return ;
	//tmpVS.GetPatchSurfaces(planePC);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr patch_centorid(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::Normal>::Ptr patch_normal(new pcl::PointCloud<pcl::Normal>);
	//tmpVS.GetSurfaceInfo(patch_centorid,patch_normal);

	//// 显示所有的patch和其属性
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	//AddPT2Viewer(viewer,planePC,"patch_raw_PC");
	//AddPTNV2Viewer(viewer,patch_centorid,patch_normal,"patch_centorid","patch_normal");
	//showViewer(viewer);

	// 显示面和非面的点云信息
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	AddPT2Viewer(viewer,planePC,"planePC");
	AddPT2Viewer(viewer,nonplanePC,"nonplanePC");
	showViewer(viewer);
}

// 测试每一帧的矢量化结果
void testFrameSeg(string file_dir)
{
	string file_name;
	int first_=1;
	int nfiles=1;
	for(int i=0;i<nfiles;i++)
	{
		stringstream sfile;
		sfile<<file_dir<<nfiles<<".pcd";
		sfile>>file_name;
		CVoxelGridAndNormal tmpVS(file_name);
		// 对点云进行切割，然后分析每个cluster的属性
		tmpVS.SegmentToSurfacePatches(10,CVoxelGridAndNormal::HUENORMAL);		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
		tmpVS.GetPatchSurfaces(tmpPC);

		// 计算并获取每个patch的属性
		tmpVS.CalculateSurfaceAttribute();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr patch_centorid(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::Normal>::Ptr patch_normal(new pcl::PointCloud<pcl::Normal>);
		tmpVS.GetSurfaceInfo(patch_centorid,patch_normal);

		tmpVS.FindPlanes(tmpVS.m_plane_index);		// 从patch找到相应的面

		// 显示所有的patch和其属性
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		AddPT2Viewer(viewer,tmpPC,"patch_raw_PC");
		AddPTNV2Viewer(viewer,patch_centorid,patch_normal,"patch_centorid","patch_normal");
		showViewer(viewer);

		file_name.clear();
	}
}

void testPlaneAnalysis(string file_name)
{
	CVoxelGridAndNormal tmpVS(file_name);
	tmpVS.SegmentToSurfacePatches(100,CVoxelGridAndNormal::HUE);
	tmpVS.CalculateSurfaceAttribute();
	tmpVS.FindPlanes(tmpVS.m_plane_index);
	tmpVS.BlendPlanes(tmpVS.m_plane_index);
	tmpVS.AnalysisPlanes(tmpVS.m_surfType);

	//tmpVS.DisplaySurfaceInfo();

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePC(new pcl::PointCloud<pcl::PointXYZRGB>);

	//pcl::PointCloud<pcl::Normal>::Ptr tmpNV(new pcl::PointCloud<pcl::Normal>);
	//for(size_t i=0;i<tmpVS.m_surfaces.size();i++)
	//{
	//	if(tmpVS.m_plane_index[i]==false)
	//	{
	//		for(size_t j=0;j<tmpVS.m_surfaces[i].size();j++)
	//		{
	//			tmpPC->points.push_back(tmpVS.m_pFilterPC->points[tmpVS.m_surfaces[i][j]]);
	//			//tmpNV->points.push_back(tmpVS.m_pNormals->points[tmpVS.m_surfaces[i][j]]);
	//		}
	//	}
	//	else{
	//		for(size_t j=0;j<tmpVS.m_surfaces[i].size();j++)
	//		{
	//			planePC->points.push_back(tmpVS.m_pFilterPC->points[tmpVS.m_surfaces[i][j]]);
	//			//tmpNV->points.push_back(tmpVS.m_pNormals->points[tmpVS.m_surfaces[i][j]]);
	//		}
	//	}
	//	
	//}
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	//initViewer(viewer);
	//AddPT2Viewer(viewer,tmpPC,"oripc");
	//// green planes
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> tablePC_color_handler (planePC, green().r, green().g, green().b);
	//viewer->addPointCloud (planePC, tablePC_color_handler, "planePC");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "planePC");
	//showViewer(viewer);


	//DisplayPTAndNormal(tmpPC,tmpNV);

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPC1(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::Normal>::Ptr tmpNV1(new pcl::PointCloud<pcl::Normal>);
	//tmpVS.GetSurfaceInfo(tmpPC1,tmpNV1);
	//cout<<"tmpPC.size="<<tmpPC->points.size()<<endl;
	//cout<<"tmpNV.size="<<tmpNV->points.size()<<endl;
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	//initViewer(viewer);
	//AddPT2Viewer(viewer,tmpVS.m_pFilterPC,"oripc");
	//AddPTNV2Viewer(viewer,tmpPC1,tmpNV1,"centroid","normalvector");
	//showViewer(viewer);
	//DisplayPTAndNormal(tmpVS.m_pFilterPC,tmpVS.m_pNormals);
	//

	//tmpVS.AnalysisPlanes(tmpVS.m_surfType);
	//
	//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr wallPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cupboradPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr floorPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tablePC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr unknownPC(new pcl::PointCloud<pcl::PointXYZRGB>);


	tmpVS.GetPatchSurfaces(wallPC,PLANE_TYPE::WALL);
	tmpVS.GetPatchSurfaces(cupboradPC,PLANE_TYPE::CUPBOARD);
	tmpVS.GetPatchSurfaces(floorPC,PLANE_TYPE::FLOOR);
	tmpVS.GetPatchSurfaces(tablePC,PLANE_TYPE::TABLE);
	tmpVS.GetPatchSurfaces(unknownPC,PLANE_TYPE::UNKNOWN);

	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.setBackgroundColor (1, 1, 1);
	viewer.addCoordinateSystem (1.0f);
	// black wall
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> wallPC_color_handler (wallPC, black().r, black().g, black().b);
	viewer.addPointCloud (wallPC, wallPC_color_handler, "wallPC");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "wallPC");

	// blue cupboard
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cupboradPC_color_handler (cupboradPC, blue().r, blue().g, blue().b);
	viewer.addPointCloud (cupboradPC, cupboradPC_color_handler, "cupboradPC");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cupboradPC");

	// grey floor
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> floorPC_color_handler (floorPC, grey().r, grey().g, grey().b);
	viewer.addPointCloud (floorPC, floorPC_color_handler, "floorPC");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "floorPC");

	// green table
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> tablePC_color_handler (tablePC, green().r, green().g, green().b);
	viewer.addPointCloud (tablePC, tablePC_color_handler, "tablePC");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tablePC");

	// red unknown
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> unknownPC_color_handler (unknownPC, red().r, red().g, red().b);
	viewer.addPointCloud (unknownPC, unknownPC_color_handler, "unknownPC");

    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(unknownPC);
	//viewer.addPointCloud (unknownPC, rgb, "unknownPC");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "unknownPC");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePC(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(size_t i=0;i<tmpVS.m_surfaces.size();i++){
		if(tmpVS.m_plane_index[i]){
			//tmpVS.CalculateContourPoints(i,tmpPC);
			//tmpVS.CalculateEdgePoints(i,tmpPC);
			tmpVS.CaculateProjectEdgePoints(i,tmpPC);
			planePC->points.insert(planePC->points.end(),tmpPC->points.begin(),tmpPC->points.end());
			tmpPC->points.clear();
		}
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> planePC_color_handler (planePC, golden().r, golden().g, golden().b);
	viewer.addPointCloud (planePC, planePC_color_handler, "planePC");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10, "planePC");

	while (!viewer.wasStopped ())
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void testTC(string file_name)
{
	CVoxelGridAndNormal tmpVS(file_name);
	DisplayPT(tmpVS.m_pFilterPC);
	//cout<<"sparse size: "<<tmpVS.m_pFilterPC->points.size()<<endl;
	//cout<<"normal size: "<<tmpVS.m_pNormals->points.size()<<endl;
	DisplayPTAndNormal(tmpVS.m_pFilterPC,tmpVS.m_pNormals);	
}
void alternatename(string ori, string& out)
{
	static set<string> name_set;
	static map<string, int> name_index;
	if(name_set.find(ori)==name_set.end())
	{
		out=ori;
		name_set.insert(ori);
		name_index.insert(make_pair(ori,0));
	}
	else
	{
		map<string,int>::iterator it=name_index.find(ori);
		it->second++;
		int index=it->second;
		stringstream tmpss;
		tmpss<<it->first<<index;
		out=tmpss.str();
		name_set.insert(out);
	}
}

void testTriangulation(string file_name)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(pcl::io::loadPCDFile(file_name,*m_pc)==-1)
	{
		cout<<"failed to load file: "<<file_name<<endl;
		return ;
	}
	CTriangleMesh trimesh;
	boost::shared_ptr<Triangles> trindex(new Triangles);
	trimesh.compute(m_pc,trindex);

	vector<pcl::Vertices> tmpVertices;
	for(size_t i=0;i<trindex->m_indices.size();i+=3)
	{
		pcl::Vertices tmpv;
		tmpv.vertices.push_back(trindex->m_indices[i]);
		tmpv.vertices.push_back(trindex->m_indices[i+1]);
		tmpv.vertices.push_back(trindex->m_indices[i+2]);
		tmpVertices.push_back(tmpv);
	}
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	initViewer(viewer);
	viewer->addPolygonMesh<pcl::PointXYZRGB>(trindex->m_cloud,tmpVertices);
	showViewer(viewer);
}

void testTransfer(string file_name)
{
	CVoxelGridAndNormal tmpVS(file_name);
	vector<boost::shared_ptr<Triangles> > trindex;
	tmpVS.compute(trindex);

	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	initViewer(viewer);
	AddPT2Viewer(viewer,tmpVS.m_pFilterPC,"ori");
	for(size_t i=0;i<trindex.size();i++){
		//if(tmpVS.m_plane_index[i]){
			boost::shared_ptr<Triangles>& ptr=trindex[i];
			vector<pcl::Vertices> tmpVertices;
			for(size_t i=0;i<ptr->m_indices.size();i+=3)
			{
				pcl::Vertices tmpv;
				tmpv.vertices.push_back(ptr->m_indices[i]);
				tmpv.vertices.push_back(ptr->m_indices[i+1]);
				tmpv.vertices.push_back(ptr->m_indices[i+2]);
				tmpVertices.push_back(tmpv);
			}
			string polyname;
			alternatename(ptr->m_name,polyname);
			viewer->addPolygonMesh<pcl::PointXYZRGB>(ptr->m_cloud,tmpVertices,polyname);
		//}
	//	else
	//	{

	//	}
	}
	showViewer(viewer);
}

void testPC23DS(string file_in, string file_out)
{
	CVoxelGridAndNormal tmpVS;
	tmpVS.compute(file_in,file_out);
}