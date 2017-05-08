// #include "octovis/ViewerGui.h"
#include "OctoVizWrapper.h"
#include "octomap/OcTree.h"
#include "octomap/ColorOcTree.h"
#include "ColorOctreeImpl.h"
#include "octomap/Pointcloud.h"
#include "octomap/ScanGraph.h"
#include "ros/ros.h"
#include <sstream>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

using namespace octomap;

void COctVizWrapper::setPCIncrementalModel(){m_pc_model = PC_Incremental;}
void COctVizWrapper::setPCOnlyOneModel(){m_pc_model = PC_OnlyOne;}

COctVizWrapper::COctVizWrapper(float _Resolution)
    : ViewerGui("",0),
    // m_pViewer(new ViewerGui("",NULL)),
    m_bFirstPC(true),
    // m_pOcTree(0),
    m_octreeResolution(_Resolution),
    // m_pc_model(PC_Incremental),
    m_pc_model(PC_OnlyOne),
    m_pc_id(DEFAULT_OCTREE_ID)
{
    // m_pViewer->m_occupancyThresh = _Resolution;
    InitRender();
}

COctVizWrapper::~COctVizWrapper()
{
    // delete m_pViewer;
}

void COctVizWrapper::InitRender()
{
    ui.actionSettings->setEnabled(true);
    ui.actionPointcloud->setEnabled(true);
    ui.actionPointcloud->setChecked(false);
    ui.actionTrajectory->setEnabled(true);
    ui.actionTrajectory->setChecked(true);
    ui.actionOctree_cells->setEnabled(true);
    ui.actionOctree_cells->setChecked(true);
    ui.actionOctree_structure->setEnabled(true);
    ui.actionOctree_structure->setChecked(false);
    ui.actionFree->setChecked(false);
    ui.actionFree->setEnabled(true);
    ui.actionReload_Octree->setEnabled(true);
    ui.actionConvert_ml_tree->setEnabled(true);
}

void COctVizWrapper::prepareGraph(ScanGraph* graph_, bool comp)
{
    m_scanGraph = graph_;
    loadGraph(comp);
}

void COctVizWrapper::addColorPC(color_point_cloud& pcl_pc, octomap::pose6d& frame_ori)
{
    Pointcloud oct_pc;
    point3d ori_pose;
    vector<gl_color> colors;
    // fromPCL2OctoPC(pcl_pc, oct_pc, ori_pose);
    // ColorOctreeImpl::setOctreeColor(gc);
    fromColorPCL2OctoPC(pcl_pc, oct_pc, ori_pose, colors);
    addColorPC(oct_pc, ori_pose, colors, frame_ori);
}

void COctVizWrapper::addColorPC(color_point_cloud& pcl_pc)
{
    Pointcloud oct_pc;
    vector<gl_color> colors;
    point3d ori_pose;
    // fromPCL2OctoPC(pcl_pc, oct_pc, ori_pose);
    fromColorPCL2OctoPC(pcl_pc, oct_pc, ori_pose, colors);
    // ColorOctreeImpl::setOctreeColor(gc);
    addColorPC(oct_pc, ori_pose, colors);
}

void COctVizWrapper::addPC(point_cloud& pcl_pc, gl_color gc)
{
    Pointcloud oct_pc;
    point3d ori_pose;
    fromPCL2OctoPC(pcl_pc, oct_pc, ori_pose);
    ColorOctreeImpl::setOctreeColor(gc);
    addPC(oct_pc, ori_pose);
}

void COctVizWrapper::addPC(point_cloud& pcl_pc, pose6d& frame_ori, gl_color gc)
{
    Pointcloud oct_pc;
    point3d ori_pose;
    fromPCL2OctoPC(pcl_pc, oct_pc, ori_pose);
    ColorOctreeImpl::setOctreeColor(gc);
    addPC(oct_pc, ori_pose, frame_ori);
}

void COctVizWrapper::addColorPC(Pointcloud& oct_pc, point3d& ori_pose, vector<gl_color>& colors)
{
    if(m_bFirstPC || m_pc_model == PC_Incremental) 
    {
        m_bFirstPC = false;
        // OcTree*  m_pOcTree = new octomap::OcTree(m_octreeResolution);
        // ColorOcTree* m_pOcTree = new octomap::ColorOcTree(m_octreeResolution);
        ColorOctreeImpl * m_pOcTree = new ColorOctreeImpl(m_octreeResolution);
        m_pOcTree->insertColorPointCloud(oct_pc, ori_pose, colors, m_laserMaxRange);
        this->addOctree(m_pOcTree, m_pc_id);
        OcTreeRecord * r;
        if(getOctreeRecord(DEFAULT_OCTREE_ID,r))
        {
            r->octree_drawer->enableHeightColorMode();
            // r->octree_drawer->enablePrintoutMode();
            // r->octree_drawer->enableSemanticColoring();
        }else
        {
            ROS_ERROR("Insert oct_pc error!");
        }
    }
    else
    {
        if(m_pc_model == PC_OnlyOne)
        {
            OcTreeRecord* r;
            if (!getOctreeRecord(DEFAULT_OCTREE_ID, r)) 
            {
                fprintf(stderr, "ERROR: OctreeRecord for id %d not found!\n", DEFAULT_OCTREE_ID);
                return;
            }
            //((OcTree*) r->octree)->insertPointCloud(oct_pc, ori_pose, m_laserMaxRange);
            //((ColorOcTree*) r->octree)->insertPointCloud(oct_pc, ori_pose, m_laserMaxRange);
            ((ColorOctreeImpl*) r->octree)->insertColorPointCloud(oct_pc, ori_pose, colors, m_laserMaxRange);
        }else
        {
            printf("ERROR: PC_Model must have error!\n");
        }
    }
    if(m_pc_model == PC_Incremental) ++m_pc_id;
    showOcTree();
}

void COctVizWrapper::addPC(Pointcloud& oct_pc, point3d& ori_pose)
{
    if(m_bFirstPC || m_pc_model == PC_Incremental) 
    {
        m_bFirstPC = false;
        // OcTree*  m_pOcTree = new octomap::OcTree(m_octreeResolution);
        // ColorOcTree* m_pOcTree = new octomap::ColorOcTree(m_octreeResolution);
        ColorOctreeImpl * m_pOcTree = new ColorOctreeImpl(m_octreeResolution);
        m_pOcTree->insertPointCloud(oct_pc, ori_pose, m_laserMaxRange);
        this->addOctree(m_pOcTree, m_pc_id);
        OcTreeRecord * r;
        if(getOctreeRecord(DEFAULT_OCTREE_ID,r))
        {
            r->octree_drawer->enableHeightColorMode();
            // r->octree_drawer->enablePrintoutMode();
            // r->octree_drawer->enableSemanticColoring();
        }else
        {
            ROS_ERROR("Insert oct_pc error!");
        }
    }
    else
    {
        if(m_pc_model == PC_OnlyOne)
        {
            OcTreeRecord* r;
            if (!getOctreeRecord(DEFAULT_OCTREE_ID, r)) 
            {
                fprintf(stderr, "ERROR: OctreeRecord for id %d not found!\n", DEFAULT_OCTREE_ID);
                return;
            }
            //((OcTree*) r->octree)->insertPointCloud(oct_pc, ori_pose, m_laserMaxRange);
            //((ColorOcTree*) r->octree)->insertPointCloud(oct_pc, ori_pose, m_laserMaxRange);
            ((ColorOctreeImpl*) r->octree)->insertPointCloud(oct_pc, ori_pose, m_laserMaxRange);
        }else
        {
            printf("ERROR: PC_Model must have error!\n");
        }
    }
    if(m_pc_model == PC_Incremental) ++m_pc_id;
    showOcTree();
}

void COctVizWrapper::addColorPC(Pointcloud& oct_pc, point3d& ori_pose, vector<gl_color>& colors, pose6d& frame_ori)
{
    if( m_bFirstPC || m_pc_model == PC_Incremental) 
    {
        m_bFirstPC = false;
        // OcTree*  m_pOcTree = new octomap::OcTree(m_octreeResolution);
        // ColorOcTree* m_pOcTree = new octomap::ColorOcTree(m_octreeResolution);
        ColorOctreeImpl * m_pOcTree = new ColorOctreeImpl(m_octreeResolution);
        m_pOcTree->insertColorPointCloud(oct_pc, ori_pose, colors, frame_ori,  m_laserMaxRange);
        this->addOctree(m_pOcTree, m_pc_id);
        OcTreeRecord * r;
        if(getOctreeRecord(m_pc_id,r))
        {
            r->octree_drawer->enableHeightColorMode();
            // r->octree_drawer->enablePrintoutMode();
            // r->octree_drawer->enableSemanticColoring();
        }else
        {
            ROS_ERROR("Insert oct_pc error!");
        }
    }
    else
    {
        if(m_pc_model == PC_OnlyOne)
        {
            OcTreeRecord* r;
            if (!getOctreeRecord(m_pc_id, r)) 
            {
                fprintf(stderr, "ERROR: OctreeRecord for id %d not found!\n", DEFAULT_OCTREE_ID);
                return;
            }
            //((OcTree*) r->octree)->insertPointCloud(oct_pc, ori_pose, m_laserMaxRange);
            //((ColorOcTree*) r->octree)->insertPointCloud(oct_pc, ori_pose, m_laserMaxRange);
            ((ColorOctreeImpl*) r->octree)->insertColorPointCloud(oct_pc, ori_pose, colors, frame_ori, m_laserMaxRange);
        }else
        {
            printf("ERROR: PC_Model must have error!\n");
        }
    }
    if(m_pc_model == PC_Incremental) ++m_pc_id;
    showOcTree();
}

void COctVizWrapper::addPC(Pointcloud& oct_pc, point3d& ori_pose, pose6d& frame_ori)
{
    if( m_bFirstPC || m_pc_model == PC_Incremental) 
    {
        m_bFirstPC = false;
        // OcTree*  m_pOcTree = new octomap::OcTree(m_octreeResolution);
        // ColorOcTree* m_pOcTree = new octomap::ColorOcTree(m_octreeResolution);
        ColorOctreeImpl * m_pOcTree = new ColorOctreeImpl(m_octreeResolution);
        m_pOcTree->insertPointCloud(oct_pc, ori_pose, frame_ori, m_laserMaxRange);
        this->addOctree(m_pOcTree, m_pc_id);
        OcTreeRecord * r;
        if(getOctreeRecord(m_pc_id,r))
        {
            r->octree_drawer->enableHeightColorMode();
            // r->octree_drawer->enablePrintoutMode();
            // r->octree_drawer->enableSemanticColoring();
        }else
        {
            ROS_ERROR("Insert oct_pc error!");
        }
    }
    else
    {
        if(m_pc_model == PC_OnlyOne)
        {
            OcTreeRecord* r;
            if (!getOctreeRecord(m_pc_id, r)) 
            {
                fprintf(stderr, "ERROR: OctreeRecord for id %d not found!\n", DEFAULT_OCTREE_ID);
                return;
            }
            //((OcTree*) r->octree)->insertPointCloud(oct_pc, ori_pose, m_laserMaxRange);
            //((ColorOcTree*) r->octree)->insertPointCloud(oct_pc, ori_pose, m_laserMaxRange);
            ((ColorOctreeImpl*) r->octree)->insertPointCloud(oct_pc, ori_pose, m_laserMaxRange);
        }else
        {
            printf("ERROR: PC_Model must have error!\n");
        }
    }
    if(m_pc_model == PC_Incremental) ++m_pc_id;
    showOcTree();
}

void COctVizWrapper::receCameraImgs(sensor_msgs::ImageConstPtr pimg, sensor_msgs::ImageConstPtr pdpt, sensor_msgs::CameraInfoConstPtr pcamera)
{
    static unsigned int ncout = 0;
    if(ncout % 3 == 0)
    {
        static unsigned int n = 0;
        if(n<13)
        {
            stringstream ss1,ss2;
            ss1<<++n<<"image.jpg";
            ss2<<n<<"depth.jpg";
            cv_bridge::CvImagePtr image_pImg = cv_bridge::toCvCopy(pimg);
            cv_bridge::CvImagePtr depth_pImg = cv_bridge::toCvCopy(pdpt);
            cv::imwrite(ss1.str().c_str(), image_pImg->image);
            cv::imwrite(ss2.str().c_str(), depth_pImg->image);
            // pcl::io::savePCDFile(ss.str().c_str(),*pc);
        }
    }

    ROS_INFO_STREAM("OctWrpper rece camera imgs: "<<++ncout<<" ");
}

void COctVizWrapper::receColorPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc)
{
    static unsigned int ncout = 0;
    if(ncout % 3 == 0)
    {
        static unsigned int n = 0;
        if(n<13)
        {
            stringstream ss;
            ss<<++n<<".pcd";
            pcl::io::savePCDFile(ss.str().c_str(),*pc);
        }
    }
    ROS_INFO_STREAM("OctWrapper rece ColorPointCloud: "<<++ncout);
}
