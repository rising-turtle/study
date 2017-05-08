#ifndef OCTOVIZ_WRAPPER_H
#define OCTOVIZ_WRAPPER_H
#include "globaldef.h"
#include "octomap/octomap_types.h"
// #include <octovis/ViewerGui.h>
#include "ViewerGui.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <QObject>
#include <QWidget>
namespace octomap
{
    class OcTree;
    class Pointcloud;
    class ViewerGui;
    class ScanGraph;
}

class COctVizWrapper : public octomap::ViewerGui
{
    Q_OBJECT
public:
    COctVizWrapper(float _Resolution = 0.05);
    virtual ~COctVizWrapper();
    void InitRender();
    void setResolution(float res){m_octreeResolution = res;}
public Q_SLOTS:
    void receCameraImgs(sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfoConstPtr);
    void receColorPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr );
public:
    void addPC(octomap::Pointcloud&, octomap::point3d& );
    void addPC(octomap::Pointcloud&, octomap::point3d&, octomap::pose6d& );
    
    void addPC(point_cloud&, gl_color gc=getRed());
    void addPC(point_cloud&, octomap::pose6d& , gl_color gc=getRed());

    void addColorPC(octomap::Pointcloud&, octomap::point3d&, vector<gl_color>& );
    void addColorPC(octomap::Pointcloud&, octomap::point3d&, vector<gl_color>&, octomap::pose6d&);
    void addColorPC(color_point_cloud&);
    void addColorPC(color_point_cloud&, octomap::pose6d&);

    void prepareGraph(octomap::ScanGraph* , bool );
    bool m_bFirstPC;
    // octomap::ViewerGui* m_pViewer;
    // octomap::OcTree* m_pOcTree;
    float m_octreeResolution;

    // setting PC incremental style 
    enum PcModel{PC_Incremental, PC_OnlyOne};
    PcModel m_pc_model;
    unsigned int m_pc_id;
    void setPCIncrementalModel();
    void setPCOnlyOneModel();
};

#endif
