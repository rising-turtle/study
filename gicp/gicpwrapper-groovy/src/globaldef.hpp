
// extern void readPcdAndPose(const char*, std::vector<point_cloud_ptr>& , std::vector<octomap::pose6d>&, int num = -1, int step = 5);

template<typename PointT>
void readPcdAndPose(const char* fs, std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > >& pcs, std::vector<octomap::pose6d>& poses, int num =-1, int step=5)
{
    CFileReader files;
    files.readDir(fs);
    string pcdf;
    string posef;
    octomap::pose6d pose;
    int tn;
    if(num<0) tn = files.m_lastnum;
    else tn = num < files.m_lastnum ? num : files.m_lastnum;
    for(int i=files.m_firstnum; i<= tn; i+=step)
    {
        boost::shared_ptr<pcl::PointCloud<PointT> > pc(new pcl::PointCloud<PointT>);
        // 1, get pcd & pose file
        files.getPoseFile(i, posef);
        files.getPcdFile(i, pcdf);
        // 2, get point cloud & pose6d 
        pcl::io::loadPCDFile(pcdf, *pc);
        pcs.push_back(pc);
        readPose(posef.c_str(), pose);
        poses.push_back(pose);
    }
}

template<typename PointT>
void fromColorPCL2OctoPC(pcl::PointCloud<PointT>& pcl_pc, octomap::Pointcloud& oct_pc, octomap::point3d& ori_pose, vector<gl_color>& color)
{
    octomap::point3d oc_pt;
    for(int i=0;i<pcl_pc.points.size();i++)
    {
        PointT& pt = pcl_pc.points[i];
        if(!pcl_isfinite(pt.x) || \
                !pcl_isfinite(pt.y) || \
                !pcl_isfinite(pt.z))
            continue;
        oc_pt(0) = pt.x;
        oc_pt(1) = pt.y;
        oc_pt(2) = pt.z;
        oct_pc.push_back(oc_pt);
        color.push_back(gl_color(pt.r,pt.g,pt.b));
    }   
    Eigen::Vector4f& _pose = pcl_pc.sensor_origin_;
    ori_pose(0) = _pose[0];
    ori_pose(1) = _pose[1];
    ori_pose(2) = _pose[2];
}

