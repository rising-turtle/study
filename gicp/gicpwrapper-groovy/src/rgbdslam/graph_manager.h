/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
 * graph_manager.h
 *
 *  Created on: 19.01.2011
 *      Author: hess
 */

#ifndef GRAPH_MANAGER_H_
#define GRAPH_MANAGER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "node.h"
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <map>
#include <QObject>
#include <QString>
#include <QMatrix4x4>
#include <QList>
#include <QMutex>

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <memory> //for auto_ptr
#include "parameter_server.h"

#include "g2o/core/graph_optimizer_sparse.h"

#include "g2o/core/hyper_dijkstra.h"

//#define ROSCONSOLE_SEVERITY_INFO

//!Computes a globally optimal trajectory from transformations between Node-pairs
class GraphManager : public QObject {
    Q_OBJECT
    Q_SIGNALS:
    ///Connect to this signal to get the transformation matrix from the last frame as QString
    void newTransformationMatrix(QString);
    void sendFinished();
    void setGUIInfo(QString message);
    void setGUIStatus(QString message);
    void setPointCloud(pointcloud_type * pc, QMatrix4x4 transformation);
    void updateTransforms(QList<QMatrix4x4>* transformations);
    void setGUIInfo2(QString message);
    void setGraphEdges(QList<QPair<int, int> >* edge_list);
    void deleteLastNode();
    void resetGLViewer();
    
    public Q_SLOTS:
    /// Start over with new graph
    void reset();
    ///iterate over all Nodes, sending their transform and pointcloud
    void sendAllClouds();
    ///Call saveIndividualCloudsToFile, as background thread if threaded=true and possible 
    void saveIndividualClouds(QString file_basename, bool threaded=true);
    ///Call saveAllCloudsToFile,  as background thread if threaded=true and possible 
    void saveAllClouds(QString filename, bool threaded=true);
    ///Throw the last node out, reoptimize
    void deleteLastFrame(); 
    void setMaxDepth(float max_depth);
    ///Save trajectory of computed motion and ground truth (if available)
    void saveTrajectory(QString filebasename);
    void cloudRendered(pointcloud_type const * pc);
    void optimizeGraph(int iter = -1);
    void printEdgeErrors(QString);
    void pruneEdgesWithErrorAbove(float);
    void sanityCheck(float);

    public:
    GraphManager(ros::NodeHandle);
    ~GraphManager();

    //! Add new node to the graph.
    /// Node will be included, if a valid transformation to one of the former nodes
    /// can be found. If appropriate, the graph is optimized
    /// graphmanager owns newNode after this call. Do no delete the object
    /// \callergraph
    bool addNode(Node* newNode); 

    ///Draw the features's motions onto the canvas
    void drawFeatureFlow(cv::Mat& canvas, 
                         cv::Scalar line_color = cv::Scalar(255,0,0,0), 
                         cv::Scalar circle_color = cv::Scalar(0,0,255,0)); 

    bool isBusy();
    
    float Max_Depth;
    //void setMaxDepth(float max_depth);
    //!Warning: This is a dangerous way to save memory. Some methods will behave undefined after this.
    ///Notable exception: optimizeGraph()
    void deleteFeatureInformation();
protected:
    
    ///iterate over all Nodes, transform them to the fixed frame, aggregate and save
    void saveAllCloudsToFile(QString filename);
    ///iterate over all Nodes, save each in one pcd-file
    void saveIndividualCloudsToFile(QString filename);
    void pointCloud2MeshFile(QString filename, pointcloud_type full_cloud);
    std::vector < cv::DMatch > last_inlier_matches_;
    std::vector < cv::DMatch > last_matches_;

    /// The parameter max_targets determines how many potential edges are wanted
    /// max_targets < 0: No limit
    /// max_targets = 0: Compare to first frame only
    /// max_targets = 1: Compare to previous frame only
    /// max_targets > 1: Select intelligently
    QList<int> getPotentialEdgeTargets(const Node* new_node, int max_targets);
    
    //std::vector<int> getPotentialEdgeTargetsFeatures(const Node* new_node, int max_targets);
    
    bool addEdgeToG2O(const LoadedEdge3D& edge, bool good_edge, bool set_estimate=false);


    ///Send markers to visualize the graph edges (cam transforms) in rviz (if somebody subscribed)
    void visualizeGraphEdges() const;
    ///Send markers to visualize the graph nodes (cam positions) in rviz (if somebody subscribed)
    void visualizeGraphNodes() const;
    ///Send markers to visualize the graph ids in rviz (if somebody subscribed)
    void visualizeGraphIds() const;
        
    
    
    ///Send markers to visualize the last matched features in rviz (if somebody subscribed)
    void visualizeFeatureFlow3D(unsigned int marker_id = 0,
                                bool draw_outlier = true) const;
    ///Send the transform between openni_camera (the initial position of the cam)
    ///and the cumulative motion. 
    ///This is called periodically by a ros timer and after each optimizer run
    void broadcastTransform(const ros::TimerEvent& event);
    ///Constructs a list of transformation matrices from all nodes (used for visualization in glviewer)
    ///The object lives on the heap and needs to be destroyed somewhere else (i.e. in glviewer)

    //! Return pointer to a list of the optimizers graph poses on the heap(!)
    QList<QMatrix4x4>* getAllPosesAsMatrixList();
    QList<QMatrix4x4> current_poses_;
    //! Return pointer to a list of the optimizers graph edges on the heap(!)
    QList<QPair<int, int> >* getGraphEdges(); 
    QList<QPair<int, int> > current_edges_;
    void resetGraph();

    void mergeAllClouds(pointcloud_type & merge);
    double geodesicDiscount(g2o::HyperDijkstra& hypdij, const MatchingResult& mr);
    
    g2o::SparseOptimizer* optimizer_;

    ros::Publisher marker_pub_; 
    ros::Publisher ransac_marker_pub_;
    ros::Publisher whole_cloud_pub_;
    ros::Publisher batch_cloud_pub_;
    
    //!Used to start the broadcasting of the pose estimate regularly
    ros::Timer timer_;
    //!Used to broadcast the pose estimate
    tf::TransformBroadcaster br_;
    tf::Transform computed_motion_; ///<transformation of the last frame to the first frame (assuming the first one is fixed)
    tf::Transform  init_base_pose_;
    tf::Transform base2points_;//base_frame -> optical_frame 
    //Eigen::Matrix4f latest_transform_;///<same as computed_motion_ as Eigen
    QMatrix4x4 latest_transform_;///<same as computed_motion_ as Eigen


    //!Map from node id to node. Assumption is, that ids start at 0 and are consecutive
    std::map<int, Node* > graph_;
    bool reset_request_;
    unsigned int marker_id;
    int last_matching_node_;
    g2o::SE3Quat edge_to_previous_node_;
    bool batch_processing_runs_;
    bool process_node_runs_;
    QMutex optimizer_mutex;
    //cv::FlannBasedMatcher global_flann_matcher;
    
};



#endif /* GRAPH_MANAGER_H_ */
