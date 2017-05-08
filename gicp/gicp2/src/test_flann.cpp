#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "globaldef.h"
#include "transform.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <ctime>

using namespace std;

int test_flann ()
{
    // srand (time (NULL));

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    point_cloud_ptr rawPC(new point_cloud);
    point_cloud_ptr targetPC(new point_cloud);
    
    pcl::io::loadPCDFile("person.pcd",*rawPC);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*rawPC, *targetPC, indices);
    point_cloud_ptr queryPC(new point_cloud(*targetPC));
    
    cout<<"finish reading pcds!"<<endl;

    dgc_transform_t trans;
    dgc_transform_identity(trans);
    dgc_transform_translate(trans, 0.3, -0.2, 1);
    Eigen::Matrix4f trans1;
    fromdgcTrans2Eigen(trans, trans1);

    ofstream flann_corr_out("flann_corr.log");

    pcl::KdTreeFLANN<point_type> kdtree;
    kdtree.setInputCloud (targetPC);
    /*pcl::PointXYZ*/ 
    point_type searchPoint;
    point_type rawPoint;
    float max_d_sq = pow(2,2);
    // K nearest neighbor search
    int N = queryPC->points.size();
    int K = 1;

    vector< std::vector<int> > pointIdxNKNSearch( N , vector<int>(K));
    vector< std::vector<float> > pointNKNSquaredDistance( N , vector<float>(K));
    
    cout<<"start to search in KDTree!"<<endl;
    StartTiming();
    for(int i=0;i<N;i++)
    {
        rawPoint = queryPC->points[i];
        transformPoint(searchPoint, rawPoint, trans1);
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch[i], pointNKNSquaredDistance[i]) > 0 )
        {
            for (size_t j = 0; j < pointIdxNKNSearch[i].size (); ++j)
            {
                if(pointNKNSquaredDistance[i][j] < max_d_sq)
                {
                  flann_corr_out<<i<<"\t"<<pointIdxNKNSearch[i][j]<<endl;  
                  // cout<<"matched pair: "<<i<<"\t"<<pointIdxNKNSearch[i][j]<<endl;  
                }
            }
            
               /* std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                    << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                    << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                    << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;*/
        }
    }
    double duration = StopTiming()*1000;
    cout<<"finish to search in KDTree!"<<endl;
    cout<<"Flann cost: "<<duration<<" ms!"<<endl;
    return 0;
}


/*
    // Generate pointcloud data
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
    }
    searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);
    // Neighbors within radius search

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << searchPoint.x 
        << " " << searchPoint.y 
        << " " << searchPoint.z
        << ") with radius=" << radius << std::endl;


    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }



*/

