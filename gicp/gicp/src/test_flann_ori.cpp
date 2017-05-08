#include "globaldef.h"
#include <iostream>
#include <fstream>
#include "transform.h"
#include "Eigen/Core"
#include "flann/flann.hpp"
#include "boost/shared_ptr.hpp"
#include "pcl/point_types.h"
#include "pcl/filters/filter.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_representation.h"
#include <pcl/common/transforms.h>

using namespace flann;
using namespace std;
typedef boost::shared_ptr<pcl::PointRepresentation<point_type> > PointRepresentationPtr;

/** \brief Converts a PointCloud to the internal FLANN point array representation. Returns the number
 * of points.
 * \param cloud the PointCloud 
 */
float* convertCloudToArray (const point_cloud &cloud)
{
    float *cloud_ = 0;
    static std::vector<int> index_mapping_;
    bool identity_mapping_ = true;
    PointRepresentationPtr point_representation_(new pcl::DefaultPointRepresentation<point_type>);
    int dim_ = point_representation_->getNumberOfDimensions();

    cout<<"dim is: "<<dim_<<endl;
    // No point in doing anything if the array is empty
    if (cloud.empty ())
    {
        cloud_ = NULL;
        cout<<"cloud is empty!"<<endl;
        return;
    }
    
    int original_no_of_points = static_cast<int> (cloud.points.size());
    cout<<"num is : "<<original_no_of_points<<endl;
    cloud_ = static_cast<float*> (malloc (original_no_of_points * dim_ * sizeof (float)));
    float* cloud_ptr = cloud_;
    index_mapping_.reserve (original_no_of_points);
    identity_mapping_ = true;

    for (int cloud_index = 0; cloud_index < original_no_of_points; ++cloud_index)
    {
        // Check if the point is invalid
        // if (!isRowValid (cloud.points[cloud_index]))
        if (!point_representation_->isValid (cloud.points[cloud_index]))
        {
            printf("idnetity is false!\n");
            identity_mapping_ = false;
            continue;
        }

        index_mapping_.push_back (cloud_index);
        point_representation_->vectorize (cloud.points[cloud_index], cloud_ptr);
        cloud_ptr+=dim_;
        /*
        for (size_t i = 0; i < static_cast<size_t> (dim_); ++i)
        {
            *cloud_ptr = cloud.points.coeffRef (cloud_index, i);
            cloud_ptr++;
        }*/
    }
    cout<<"finish coversion!"<<endl;
    return cloud_;
}

void test_flann_ori()
{
    // 1 read pcds
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    point_cloud_ptr rawPC(new point_cloud);
    point_cloud_ptr targetPC(new point_cloud);

    pcl::io::loadPCDFile("person.pcd",*rawPC);
    std::vector<int> indices2;
    pcl::removeNaNFromPointCloud(*rawPC, *targetPC, indices2);
    point_cloud_ptr queryPC(new point_cloud(*targetPC));
    point_cloud_ptr searchPC(new point_cloud);
    cout<<"finished reading pcds!"<<endl;
    
    // 2 set transformations
    dgc_transform_t trans;
    dgc_transform_identity(trans);
    dgc_transform_translate(trans, 0.3, -0.2, 1);
    Eigen::Matrix4f trans1;
    fromdgcTrans2Eigen(trans, trans1);

    // 3 build kd-tree
    PointRepresentationPtr point_representation_(new pcl::DefaultPointRepresentation<point_type>);
    int dim_ = point_representation_->getNumberOfDimensions();
    int n_ = targetPC->points.size();
    float* pdata = convertCloudToArray(*targetPC);
    // float* pquery = new float[n_*dim_];
    // memcpy(pquery, pdata, n_*dim_*sizeof(float));
    Matrix<float> dataset(pdata, n_, dim_);
    // Matrix<float> query(pquery, n_, dim_);
    
    int nn = 3;
    int k  = nn;
    // Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
    // Matrix<float> dists(new float[query.rows*nn], query.rows, nn);

    // construct an randomized kd-tree index using 4 kd-trees
    // Index<L2<float> > index(dataset, flann::KDTreeIndexParams(4));
    Index<L2_Simple<float> > index(dataset, flann::KDTreeSingleIndexParams(15));
    index.buildIndex();                                                                                                   
    cout<<"after building index!"<<endl;
    // struct timeval st, et; 
    // gettimeofday(&st, 0); 
    
    // 4 start search in flann kd-tree
    float max_d_sq = pow(2,2);
    point_type rawPoint, searchPoint;
    ofstream flann_corr_ori_out("flann_ori.log");
    vector<float> vquery(dim_);
    vector<int> k_indices(k);
    vector<float> k_distances(k);
    int nchecks = 16;
    pcl::transformPointCloud(*queryPC, *searchPC, trans1);
    float* pquerydata = convertCloudToArray(*searchPC);
    flann::Matrix<float> mquery(pquerydata, n_, dim_);
    Matrix<int> indices(new int[mquery.rows*nn], mquery.rows, nn);
    Matrix<float> dists(new float[mquery.rows*nn], mquery.rows, nn);
    StartTiming();
    index.knnSearch(mquery, indices, dists, nn, flann::SearchParams(-1));
    double durantion = StopTiming()*1000;
    printf("cost time: %lf ms \n", durantion);


    for(int i=0;i<n_;i++)
    {
        for(int j=0;j<k;j++)
        {   
            if(*(dists[i*k+j]) < max_d_sq)
            {
                 flann_corr_ori_out<<i<<"\t"<<*(indices[i*k+j])<<endl;
            }
        }
    }
    
    // do a knn search, using 128 checks
    // index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
    
    /*for(int i=0;i<n_;i++)
    {
        rawPoint = queryPC->points[i];
        transformPoint(searchPoint, rawPoint, trans1);
        point_representation_->vectorize (static_cast<point_type> (searchPoint), vquery);

        flann::Matrix<int> k_indices_mat (&k_indices[0], 1, k);
        flann::Matrix<float> k_distances_mat (&k_distances[0], 1, k);
        // Wrap the k_indices and k_distances vectors (no data copy)

        index.knnSearch (flann::Matrix<float> (&vquery[0], 1, dim_), 
                k_indices_mat, k_distances_mat, k, flann::SearchParams(nchecks));
        for(int j=0;j<k;j++)
        {
            if(k_distances[j] < max_d_sq)
                flann_corr_ori_out<<i<<"\t"<<k_indices[j]<<endl;
        }
    }*/

    // index.knnSearch(query, indices, dists, nn, flann::SearchParams(-1));
    // double durantion = StopTiming()*1000;
    // gettimeofday(&et, 0); 
    // double durantion = (et.tv_sec - st.tv_sec)*1000 + (et.tv_usec - st.tv_usec)/1000.;
    printf("cost time: %lf ms \n", durantion);

    delete[] dataset.ptr();
    delete[] mquery.ptr();
    delete[] indices.ptr();
    delete[] dists.ptr();
}
