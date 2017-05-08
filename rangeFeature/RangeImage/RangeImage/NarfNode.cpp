#include "NarfNode.h"
#include "globaldefinition.h"
#include <pcl/features/fpfh.h>
#include "pcl/features/pfh.h"
#include "boost/dynamic_bitset.hpp"
//#include "pcl/common/transformation_from_correspondences.h"
#include "CMyTransformation.h"

// Narf Parameters
float CNarfNode::narfnode_angular_resolution = pcl::deg2rad (0.5f);
float CNarfNode::narfnode_support_size = 0.2f;//0.5f;
pcl::RangeImage::CoordinateFrame CNarfNode::narfnode_coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool CNarfNode::narfnode_setUnseenToMaxRange = false;
float CNarfNode::narfnode_noise_level = 0.0;
float CNarfNode::narfnode_min_range = 0.0f;
int CNarfNode::narfnode_border_size = 1;
bool CNarfNode::narfnode_rotation_invariant=true;

// PFH Parameters
float CNarfNode::pfh_feature_radius=0.08;
float CNarfNode::pfh_voxel_grid_leaf_size = 0.01;
float CNarfNode::pfh_normal_radius=0.04;

CNarfNode::CNarfNode():m_pPC(new pcl::PointCloud<pcl::PointXYZRGB>),
m_pfhKeyPoints(new pcl::PointCloud<pcl::PointXYZ>),
m_pNormals(new pcl::PointCloud<pcl::Normal>),
m_pRangeImage(new pcl::RangeImage),
m_pFarRanges(new pcl::PointCloud<pcl::PointWithViewpoint>),
m_pNarfDest(new pcl::PointCloud<pcl::Narf36>),
m_pNarfKDFlann(new pcl::NarfKdTreeFLANN<pcl::Narf36>),
m_pPFHDest(new pcl::PointCloud<pcl::PFHSignature125>),
m_pPFHKDFlann(new pcl::KdTreeFLANN<pcl::PFHSignature125>),
m_scene_sensor_pose(Eigen::Affine3f::Identity ())
{
}
CNarfNode::~CNarfNode(){}
CNarfNode::CNarfNode(string filename):m_pPC(new pcl::PointCloud<pcl::PointXYZRGB>),
m_pfhKeyPoints(new pcl::PointCloud<pcl::PointXYZ>),
m_pNormals(new pcl::PointCloud<pcl::Normal>),
m_pRangeImage(new pcl::RangeImage),
m_pFarRanges(new pcl::PointCloud<pcl::PointWithViewpoint>),
m_pNarfDest(new pcl::PointCloud<pcl::Narf36>),
m_pNarfKDFlann(new pcl::NarfKdTreeFLANN<pcl::Narf36>),
m_pPFHDest(new pcl::PointCloud<pcl::PFHSignature125>),
m_pPFHKDFlann(new pcl::KdTreeFLANN<pcl::PFHSignature125>),
m_scene_sensor_pose(Eigen::Affine3f::Identity ())
{	
	m_ValidNode=Init(filename);
}

bool CNarfNode::Init(string filename)
{
	if (pcl::io::loadPCDFile (filename, *m_pPC) == -1)
	{
		cout<<"failed to read file: "<<filename<<endl;
		return false;
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_downpc(new  pcl::PointCloud<pcl::PointXYZRGB>);
		// Downsample the cloud
		Downsample (m_pPC, CNarfNode::pfh_voxel_grid_leaf_size, m_downpc);
		m_pPC.swap(m_downpc);

		// find (0,0,0) points
		FindZeroPoints<pcl::PointXYZRGB>(m_pPC);

		// Compute surface normals
		ComputeSurfaceNormals(m_pPC, CNarfNode::pfh_normal_radius, m_pNormals);

		// --------------------------------
		// -----Create Range Image from PC-
		// --------------------------------
		pcl::RangeImage& range_image = *m_pRangeImage; 
		pcl::PointCloud<pcl::PointXYZRGB>& point_cloud = *m_pPC;
		range_image.createFromPointCloud (point_cloud, CNarfNode::narfnode_angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),m_scene_sensor_pose,\
			CNarfNode::narfnode_coordinate_frame, CNarfNode::narfnode_noise_level, CNarfNode::narfnode_min_range, CNarfNode::narfnode_border_size);
		range_image.integrateFarRanges(*m_pFarRanges);
		if (CNarfNode::narfnode_setUnseenToMaxRange)
			m_pRangeImage->setUnseenToMaxRange ();

		// --------------------------------
		// -----Extract NARF keypoints-----
		// --------------------------------
		pcl::PointCloud<int> keypoint_indices;
		pcl::RangeImageBorderExtractor range_image_border_extractor;
		pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);

		narf_keypoint_detector.setRangeImage (&range_image);
		narf_keypoint_detector.getParameters ().support_size = CNarfNode::narfnode_support_size;
		narf_keypoint_detector.compute (keypoint_indices);

		// this will cause undesirable results 
		if(keypoint_indices.size()==0)
		{
			cout<<"No keypoints are extracted!"<<endl;
			return false;
		}

		// ------------------------------------------------------
		// -----Extract NARF descriptors for interest points-----
		// ------------------------------------------------------
		m_keypoint_indices.resize (keypoint_indices.points.size ());
		for (unsigned int i=0; i<m_keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
			m_keypoint_indices[i]=keypoint_indices.points[i];
		pcl::NarfDescriptor narf_descriptor (&range_image, &m_keypoint_indices);
		narf_descriptor.getParameters ().support_size = CNarfNode::narfnode_support_size;
		narf_descriptor.getParameters ().rotation_invariant = CNarfNode::narfnode_rotation_invariant;
		narf_descriptor.compute (*m_pNarfDest);
		cout << "Extracted "<<m_pNarfDest->size ()<<" descriptors for "
			<<keypoint_indices.points.size ()<< " keypoints.\n";
		// --------------------------------
		// -----Insert NARF into KDFLANN-----
		// --------------------------------
		m_pNarfKDFlann->setInputCloud(m_pNarfDest);

		// ------------------------------------------------------
		// -----Extract PFH descriptors for interest points-----
		// ------------------------------------------------------
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_backPC(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud (*m_pPC, *m_backPC);

		pcl::PointCloud<pcl::PointXYZ>& keypoints = *m_pfhKeyPoints;
		keypoints.points.resize (m_keypoint_indices.size ());
		for (size_t i=0; i<m_keypoint_indices.size (); ++i)
			keypoints.points[i].getVector3fMap () = range_image.points[m_keypoint_indices[i]].getVector3fMap ();

		// first do not use to calculate PFH
		CalPFHwithKeyPoints(m_backPC,m_pNormals,m_pfhKeyPoints,m_pPFHDest);

		// --------------------------------
		// -----Insert PFH into KDFLANN-----
		// --------------------------------
		m_pPFHKDFlann->setInputCloud(m_pPFHDest);
	}
	return true;
}
void CNarfNode::FindMultiPairsFlann(CNarfNode* pOldNode,vector<cv::DMatch>* out_matches,int num)
{
	assert(out_matches->size()==0);
	if(pOldNode->m_pNarfKDFlann==NULL)
	{
		cout<<"older_node has no PFH kd_tree!"<<endl;
		return;
	}
	const int k = num;
	std::vector<int> k_indices (k);
	std::vector<float> k_squared_distances (k);
	cv::DMatch match;
	for (size_t i = 0; i < m_pNarfDest->points.size(); ++i)
	{
		pOldNode->m_pNarfKDFlann->nearestKSearch(*(this->m_pNarfDest),i,k,k_indices,k_squared_distances);
		match.queryIdx=i;
		for(int j=0;j<k;j++)
		{
			match.trainIdx=k_indices[j];
			match.distance=k_squared_distances[j];
			if(match.trainIdx<pOldNode->m_pNarfDest->size())
			{
				out_matches->push_back(match);
			}
		}
	}
}
void CNarfNode::DeleteMultiMatches(vector<cv::DMatch>* out_matches)
{
	if(out_matches->size()<3) return;
	boost::dynamic_bitset<> valid_match;
	valid_match.resize(out_matches->size(),1);
	static const int noisy_match_threshold=2;

	// find invalid matches
	vector<int> noisy_match_index;
	vector<cv::DMatch>& match=*out_matches;
	for(size_t i=0;i<match.size();i++)
	{
		if(!valid_match[i]) continue;
		noisy_match_index.push_back(i);
		int same_trainIdx=0;
		for(size_t j=i+1;j<match.size();j++)
		{
			if(match[i].trainIdx==match[j].trainIdx)
			{
				same_trainIdx++;
				noisy_match_index.push_back(j);
			}
		}
		if(same_trainIdx >= noisy_match_threshold)
		{
			for(size_t k=0;k<noisy_match_index.size();k++)
				valid_match[noisy_match_index[k]]=false;
		}
		noisy_match_index.clear();
	}

	// delete those invalid matches
	vector<cv::DMatch> match_back;
	for(size_t i=0;i<match.size();i++)
	{
		if(valid_match[i])
			match_back.push_back(match[i]);
	}
	match.swap(match_back);
}
void CNarfNode::FindPairsFlann2(CNarfNode* pOldNode,vector<cv::DMatch>* out_matches,int num)
{
	assert(out_matches->size()==0);
	const int k = num;
	// kd search for narf descriptors
	std::vector<int> k_indices_narf (k);
	std::vector<float> k_squared_distances_narf (k);
	// kd search for pfh descriptors
	std::vector<int> k_indices_pfh (k);
	std::vector<float> k_squared_distances_pfh (k);
	cv::DMatch match;
	if(pOldNode->m_pNarfKDFlann==NULL || pOldNode->m_pPFHKDFlann==NULL)
	{
		cout<<"older_node has no PFH or NARF kd_tree!"<<endl;
		return;
	}
	pcl::PointXYZ spKeyPoint;
	int index_of_pfh_keypoint;
	for(size_t i=0;i<m_pNarfDest->points.size();i++)
	{
		pOldNode->m_pNarfKDFlann->nearestKSearch(*(this->m_pNarfDest),i,k,k_indices_narf,k_squared_distances_narf);
		// find corresponding pfh point
		index_of_pfh_keypoint=-1;
		pcl::Narf36& spNarf=m_pNarfDest->points[i];
		spKeyPoint.x=spNarf.x; spKeyPoint.y=spNarf.y; spKeyPoint.z=spNarf.z;
		for(size_t j=0;j<m_pfhKeyPoints->points.size();j++)
		{
			if(IsSamePoint<pcl::PointXYZ>(spKeyPoint,m_pfhKeyPoints->points[j]))
			{
				index_of_pfh_keypoint=j;
				break;
			}
		}
		if(index_of_pfh_keypoint!=-1) // find responding pfh point
		{
			pOldNode->m_pPFHKDFlann->nearestKSearch(*(this->m_pPFHDest),index_of_pfh_keypoint,k,k_indices_pfh,k_squared_distances_pfh);
			// find the same matched point 
			for(int id1=0;id1<k;id1++)
			{
				pcl::Narf36& Narf1 = pOldNode->m_pNarfDest->points[k_indices_narf[id1]];
				spKeyPoint.x=Narf1.x; spKeyPoint.y=Narf1.y; spKeyPoint.z=Narf1.z;
				for(int id2=0;id2<k;id2++)
				{
					pcl::PointXYZ& Pfh1 = pOldNode->m_pfhKeyPoints->points[k_indices_pfh[id2]];
					if(IsSamePoint(Pfh1,spKeyPoint))// record this match
					{
						match.queryIdx=index_of_pfh_keypoint;
						match.trainIdx=k_indices_pfh[id2];
						match.distance=k_squared_distances_pfh[id2];
						out_matches->push_back(match);
						id1=k; // only find the first match
						break;
					}
				}
			}
		}
	}


}
void CNarfNode::FindPairsFlann(CNarfNode* pOldNode,vector<cv::DMatch>* out_matches,bool useNarf,int _k)
{
	assert(out_matches->size()==0);
	// Find feature correspondences
	//std::vector<int> correspondences;
	//std::vector<float> correspondence_scores;
	//find_feature_correspondences (descriptors1, descriptors2, correspondences, correspondence_scores);
	// Find the index of the best match for each keypoint, and store it in "correspondences_out"
	const int k = _k;
	std::vector<int> k_indices (k);
	std::vector<float> k_squared_distances (k);
	cv::DMatch match;
	if(useNarf) // use Narf to match keypoints
	{
		if(pOldNode->m_pNarfKDFlann==NULL)
		{
			cout<<"older_node has no PFH kd_tree!"<<endl;
			return;
		}
		int index_kpt;
		for (size_t i = 0; i < m_pNarfDest->points.size(); ++i)
		{
			pOldNode->m_pNarfKDFlann->nearestKSearch(*(this->m_pNarfDest),i,k,k_indices,k_squared_distances);
			index_kpt=findNarfKeyPoint(m_pNarfDest->points[i]);
			if(index_kpt==-1)
			{
				cout<<"No keypoint is matched with Narf pt!"<<endl;
				continue;
			}
			match.queryIdx=index_kpt;
		//	match.trainIdx=k_indices[0];

		//	assert(match.queryIdx<this->m_pNarfDest->size());
		//	assert(match.trainIdx<pOldNode->m_pNarfDest->size());

			for(int l=0;l<k;l++)
			{
				index_kpt=pOldNode->findNarfKeyPoint(pOldNode->m_pNarfDest->points[k_indices[l]]);
				if(index_kpt==-1)
				{
					cout<<"No keypoint is matched with Narf pt!"<<endl;
					continue;
				}
				match.trainIdx=index_kpt;//k_indices[l];
				match.distance=k_squared_distances[l];
				out_matches->push_back(match);
			}

			//if(k_squared_distances[0]<0.6*k_squared_distances[1])
			//{
			//	match.distance=k_squared_distances[0];
			//	out_matches->push_back(match);
			//}
		}
	}
	else
	{
		if(pOldNode->m_pPFHDest==NULL)
		{
			cout<<"older_node has no PFH kd_tree!"<<endl;
			return;
		}
		for (size_t i = 0; i < m_pPFHDest->points.size(); ++i)
		{
			pOldNode->m_pPFHKDFlann->nearestKSearch(*(this->m_pPFHDest),i,k,k_indices,k_squared_distances);
			match.queryIdx=i;
			//match.trainIdx=k_indices[0];

			for(int l=0;l<k;l++)
			{
				match.trainIdx=k_indices[l];
				match.distance=k_squared_distances[l];
				out_matches->push_back(match);
			}

			//assert(match.queryIdx<this->m_pPFHDest->size());
			//assert(match.trainIdx<pOldNode->m_pPFHDest->size());

			/*if(k_squared_distances[0]<0.6*k_squared_distances[1])
			{
				match.distance=k_squared_distances[0];
				out_matches->push_back(match);
			}*/
		}
	}
}
MatchingResult CNarfNode::MatchNodePair(CNarfNode* older_node)
{
	MatchingResult mr;

	// get matched pairs
	this->FindPairsFlann(older_node, &mr.all_matches); 

	vector<pcl::TMatchingPair> cors;
	for(std::vector<cv::DMatch>::iterator it = mr.all_matches.begin();
		it!= mr.all_matches.end(); it++)
	{
		pcl::Narf36& sp_src=m_pNarfDest->points[(*it).queryIdx];
		pcl::Narf36& sp_tar=older_node->m_pNarfDest->points[(*it).trainIdx];
		/*Eigen::Vector4f src = m_keypt->points[(*it).queryIdx];
		Eigen::Vector4f dst = older_node->feature_locations_3d_[(*it).trainIdx];*/
		cors.push_back(pcl::TMatchingPair(0,0,sp_src.x,sp_src.y,sp_src.z,sp_tar.x,sp_tar.y,sp_tar.z));

		//f<<src[0]<<" "<<src[1]<<" "<<src[2]<<""<<dst[0]<<" "<<dst[1]<<" "<<dst[2]<<endl;
		/*cors.push_back(pcl::TMatchingPair(0,0,older_node->feature_locations_3d_[(*it).trainIdx].x,older_node->feature_locations_3d_[(*it).trainIdx].y,\
		older_node->feature_locations_3d_[(*it).trainIdx].z,feature_locations_3d_[(*it).queryIdx].x,feature_locations_3d_[(*it).queryIdx].y,\
		feature_locations_3d_[(*it).queryIdx].z));*/
		//cors.push_back(pcl::TMatchingPair(0,0,dst[0],dst[1],dst[2],src[0],src[1],src[2]));

	}
	pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
	CPose3D finalPose;
	icp.leastSquareErrorRigidTransformation6D(cors,finalPose);

	//mr.final_trafo = mr.ransac_trafo;
	finalPose.getHomogeneousMatrix(mr.final_trafo);
	mr.edge.id1 = older_node->m_id;//and we have a valid transformation
	mr.edge.id2 = this->m_id; //since there are enough matching features,
	mr.edge.mean = eigen2Hogman(mr.final_trafo);//we insert an edge between the frames
	mr.edge.informationMatrix =   Matrix6::eye(mr.inlier_matches.size()*mr.inlier_matches.size()); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)

	return mr;
}
void CNarfNode::CalPFHwithKeyPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &points, pcl::PointCloud<pcl::Normal>::Ptr &normals, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoints,pcl::PointCloud<pcl::PFHSignature125>::Ptr& descriptors_out)
{
	// Create a PFHEstimation object
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_est;

	// Set it to use a FLANN-based KdTree to perform its neighborhood searches
	pfh_est.setSearchMethod (pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr (new pcl::KdTreeFLANN<pcl::PointXYZ>));

	// Specify the radius of the PFH feature
	pfh_est.setRadiusSearch (CNarfNode::pfh_feature_radius);

	// Use all of the points for analyzing the local structure of the cloud
	pfh_est.setSearchSurface (points);  
	pfh_est.setInputNormals (normals);  

	// But only compute features at the keypoints
	pfh_est.setInputCloud (keypoints);

	// Compute the features
	pfh_est.compute (*descriptors_out);

}

//Find transformation with largest support, RANSAC style.
///Return false if no transformation can be found
bool CNarfNode::getRelativeTransformationTo(CNarfNode* earlier_node,
    std::vector<cv::DMatch>* initial_matches,
    Eigen::Matrix4f& resulting_transformation,
    float& rmse, 
    std::vector<cv::DMatch>& matches,
    unsigned int ransac_iterations){

  assert(initial_matches != NULL);
  matches.clear();
  
  // get 30% of initial matches as inliers (but cut off at 30)
  float pct = 0.2;	// at about 1/4 points are matched
  unsigned int min_fix = 3; // at least 5 points are enough to calculate transformation
  
  unsigned int min_inlier_threshold = int(initial_matches->size()*pct);
  min_inlier_threshold = max(min_inlier_threshold,min_fix);
 // min_inlier_threshold = max(min_inlier_threshold,min_fix);
 // min_inlier_threshold = max(min_inlier_threshold,global_min_inliers);
    
  unsigned int min_feature_cnt = 5;
 // min_feature_cnt = max(min_feature_cnt, global_min_inliers);

  if(initial_matches->size() <= min_feature_cnt){ 
	  cout<<"Geometry features are not enough !"<<endl;
	  return false;
  }
  std::vector<cv::DMatch> inlier; //holds those feature correspondences that support the transformation
  double inlier_error; //all squared errors
  srand((long)std::clock());
  
  // a point is an inlier if it's no more than max_dist_m m from its partner apart
  const float max_dist_m = 0.03;
  const unsigned int sample_size = 3;// chose this many randomly from the correspondences:
  vector<double> errors;
  vector<double> temp_errorsA;

  double best_error = 1e6;
  unsigned int best_inlier_cnt = 0;

  int valid_iterations = 0;
  Eigen::Matrix4f transformation;

  // best values of all iterations (including invalids)
  double best_error_invalid = 1e6;
  unsigned int best_inlier_invalid = 0;

  for (unsigned int n_iter = 0; n_iter < ransac_iterations; n_iter++) {
    std::set<cv::DMatch> sample_matches;
    std::vector<cv::DMatch> sample_matches_vector;
	set<int> sample_id;
	double incresement=0.001;
    while(sample_matches.size() < sample_size){
      int id = rand() % initial_matches->size();
	  // to deal with case: all dis = 0
	  if(initial_matches->at(id).distance < 1e-6)
	  {
		  initial_matches->at(id).distance += incresement; 
		  incresement+=0.001;
	  }
      sample_matches.insert(initial_matches->at(id));
      sample_matches_vector.push_back(initial_matches->at(id));
	  sample_id.insert(id);
    }
	for(set<int>::iterator it=sample_id.begin();it!=sample_id.end();it++){
		cout<<*it<<" ";
	}

    bool valid;

    transformation = getTransformFromMatches(earlier_node, sample_matches.begin(), sample_matches.end(),&valid,max_dist_m);

    // valid is false iff the sampled points aren't inliers themself 
    if (!valid)
      continue;

    computeInliersAndError(*initial_matches, transformation, this->m_pfhKeyPoints->points, 
        earlier_node->m_pfhKeyPoints->points, inlier, inlier_error,  /*output*/
        temp_errorsA, max_dist_m*max_dist_m); /*output*/

    // check also invalid iterations
    if (inlier.size() > best_inlier_invalid)
    {
      best_inlier_invalid = inlier.size();
      best_error_invalid = inlier_error;
    }

    if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
      continue;
    }
    valid_iterations++;
    assert(inlier_error>0);

    //Performance hacks:
    ///Iterations with more than half of the initial_matches inlying, count twice
    if (inlier.size() > initial_matches->size()*0.5) n_iter++;
    ///Iterations with more than 80% of the initial_matches inlying, count threefold
    if (inlier.size() > initial_matches->size()*0.8) n_iter++;

    if (inlier_error < best_error) { //copy this to the result
      resulting_transformation = transformation;
      matches = inlier;
      assert(matches.size()>= min_inlier_threshold);
      best_inlier_cnt = inlier.size();
      //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
      rmse = inlier_error;
      errors = temp_errorsA;
      best_error = inlier_error;
    }else
    {
    }

    //int max_ndx = min((int) min_inlier_threshold,30); //? What is this 30?
    double new_inlier_error;

    transformation = getTransformFromMatches(earlier_node, matches.begin(), matches.end()); // compute new trafo from all inliers:
    computeInliersAndError(*initial_matches, transformation,
        this->m_pfhKeyPoints->points, earlier_node->m_pfhKeyPoints->points,
        inlier, new_inlier_error, temp_errorsA, max_dist_m*max_dist_m);

    // check also invalid iterations
    if (inlier.size() > best_inlier_invalid)
    {
      best_inlier_invalid = inlier.size();
      best_error_invalid = inlier_error;
    }

    if(inlier.size() < min_inlier_threshold || new_inlier_error > max_dist_m){
      continue;
    }

    assert(new_inlier_error>0);

    if (new_inlier_error < best_error) 
    {
      resulting_transformation = transformation;
      matches = inlier;
      assert(matches.size()>= min_inlier_threshold);
      //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
      rmse = new_inlier_error;
      errors = temp_errorsA;
      best_error = new_inlier_error;
    }else
    {
    }
  } //iterations

  if(matches.size()<min_inlier_threshold)
  {
	  cout<<"matched.size()="<<matches.size()<<" < min_inlier_threshold:"<<min_inlier_threshold<<endl;
	  return false;
  }
  return matches.size() >= min_inlier_threshold;
}

void CNarfNode::computeInliersAndError( std::vector<cv::DMatch>& matches,
    Eigen::Matrix4f& transformation,
	vector<pcl::PointXYZ,Eigen::aligned_allocator<pcl::PointXYZ> >& origins,
	vector<pcl::PointXYZ,Eigen::aligned_allocator<pcl::PointXYZ> >& earlier,
    std::vector<cv::DMatch>& inliers, //output var
    double& mean_error,
    vector<double>& errors,
    double squaredMaxInlierDistInM) { //output var

	inliers.clear();
	errors.clear();

	vector<pair<float,int> > dists;
	std::vector<cv::DMatch> inliers_temp;

	assert(matches.size() > 0);
	mean_error = 0.0;
	float ort[4],ert[4];
  for (unsigned int j = 0; j < matches.size(); j++){ //compute new error and inliers

    unsigned int this_id = matches[j].queryIdx;
    unsigned int earlier_id = matches[j].trainIdx;

	pcl::PointXYZ& cursp=origins[this_id];
	pcl::PointXYZ& presp=earlier[earlier_id];
	ort[0]=cursp.x; ort[1]=cursp.y; ort[2]=cursp.z; ort[3]=1.0;
	ert[0]=presp.x; ert[1]=presp.y; ert[2]=presp.z; ert[3]=1.0;
	Eigen::Map<Eigen::Vector4f> thispt=Eigen::Vector4f::Map(ort);
	Eigen::Map<Eigen::Vector4f> thatpt=Eigen::Vector4f::Map(ert);

    Eigen::Vector4f vec = (transformation * thispt) - thatpt;

    double error = vec.dot(vec);

    if(error > squaredMaxInlierDistInM)
      continue; //ignore outliers


    error = sqrt(error);
    dists.push_back(pair<float,int>(error,j));
    inliers_temp.push_back(matches[j]); //include inlier

    mean_error += error;
    errors.push_back(error);
  }

  if (inliers_temp.size()==0){
    mean_error = -1;
    inliers = inliers_temp;
  }
  else
  {
    mean_error /= inliers_temp.size();

    // sort inlier ascending according to their error
    sort(dists.begin(),dists.end());

    inliers.resize(inliers_temp.size());
    for (unsigned int i=0; i<inliers_temp.size(); i++){
      inliers[i] = matches[dists[i].second];
    }
  }
}

void CNarfNode::Downsample (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float leaf_size,
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr &downsampled_out)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
	vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
	vox_grid.setInputCloud (points);
	vox_grid.filter (*downsampled_out);
}

void CNarfNode::ComputeSurfaceNormals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float normal_radius,
										 pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;

	// Use a FLANN-based KdTree to perform neighborhood searches
	norm_est.setSearchMethod (pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr (new pcl::KdTreeFLANN<pcl::PointXYZRGB>));

	// Specify the size of the local neighborhood to use when computing the surface normals
	norm_est.setRadiusSearch (normal_radius);

	// Set the input points
	norm_est.setInputCloud (points);

	// Estimate the surface normals and store the result in "normals_out"
	norm_est.compute (*normals_out);
}

template<class InputIterator>
Eigen::Matrix4f CNarfNode::getTransformFromMatches(const CNarfNode* earlier_node,
    InputIterator iter_begin,
    InputIterator iter_end,
    bool* valid, 
    float max_dist_m){

  //pcl::TransformationFromCorrespondences tfc;
  CMyTransformationFromCorrespondences tfc;

  vector<Eigen::Vector3f> f;
  vector<Eigen::Vector3f> t;


  for ( ;iter_begin!=iter_end; iter_begin++) {
    int this_id    = iter_begin->queryIdx;
    int earlier_id = iter_begin->trainIdx;

    Eigen::Vector3f from(this->m_pfhKeyPoints->points[this_id].x,
        this->m_pfhKeyPoints->points[this_id].y,
        this->m_pfhKeyPoints->points[this_id].z);
    Eigen::Vector3f  to (earlier_node->m_pfhKeyPoints->points[earlier_id].x,
        earlier_node->m_pfhKeyPoints->points[earlier_id].y,
        earlier_node->m_pfhKeyPoints->points[earlier_id].z);

    if (max_dist_m > 0)
    {
      f.push_back(from);
      t.push_back(to);    
    }

    tfc.add(from, to);
  }
  // find smalles distance between a point and its neighbour in the same cloud
  // je groesser das dreieck aufgespannt ist, desto weniger fallen kleine positionsfehler der einzelnen
  // Punkte ist Gewicht!

  if (max_dist_m > 0)
  {  
	  float min_neighbour_dist = 1e6;
	  Eigen::Matrix4f foo;

	  *valid = true;
	  for (unsigned int i=0; i<f.size(); i++)
	  {
		  float d_f = (f.at((i+1)%f.size())-f.at(i)).norm();

		  min_neighbour_dist = min(d_f,min_neighbour_dist);

		  float d_t = (t.at((i+1)%t.size())-t.at(i)).norm();

		  // distances should be equal since moving and rotating preserves distances
		  // 5 cm as threshold
		  if ( abs(d_f-d_t) > max_dist_m )
		  {
			  *valid = false;
			  return foo;
		  }
	  }

	  // check minimal size
	  if (min_neighbour_dist < 0.5)
	  {
	  }

  }

  // get relative movement from samples
  return tfc.getTransformation().matrix();
}

int CNarfNode::findNarfKeyPoint(pcl::Narf36& narfpt)
{
	pcl::PointXYZ pt;
	pt.x=narfpt.x; pt.y=narfpt.y; pt.z=narfpt.z;
	for(size_t i=0;i<m_pfhKeyPoints->points.size();i++)
	{
		pcl::PointXYZ& pt1=m_pfhKeyPoints->points[i];
		if(IsSamePoint(pt,pt1))
			return i;
	}
	return -1;
}