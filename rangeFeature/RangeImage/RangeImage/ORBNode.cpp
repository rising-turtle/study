#include "ORBNode.h"
#include "CPose3D.h"
#include "CMyTransformation.h"
#include "boost/shared_array.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "boost/dynamic_bitset.hpp"
#include <Eigen/StdVector>

#define isnan _isnan
typedef unsigned int uint;
double COrbNode::m_sMinFeatureDis=0.01; // 特征点之间最小的距离 10cm

COrbNode::COrbNode(const cv::Mat& visual,\
				   boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >m_pc):m_porb(new cv::ORB),m_flannIndex(NULL),\
				   m_pPC(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	if (!visual.data)
	{
		cout<<"error! visual is empty!"<<endl;
		assert(false);
	}
	// 保留原始点云信息
	//pc_col = *m_pc;
	m_pPC = m_pc;

	double start_time,end_time;
	if(_LOG){
		start_time=::GetTickCount();
	}

	// 计算ORB features
	(*m_porb)(visual, cv::Mat(), m_keyPoints, m_descriptors);  


	// 计算每个2D feature的3D 坐标
	projectTo3D(m_keyPoints,m_descriptors, m_feature_locations_3d_, *m_pPC/*pc_col*/); //takes less than 0.01 sec
	
	assert(m_keyPoints.size() == m_feature_locations_3d_.size());
}

COrbNode::~COrbNode(){
	delete m_porb;
	if(m_flannIndex!=NULL)
		delete m_flannIndex;
}

// LSH ? KDTree 不能直接用在ORB feature上面，貌似很复杂，OpenCV 2.3.1将会出相应的支持代码
void COrbNode::buildflannIndex()
{
	// m_flannIndex = new cv_flannIndex(m_descriptors, cv::flann::KDTreeIndexParams(4));
}
int COrbNode::findPairsFlann(const COrbNode* other, vector<cv::DMatch>* matches) const
{
	std::clock_t starttime=std::clock();
	assert(matches->size()==0);

	if (other->m_flannIndex == NULL) {
		cout<<"KDFlannTree is empty!"<<endl;	
		return -1;
	}

	// number of neighbours found (has to be two, see l. 57)
	const int k = 2;

	// compare
	// http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
	cv::Mat indices(m_descriptors.rows, k, CV_32S);
	cv::Mat dists(m_descriptors.rows, k, CV_32F);

	// get the best two neighbours
	other->m_flannIndex->knnSearch(m_descriptors, indices, dists, k,
		cv::flann::SearchParams(64));

	int* indices_ptr = indices.ptr<int> (0);
	float* dists_ptr = dists.ptr<float> (0);

	cv::DMatch match;
	for (int i = 0; i < indices.rows; ++i) {
		if (dists_ptr[2 * i] < 0.6 * dists_ptr[2 * i + 1]) {
			match.queryIdx = i;
			match.trainIdx = indices_ptr[2 * i];
			match.distance = dists_ptr[2 * i];

			assert(match.trainIdx < other->m_descriptors.rows);
			assert(match.queryIdx < m_descriptors.rows);

			matches->push_back(match);
		}
	}
	return matches->size();
}

void COrbNode::findPairsBruteForce(const COrbNode* older_node,std::vector<cv::DMatch>& matches)
{
	std::vector<cv::DMatch> tmp_matches;
	cv::BruteForceMatcher<cv::HammingLUT> matcher;  
	//matcher.match(older_node->m_descriptors, m_descriptors, tmp_matches);  
	matcher.match(m_descriptors, older_node->m_descriptors, tmp_matches);  

	double max_dist = 0; double min_dist = 1e6;  
	//-- Quick calculation of max and min distances between keypoints  
	for( int i = 0; i < m_descriptors.rows; i++ )  
	{   
		double dist = tmp_matches[i].distance;  
		if( dist < min_dist ) min_dist = dist;  
		if( dist > max_dist ) max_dist = dist;  
	}  
	//-- Draw only "good" matches (i.e. whose distance is less than 0.6*max_dist )  
	//-- PS.- radiusMatch can also be used here.  
	for( int i = 0; i < m_descriptors.rows; i++ )  
	{   
		if( tmp_matches[i].distance < 0.6*max_dist )  
		{   
			matches.push_back( tmp_matches[i]);   
		}  
	}  
}

void COrbNode::findPairsBruteForceGPU(const COrbNode* older_node,std::vector<cv::DMatch>& matches)
{
	//http://opencv-users.1802565.n2.nabble.com/how-to-use-the-ORB-descriptor-td6606424.html

	cv::gpu::GpuMat gpu_desc1(m_descriptors);			   // query desc
	cv::gpu::GpuMat gpu_desc2(older_node->m_descriptors);  // train desc
	cv::gpu::GpuMat gpu_ret_idx(m_descriptors);			   // store trainId
	cv::gpu::GpuMat gpu_ret_dist(m_descriptors);		   // store dis
	cv::gpu::GpuMat gpu_all_dist(m_descriptors*older_node->m_descriptors);  // store all dis (query, train)
	cv::Mat ret_idx, ret_dist; 

	cv::gpu::BruteForceMatcher_GPU < cv::L2<float> > gpu_matcher; 

	gpu_matcher.knnMatch(gpu_desc1, gpu_desc2, gpu_ret_idx, gpu_ret_dist, gpu_all_dist, 2); 
	gpu_ret_idx.download(ret_idx); 
	gpu_ret_dist.download(ret_dist); 

	cv::DMatch match;
	float ratio = 0.6f; // SIFT style feature matching 
	for(int i=0; i < ret_idx.rows; i++) { 
		if(ret_dist.at<float>(i,0) < ret_dist.at<float>(i,1)*ratio) { 
			// we got a match! 
			match.queryIdx=i;
			match.trainIdx=ret_idx.at<int>(i,0);
			match.distance=ret_dist.at<float>(i,0);
			
			assert(match.trainIdx < older_node->m_descriptors.rows);
			assert(match.queryIdx < m_descriptors.rows);

			matches.push_back(match);
		} 
	} 

}

// 通过匹配特征点，计算转移矩阵
MatchingResult COrbNode::matchNodePair(const COrbNode* older_node, bool Isadj)
{
	MatchingResult mr;
	const unsigned int min_matches = global_min_inliers>50?global_min_inliers:50; // 最小的特征匹配个数

	// 记录特征匹配的时间
	double start_fm;
	if(_LOG) start_fm=::GetTickCount();
	this->findPairsBruteForce(older_node, mr.all_matches); //找到匹配的feature pair
	//this->findPairsFlann(older_node,&mr.all_matches);
	//this->findPairsBruteForceGPU(older_node,mr.all_matches);
	if(_LOG) gl_pf_fm=gl_pf_fm+::GetTickCount()-start_fm;

	if (mr.all_matches.size() < min_matches)
	{
		cout<<"Matched features too little!"<<endl;
	} 
	else 
	{
		// 记录RANSAC 计算Motion的时间
		double start_me;
		bool bInliers;
		if(_LOG) start_me=::GetTickCount();
		if(Isadj)
			bInliers = getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches,Isadj);
		else
			bInliers = getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches,Isadj,500);
		if(_LOG) gl_pf_me=gl_pf_me+::GetTickCount()-start_me;
		if (bInliers)  
		{
			// + here we want to use correspond points
			vector<pcl::TMatchingPair> cors;

			// for debug
			/*ofstream f("D:\\PCL_install_on_VS2008\\match_points.txt");
			f<<"that " << "this"<<endl;*/

			for(std::vector<cv::DMatch>::iterator it = mr.inlier_matches.begin();
				it!= mr.inlier_matches.end(); it++)
			{
				Eigen::Vector4f src = m_feature_locations_3d_[(*it).queryIdx];
				Eigen::Vector4f dst = older_node->m_feature_locations_3d_[(*it).trainIdx];

				//cout<<"src:("<<src[0]<<","<<src[1]<<","<<src[2]<<"),dst:"<<dst[0]<<","<<dst[1]<<","<<dst[2]<<")"<<endl;
				//f<<src[0]<<" "<<src[1]<<" "<<src[2]<<""<<dst[0]<<" "<<dst[1]<<" "<<dst[2]<<endl;
				/*cors.push_back(
				pcl::TMatchingPair(0,0,older_node->feature_locations_3d_[(*it).trainIdx].x,older_node->feature_locations_3d_[(*it).trainIdx].y,\
				older_node->feature_locations_3d_[(*it).trainIdx].z,feature_locations_3d_[(*it).queryIdx].x,feature_locations_3d_[(*it).queryIdx].y,\
				feature_locations_3d_[(*it).queryIdx].z)
				);*/
				cors.push_back(pcl::TMatchingPair(0,0,dst[0],dst[1],dst[2],src[0],src[1],src[2]));
			}

			pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
			CPose3D finalPose;
			icp.leastSquareErrorRigidTransformation6D(cors,finalPose);
			finalPose.getHomogeneousMatrix(mr.final_trafo);
			mr.edge.id1 = older_node->m_id;//and we have a valid transformation
			mr.edge.id2 = this->m_id; //since there are enough matching features,
			mr.edge.mean = eigen2Hogman(mr.final_trafo);//we insert an edge between the frames
			mr.edge.informationMatrix =   Matrix6::eye(mr.inlier_matches.size()*mr.inlier_matches.size()); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)
	}
  }
  return mr;
}


///Find transformation with largest support, RANSAC style.
///Return false if no transformation can be found
bool COrbNode::getRelativeTransformationTo(const COrbNode* earlier_node,
    std::vector<cv::DMatch>* initial_matches,
    Eigen::Matrix4f& resulting_transformation,
    float& rmse, 
    std::vector<cv::DMatch>& matches,
	bool Isadj,
    unsigned int ransac_iterations) const
{
  assert(initial_matches != NULL);
  matches.clear();
  
  // get 30% of initial matches as inliers (but cut off at 30)
  float pct = 0.3;
  uint min_fix;
  
  // 对于连续的结点，保证其相连度，但是非连续的结点，防止其误匹配
  if(Isadj) min_fix = 10; // 原来是30;
  else min_fix = 40;
  
  uint min_inlier_threshold = int(initial_matches->size()*pct);
  min_inlier_threshold = min(min_inlier_threshold,min_fix);
  min_inlier_threshold = max(min_inlier_threshold,global_min_inliers);
    
  uint min_feature_cnt = 50;	//原来是50
  min_feature_cnt = max(min_feature_cnt, global_min_inliers);

  if(initial_matches->size() <= min_feature_cnt){ 
    return false;
  }
  std::vector<cv::DMatch> inlier; //holds those feature correspondences that support the transformation
  double inlier_error; //all squared errors
  srand((long)std::clock());
  
  // a point is an inlier if it's no more than max_dist_m m from its partner apart
  const float max_dist_m = 0.06; // 原来是 0.03;
  const unsigned int sample_size = 3;// chose this many randomly from the correspondences:
  vector<double> errors;
  vector<double> temp_errorsA;

  double best_error = 1e6;
  uint best_inlier_cnt = 0;

  int valid_iterations = 0;
  Eigen::Matrix4f transformation;

  // best values of all iterations (including invalids)
  double best_error_invalid = 1e6;
  uint best_inlier_invalid = 0;
  
  int max_matched_pair=0;

  for (uint n_iter = 0; n_iter < ransac_iterations; n_iter++) 
  {
    std::set<cv::DMatch> sample_matches;
    std::vector<cv::DMatch> sample_matches_vector;
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
    }

    bool valid;

    transformation = getTransformFromMatches(earlier_node, sample_matches.begin(), sample_matches.end(),&valid,max_dist_m);

    // valid is false iff the sampled points aren't inliers themself 
    if (!valid)
      continue;

    computeInliersAndError(*initial_matches, transformation, this->m_feature_locations_3d_, 
        earlier_node->m_feature_locations_3d_, inlier, inlier_error,  /*output*/
        temp_errorsA, max_dist_m*max_dist_m); /*output*/

	// 此处，我们过滤掉那些非常相近的点对
	if(Isadj)
		deleteSamePairs(inlier);
	
	if(inlier.size()>max_matched_pair)
		max_matched_pair=inlier.size();

    // check also invalid iterations
    if (inlier.size() > best_inlier_invalid)
    {
      best_inlier_invalid = inlier.size();
      best_error_invalid = inlier_error;
    }

    if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
      //inlier.size() < ((float)initial_matches->size())*min_inlier_ratio || 
      continue;
    }
    valid_iterations++;
    //if (inlier_error > 0) ROS_ERROR("size: %i", (int)temp_errorsA.size());
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
    double new_inlier_error;

    transformation = getTransformFromMatches(earlier_node, matches.begin(), matches.end()); // compute new trafo from all inliers:
    computeInliersAndError(*initial_matches, transformation,
        this->m_feature_locations_3d_, earlier_node->m_feature_locations_3d_,
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
      rmse = new_inlier_error;
      errors = temp_errorsA;
      best_error = new_inlier_error;
    }else
    {
    }
  } //iterations
	
  cout<<"max_matched_pairs: "<<max_matched_pair<<endl;
  cout<<"but min_inlier_matches: "<<min_inlier_threshold<<endl;
  return matches.size() >= min_inlier_threshold;
}
// 把相距小于m_sMinFeatureDis的特征点对都删掉
void COrbNode::deleteSamePairs(vector<cv::DMatch>& out_match) const
{
	vector<cv::DMatch> tmpMatches;
	for(int i=0;i<out_match.size();i++)
	{
		unsigned int queryid = out_match[i].queryIdx;
		unsigned int trainid = out_match[i].trainIdx;
		Eigen::Vector4f cur_pt = m_feature_locations_3d_[queryid];
		bool valid_match=true;
		for(int j=0;j<tmpMatches.size();j++)
		{
			unsigned int queryid1 = tmpMatches[j].queryIdx;
			unsigned int trainid1 = tmpMatches[j].trainIdx;
			if(trainid1 == trainid)  // 同样的目标匹配点
			{
				valid_match=false;
				break;
			}
			Eigen::Vector4f pre_pt = m_feature_locations_3d_[queryid1];
			double dis=0;
			for(int k=0;k<3;k++)
				dis+= (pre_pt[i]-cur_pt[i])*(pre_pt[i]-cur_pt[i]);
			double q_dis = dis;//sqrt(dis);
			if(q_dis<COrbNode::m_sMinFeatureDis)
			{
				valid_match=false;
				break;
			}
		}
		if(valid_match)
			tmpMatches.push_back(out_match[i]);
	}
	//cout<<"before delete: "<<out_match.size()<<endl;
	out_match.swap(tmpMatches);
	//cout<<"after delete: "<<out_match.size()<<endl;
}

void COrbNode::computeInliersAndError(const std::vector<cv::DMatch>& matches,
    const Eigen::Matrix4f& transformation,
    const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
    const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
    std::vector<cv::DMatch>& inliers, //output var
    double& mean_error,
    vector<double>& errors,
    double squaredMaxInlierDistInM) const{ //output var

  std::clock_t starttime=std::clock();

  inliers.clear();
  errors.clear();

  vector<pair<float,int> > dists;
  std::vector<cv::DMatch> inliers_temp;

  assert(matches.size() > 0);
  mean_error = 0.0;
  for (unsigned int j = 0; j < matches.size(); j++){ //compute new error and inliers

    unsigned int this_id = matches[j].queryIdx;
    unsigned int earlier_id = matches[j].trainIdx;

    Eigen::Vector4f vec = (transformation * origins[this_id]) - earlier[earlier_id];

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

template<class InputIterator>
Eigen::Matrix4f COrbNode::getTransformFromMatches(const COrbNode* earlier_node,
    InputIterator iter_begin,
    InputIterator iter_end,
    bool* valid, 
    float max_dist_m) const {

  //pcl::TransformationFromCorrespondences tfc;
  CMyTransformationFromCorrespondences tfc;

  vector<Eigen::Vector3f> f;
  vector<Eigen::Vector3f> t;


  for ( ;iter_begin!=iter_end; iter_begin++) {
    int this_id    = iter_begin->queryIdx;
    int earlier_id = iter_begin->trainIdx;

    Eigen::Vector3f from(this->m_feature_locations_3d_[this_id][0],
        this->m_feature_locations_3d_[this_id][1],
        this->m_feature_locations_3d_[this_id][2]);
    Eigen::Vector3f  to (earlier_node->m_feature_locations_3d_[earlier_id][0],
        earlier_node->m_feature_locations_3d_[earlier_id][1],
        earlier_node->m_feature_locations_3d_[earlier_id][2]);

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
    for (uint i=0; i<f.size(); i++)
    {
      float d_f = (f.at((i+1)%f.size())-f.at(i)).norm();
      
      min_neighbour_dist = min(d_f,min_neighbour_dist);
      
      float d_t = (t.at((i+1)%t.size())-t.at(i)).norm();

      // distances should be equal since moving and rotating preserves distances
      // 5 cm as threshold
      if ( abs(d_f-d_t) > max_dist_m || d_f<=1e-6 || d_t<=1e-6 ) //如果是相同的点计算出来的结果，也认为是无效的
      {
        // ROS_INFO("getTransformFromMatches: stop: dist is %.2f (max (%.2f))", 100* abs(d_f-d_t),100* max_dist_m);
        *valid = false;
        return foo;
      }
    }
  
    // check minimal size
    if (min_neighbour_dist < 0.5)
    {
      // ROS_INFO("getTransformFromMatches: stop: min neighbour dist is only %.0f", min_neighbour_dist*100);
    }
  
  }

  // get relative movement from samples
  return tfc.getTransformation().matrix();
}


void COrbNode::projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,cv::Mat& feature_descriptor, 
					   std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
					   const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud){

	   std::clock_t starttime=std::clock();
	   cv::Point2f p2d;

	   int total_feature=feature_descriptor.rows;
	   int n_delete=0;
	   cv::Mat valid_feature;
	   boost::dynamic_bitset<> m_valid_flag;
	   m_valid_flag.resize(total_feature,true);

	   if(feature_locations_3d.size()){
		   feature_locations_3d.clear();
	   }
	   int index=0; 
	   for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
		   p2d = feature_locations_2d[i].pt;
		   if (p2d.x >= point_cloud.width || p2d.x < 0 ||
			   p2d.y >= point_cloud.height || p2d.y < 0 ||
			   isnan(p2d.x) || isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
				   feature_locations_2d.erase(feature_locations_2d.begin()+i);
				   // 相应的特征描述符也要删掉
				   m_valid_flag[index] = false;
				   ++n_delete;
				   ++index;
				   continue;
		   }

		   pcl::PointXYZRGB p3d = point_cloud.at((int) p2d.x,(int) p2d.y);

		   if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z)){
			   feature_locations_2d.erase(feature_locations_2d.begin()+i);
			   // 相应的特征描述符也要删掉
			   m_valid_flag[index] = false;
			   ++n_delete;
			   ++index;
			   continue;
		   }

		   feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
		   i++; //Only increment if no element is removed from vector
		   index++;
	   }
	
	   //valid_feature.create(total_feature-n_delete,32,feature_descriptor.type());
	   valid_feature=feature_descriptor.clone();
	   valid_feature.resize(total_feature-n_delete);
	   int j=0;
	   for(int i=0;i<total_feature;i++)
	   {
			if(m_valid_flag[i])
			{
				memcpy(valid_feature.ptr<float>(j,0),feature_descriptor.ptr<float>(i,0),32*4);	
				j++;
			}
	   }
	feature_descriptor = valid_feature.clone();
}

// 获取特征点的3D坐标
void COrbNode::getFeatureLoc(vector<int> indices,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_out)
{
	int N_f = m_keyPoints.size();
	for(int i=0;i<indices.size();i++)
	{
		int index=indices[i];
		if(index<0 || index>=N_f){
			cout<<"error in getFeatureLoc()"<<endl;
			continue;
		}
		cv::Point2f p2d = m_keyPoints[index].pt;
		if (p2d.x >= m_pPC->width || p2d.x < 0 ||
			p2d.y >= m_pPC->height || p2d.y < 0 ||
			isnan(p2d.x) || isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
				cout<<"error in getFeatureLoc()"<<endl;
				continue;
		}
		Eigen::Vector4f p3d1=m_feature_locations_3d_[index];
		pcl::PointXYZRGB p3d; //= m_feature_locations_3d_;//m_pPC->at((int) p2d.x,(int) p2d.y);
		p3d.x=p3d1[0]; p3d.y=p3d1[1]; p3d.z=p3d1[2];
		m_out->points.push_back(p3d);
	}
}