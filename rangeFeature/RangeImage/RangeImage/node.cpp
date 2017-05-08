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


#include "node.h"
#include <cmath>
#include <ctime>
#include <Eigen/Geometry>
#include "pcl/ros/conversions.h"
#include "pcl/point_types.h"
//#include <pcl/common/transformation_from_correspondences.h>
#include "CMyTransformation.h"
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <float.h>
//#include <fstream>

//#include <iostream>
#include <Eigen/StdVector>
#define isnan _isnan
typedef unsigned int uint;

Transformation3 eigen2Hogman(const Eigen::Matrix4f& eigen_mat) {
  std::clock_t starttime=std::clock();

  Eigen::Affine3f eigen_transform(eigen_mat);
  Eigen::Quaternionf eigen_quat(eigen_transform.rotation());
  Vector3 translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
  Quaternion rotation(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(),
      eigen_quat.w());
  Transformation3 result(translation, rotation);

  return result;
}

Node::Node(const cv::Mat& visual,
    cv::Ptr<cv::FeatureDetector> detector,
    cv::Ptr<cv::DescriptorExtractor> extractor,
    cv::Ptr<cv::DescriptorMatcher> matcher,
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud,
    const cv::Mat& detection_mask)
: id_(0), 
flannIndex(NULL),
matcher_(matcher),
m_pPC(new pcl::PointCloud<pcl::PointXYZRGB>)
{

  detector->detect( visual, feature_locations_2d_, detection_mask);// fill 2d locations

  // 保留原始点云信息
  //pc_col = *point_cloud;
  //m_pPC = point_cloud;

  // project pixels to 3dPositions and create search structures for the gicp
  projectTo3D(feature_locations_2d_, feature_locations_3d_, *point_cloud/*pc_col*/); //takes less than 0.01 sec

  extractor->compute(visual, feature_locations_2d_, feature_descriptors_); //fill feature_descriptors_ with information 
  // extractor may erase feature_locations_2d_!!!
  if(feature_locations_2d_.size() != feature_locations_3d_.size())
	  projectTo3D(feature_locations_2d_, feature_locations_3d_, *point_cloud/*pc_col*/);

  assert(feature_locations_2d_.size() == feature_locations_3d_.size());
}

Node::~Node(){
  if(flannIndex)
    delete flannIndex;
}


// build search structure for descriptor matching
void Node::buildFlannIndex() {
  //std::clock_t starttime=std::clock();
  // use same type as in http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
  flannIndex = new cv_flannIndex(feature_descriptors_, cv::flann::KDTreeIndexParams(4));
//  ROS_DEBUG("Built flannIndex (address %p) for Node %i", flannIndex, this->id_);
  // ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.001, "timings", "buildFlannIndex runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}


//TODO: This function seems to be resistant to paralellization probably due to knnSearch
int Node::findPairsFlann(const Node* other, vector<cv::DMatch>* matches) const {
  std::clock_t starttime=std::clock();
  assert(matches->size()==0);

  if (other->flannIndex == NULL) {
//    ROS_FATAL("Node %i in findPairsFlann: flann Index of Node %i was not initialized", this->id_, other->id_);
    return -1;
  }

  // number of neighbours found (has to be two, see l. 57)
  const int k = 2;

  // compare
  // http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
  cv::Mat indices(feature_descriptors_.rows, k, CV_32S);
  cv::Mat dists(feature_descriptors_.rows, k, CV_32F);

  //ROS_INFO("find flann pairs: feature_descriptor (rows): %i", feature_descriptors_.rows);

  // get the best two neighbours
  other->flannIndex->knnSearch(feature_descriptors_, indices, dists, k,
      cv::flann::SearchParams(64));

  int* indices_ptr = indices.ptr<int> (0);
  float* dists_ptr = dists.ptr<float> (0);

  cv::DMatch match;
  for (int i = 0; i < indices.rows; ++i) {
    if (dists_ptr[2 * i] < 0.6 * dists_ptr[2 * i + 1]) {
      match.queryIdx = i;
      match.trainIdx = indices_ptr[2 * i];
      match.distance = dists_ptr[2 * i];

      assert(match.trainIdx < other->feature_descriptors_.rows);
      assert(match.queryIdx < feature_descriptors_.rows);

      matches->push_back(match);
    }
  }
  return matches->size();
}



void Node::projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
    const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud){

  std::clock_t starttime=std::clock();

  cv::Point2f p2d;

  if(feature_locations_3d.size()){
//    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    p2d = feature_locations_2d[i].pt;
    if (p2d.x >= point_cloud.width || p2d.x < 0 ||
        p2d.y >= point_cloud.height || p2d.y < 0 ||
        isnan(p2d.x) || isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
//      ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

	pcl::PointXYZRGB p3d = point_cloud.at((int) p2d.x,(int) p2d.y);

    //ROS_INFO("3d: %f, %f, %f, 2d: %f, %f", p3d.x, p3d.y, p3d.z, p2d.x, p2d.y);

    if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z)){
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
    i++; //Only increment if no element is removed from vector
  }



  //ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}



void Node::computeInliersAndError(const std::vector<cv::DMatch>& matches,
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


  //ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");

}
/*
template<class InputIterator, class T>
  InputIterator find ( InputIterator first, InputIterator last, const T& value )
  {
    for ( ;first!=last; first++) if ( *first==value ) break;
    return first;
  }
 */
template<class InputIterator>
Eigen::Matrix4f Node::getTransformFromMatches(const Node* earlier_node,
    InputIterator iter_begin,
    InputIterator iter_end,
    bool* valid, 
    float max_dist_m) const {

  //pcl::TransformationFromCorrespondences tfc;
  CMyTransformationFromCorrespondences tfc;

  vector<Eigen::Vector3f> f;
  vector<Eigen::Vector3f> t;

  // 此处只是为了检验：
  //		当只用少量的匹配点(10)计算旋转矩阵时，得到的结果会偏差多少
  //int num=0;

  for ( ;iter_begin!=iter_end; iter_begin++) {
    int this_id    = iter_begin->queryIdx;
    int earlier_id = iter_begin->trainIdx;

	//if(num++>10) break;

    Eigen::Vector3f from(this->feature_locations_3d_[this_id][0],
        this->feature_locations_3d_[this_id][1],
        this->feature_locations_3d_[this_id][2]);
    Eigen::Vector3f  to (earlier_node->feature_locations_3d_[earlier_id][0],
        earlier_node->feature_locations_3d_[earlier_id][1],
        earlier_node->feature_locations_3d_[earlier_id][2]);

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
      if ( abs(d_f-d_t) > max_dist_m )
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


///Find transformation with largest support, RANSAC style.
///Return false if no transformation can be found
bool Node::getRelativeTransformationTo(const Node* earlier_node,
    std::vector<cv::DMatch>* initial_matches,
    Eigen::Matrix4f& resulting_transformation,
    float& rmse, 
    std::vector<cv::DMatch>& matches,
    //float min_inlier_ratio,
    unsigned int ransac_iterations) const{
  //ROS_INFO("unsigned int min_inlier_threshold %i", min_inlier_threshold);
  // std::clock_t starttime=std::clock();

  assert(initial_matches != NULL);

  // ROS_INFO("inlier_threshold: %d", min_inlier_threshold);


  matches.clear();
  
  // get 30% of initial matches as inliers (but cut off at 30)
  float pct = 0.2;
  uint min_fix = 30;
  
  uint min_inlier_threshold = int(initial_matches->size()*pct);
  min_inlier_threshold = max(min_inlier_threshold,min_fix);
 //min_inlier_threshold = max(min_inlier_threshold,global_min_inliers);
    
  uint min_feature_cnt = 50;
  min_feature_cnt = max(min_feature_cnt, global_min_inliers);


  if(initial_matches->size() <= min_feature_cnt){ 
//    ROS_INFO("Only %d feature matches between %d and %d (minimal: %i)",(int)initial_matches->size() , this->id_, earlier_node->id_, min_feature_cnt);
    return false;
  }
  std::vector<cv::DMatch> inlier; //holds those feature correspondences that support the transformation
  double inlier_error; //all squared errors
  srand((long)std::clock());
  
  // a point is an inlier if it's no more than max_dist_m m from its partner apart
  const float max_dist_m = 0.04;//原来是 0.03
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

  
  
  //ROS_INFO("running %i iterations with %i initial matches, min_match: %i, max_error: %.2f", (int) ransac_iterations, (int) initial_matches->size(), (int) min_inlier_threshold, max_dist_m*100 );

  for (uint n_iter = 0; n_iter < ransac_iterations; n_iter++) {
    //generate a map of samples. Using a map solves the problem of drawing a sample more than once

    // ROS_INFO("iteration %d of %d", n_iter,ransac_iterations);

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

    computeInliersAndError(*initial_matches, transformation, this->feature_locations_3d_, 
        earlier_node->feature_locations_3d_, inlier, inlier_error,  /*output*/
        temp_errorsA, max_dist_m*max_dist_m); /*output*/

    // check also invalid iterations
    if (inlier.size() > best_inlier_invalid)
    {
      best_inlier_invalid = inlier.size();
      best_error_invalid = inlier_error;
    }

    // ROS_INFO("iteration %d  cnt: %d, best: %d,  error: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100);

    if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
      //inlier.size() < ((float)initial_matches->size())*min_inlier_ratio || 
      // ROS_INFO("Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
      continue;
    }
    // ROS_INFO("Refining iteration from %i samples: all matches: %i, inliers: %i, inlier_error: %f", (int)sample_size, (int)initial_matches->size(), (int)inlier.size(), inlier_error);
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
      // ROS_INFO("  new best iteration %d  cnt: %d, best_inlier: %d,  error: %.4f, bestError: %.4f",n_iter, inlier.size(), best_inlier_cnt, inlier_error, best_error);

    }else
    {
      // ROS_INFO("NO new best iteration %d  cnt: %d, best_inlier: %d,  error: %.4f, bestError: %.4f",n_iter, inlier.size(), best_inlier_cnt, inlier_error, best_error);
    }

    //int max_ndx = min((int) min_inlier_threshold,30); //? What is this 30?
    double new_inlier_error;

    transformation = getTransformFromMatches(earlier_node, matches.begin(), matches.end()); // compute new trafo from all inliers:
    computeInliersAndError(*initial_matches, transformation,
        this->feature_locations_3d_, earlier_node->feature_locations_3d_,
        inlier, new_inlier_error, temp_errorsA, max_dist_m*max_dist_m);

    // ROS_INFO("asd recomputed: inliersize: %i, inlier error: %f", (int) inlier.size(),100*new_inlier_error);


    // check also invalid iterations
    if (inlier.size() > best_inlier_invalid)
    {
      best_inlier_invalid = inlier.size();
      best_error_invalid = inlier_error;
    }

    if(inlier.size() < min_inlier_threshold || new_inlier_error > max_dist_m){
      //inlier.size() < ((float)initial_matches->size())*min_inlier_ratio || 
      // ROS_INFO("Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
      continue;
    }
    // ROS_INFO("Refined iteration from %i samples: all matches %i, inliers: %i, new_inlier_error: %f", (int)sample_size, (int)initial_matches->size(), (int)inlier.size(), new_inlier_error);

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
      // ROS_INFO("  improved: new best iteration %d  cnt: %d, best_inlier: %d,  error: %.2f, bestError: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100, best_error*100);
    }else
    {
      // ROS_INFO("improved: NO new best iteration %d  cnt: %d, best_inlier: %d,  error: %.2f, bestError: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100, best_error*100);
    }
  } //iterations
  //ROS_INFO("%i good iterations (from %i), inlier pct %i, inlier cnt: %i, error: %.2f cm",valid_iterations, (int) ransac_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size(),rmse*100);
  // ROS_INFO("best overall: inlier: %i, error: %.2f",best_inlier_invalid, best_error_invalid*100);


  // ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "getRelativeTransformationTo runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");

return matches.size() >= min_inlier_threshold;
}


//TODO: Merge this with processNodePair
MatchingResult Node::matchNodePair(const Node* older_node){
  MatchingResult mr;
  const unsigned int min_matches = global_min_inliers; // minimal number of feature correspondences to be a valid candidate for a link
  // std::clock_t starttime=std::clock();

  // 记录特征匹配所需用的时间
  double start_fm;
  if(_LOG) start_fm=::GetTickCount();
  this->findPairsFlann(older_node, &mr.all_matches);
  if(_LOG) gl_pf_fm=gl_pf_fm+::GetTickCount()-start_fm;

 /* for(std::vector<cv::DMatch>::iterator it = mr.all_matches.begin();it!=mr.all_matches.end();it++)
  {
	  int this_id    = it->queryIdx;
	  int earlier_id = it->trainIdx;

	  if(this_id >= this->feature_locations_3d_.size() || earlier_id >= older_node->feature_locations_3d_.size())
	  {
		cout<<"What's wrong with this!!!"<<endl;
	  }
  }*/

  //ROS_DEBUG("found %i inital matches",(int) mr.all_matches.size());
  if (mr.all_matches.size() < min_matches)
  {
    //ROS_INFO("Too few inliers: Adding no Edge between %i and %i. Only %i correspondences to begin with.",
   //     older_node->id_,this->id_,(int)mr.all_matches.size());
  } 
  else 
  {
	  // 记录利用RANSAC估计Motion的时间
	  double start_me;
	  if(_LOG) start_me=::GetTickCount();
	  bool bInliers = getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches);
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
			Eigen::Vector4f src = feature_locations_3d_[(*it).queryIdx];
			Eigen::Vector4f dst = older_node->feature_locations_3d_[(*it).trainIdx];

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
		//mr.final_trafo = mr.ransac_trafo;
		finalPose.getHomogeneousMatrix(mr.final_trafo);
		mr.edge.id1 = older_node->id_;//and we have a valid transformation
		mr.edge.id2 = this->id_; //since there are enough matching features,
        mr.edge.mean = eigen2Hogman(mr.final_trafo);//we insert an edge between the frames
        mr.edge.informationMatrix =   Matrix6::eye(mr.inlier_matches.size()*mr.inlier_matches.size()); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)
    }
  }
  // Paper
  // ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "processNodePair runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
  return mr;
}

///Compare the features of adjacent node current and last node
MatchingResult Node::matchLastNode(const Node* older_node, CPose3D& last_pose, CPose3D& curr_pose)
{
	MatchingResult mr;
	const unsigned int min_matches = 32; // minimal number of feature correspondences to be a valid candidate for a link
	// std::clock_t starttime=std::clock();

	this->findPairsFlann(older_node, &mr.all_matches); 
	
	if (mr.all_matches.size() < min_matches)
	{
		cout<<"all_matches < min_matches!"<<endl;
	} 
	else 
	{
		if (!getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches) )
		{
			cout<<"failed to find corresponding matches!"<<endl;
		} 
		else  
		{
			// + here we want to use correspond points
			vector<pcl::TMatchingPair> cors;

			// 此处测试只利用少数点对最后的影响如何
			int num_f=0;

			for(std::vector<cv::DMatch>::iterator it = mr.inlier_matches.begin();
				it!= mr.inlier_matches.end(); it++)
			{
				if(num_f++>=3) break;
				Eigen::Vector4f src = feature_locations_3d_[(*it).queryIdx];
				Eigen::Vector4f dst = older_node->feature_locations_3d_[(*it).trainIdx];
				// record matched pairs of points
				cors.push_back(pcl::TMatchingPair(0,0,dst[0],dst[1],dst[2],src[0],src[1],src[2]));
			}

			pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
			CPose3D finalPose;
			//CPose3D trusted_pose = curr_pose - last_pose;
			icp.leastSquareErrorRigidTransformation6D(cors,finalPose);
			//icp.leastSquareErrorRigidTransformation6D(trusted_pose,cors,finalPose);

			//mr.final_trafo = mr.ransac_trafo;
			finalPose.getHomogeneousMatrix(mr.final_trafo);
			mr.edge.id1 = older_node->id_;//and we have a valid transformation
			mr.edge.id2 = this->id_; //since there are enough matching features,
			mr.edge.mean = eigen2Hogman(mr.final_trafo);//we insert an edge between the frames
			mr.edge.informationMatrix =   Matrix6::eye(mr.inlier_matches.size()*mr.inlier_matches.size()); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)
		}
	}
	return mr;
}

// 获取特征点的3D坐标
void Node::getFeatureLoc(vector<int> indices,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_out)
{
	int N_f = feature_locations_2d_.size();
	for(int i=0;i<indices.size();i++)
	{
		int index=indices[i];
		if(index<0 || index>=N_f){
			cout<<"error in getFeatureLoc()"<<endl;
			continue;
		}
		cv::Point2f p2d = feature_locations_2d_[index].pt;
		if (p2d.x >= m_pPC->width || p2d.x < 0 ||
			p2d.y >= m_pPC->height || p2d.y < 0 ||
			isnan(p2d.x) || isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
				cout<<"error in getFeatureLoc()"<<endl;
				continue;
		}
		Eigen::Vector4f p3d1=feature_locations_3d_[index];
		pcl::PointXYZRGB p3d; //= m_feature_locations_3d_;//m_pPC->at((int) p2d.x,(int) p2d.y);
		p3d.x=p3d1[0]; p3d.y=p3d1[1]; p3d.z=p3d1[2];
		m_out->points.push_back(p3d);
	}
}

///Get euler angles from affine matrix (helper for isBigTrafo)
void Node::mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw) {
  roll = atan2(t(2,1),t(2,2));
  pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
  yaw = atan2(t(1,0),t(0,0));
}

void Node::mat2components(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist){

  mat2RPY(t, roll,pitch,yaw);
  mat2dist(t, dist);

  roll = roll/M_PI*180;
  pitch = pitch/M_PI*180;
  yaw = yaw/M_PI*180;

}

