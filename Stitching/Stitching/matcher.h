/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 9 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#if !defined MATCHER
#define MATCHER

#include <vector>
#include <opencv2/core/core.hpp>
#include "opencv2/core/internal.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


class RobustMatcher {

  private:

	  // pointer to the feature point detector object
	  cv::Ptr<cv::FeatureDetector> detector;
	  // pointer to the feature descriptor extractor object
	  cv::Ptr<cv::DescriptorExtractor> extractor;
	  float ratio; // max ratio between 1st and 2nd NN
	  bool refineF; // if true will refine the F matrix
	  double distance; // min distance to epipolar
	  double confidence; // confidence level (probability)

  public:

	  RobustMatcher() : ratio(0.65f), refineF(true), confidence(0.99), distance(3.0) {	  

		  // SURF is the default feature
		  // detector= new cv::SurfFeatureDetector();
		  // extractor= new cv::SurfDescriptorExtractor();
		  detector = cv::Algorithm::create<cv::FeatureDetector>("Feature2D.SURF");
		  extractor = cv::Algorithm::create<cv::DescriptorExtractor>("Feature2D.SURF");
	  }

	  // Set the feature detector
	  void setFeatureDetector(cv::Ptr<cv::FeatureDetector>& detect) {

		  detector= detect;
	  }

	  // Set descriptor extractor
	  void setDescriptorExtractor(cv::Ptr<cv::DescriptorExtractor>& desc) {

		  extractor= desc;
	  }

	  // Set the minimum distance to epipolar in RANSAC
	  void setMinDistanceToEpipolar(double d) {

		  distance= d;
	  }

	  // Set confidence level in RANSAC
	  void setConfidenceLevel(double c) {

		  confidence= c;
	  }

	  // Set the NN ratio
	  void setRatio(float r) {

		  ratio= r;
	  }

	  // if you want the F matrix to be recalculated
	  void refineFundamental(bool flag) {

		  refineF= flag;
	  }

	  // Clear matches for which NN ratio is > than threshold
	  // return the number of removed points 
	  // (corresponding entries being cleared, i.e. size will be 0)
	  int ratioTest(std::vector<std::vector<cv::DMatch>>& matches);

	  // Insert symmetrical matches in symMatches vector
	  void symmetryTest(const std::vector<std::vector<cv::DMatch>>& matches1,
		  const std::vector<std::vector<cv::DMatch>>& matches2,
		  std::vector<cv::DMatch>& symMatches);

	  // Identify good matches using RANSAC
	  cv::Mat ransacTest(const std::vector<cv::DMatch>& matches,
		  const std::vector<cv::KeyPoint>& keypoints1, 
		  const std::vector<cv::KeyPoint>& keypoints2,
		  std::vector<cv::DMatch>& outMatches);

	  // Return fundemental matrix
	  cv::Mat match(cv::Mat& image1, cv::Mat& image2, // input images 
		  std::vector<cv::DMatch>& matches, // output matches and keypoints
		  std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2);
public:
	void getKeyPoints(cv::Mat& img,std::vector<cv::KeyPoint>& kpts,cv::Mat& des);
	// calculate Homography Matrix from img2 -> img1 
	// img1 : tar_image ; img2 : src_image
	int calHomographyMatrix(cv::Mat& img1,
		cv::Mat& img2,
		std::vector<cv::KeyPoint>& kpts1,
		std::vector<cv::KeyPoint>& kpts2,
		cv::Mat& des1,
		cv::Mat& des2,
		cv::Mat& outMatrix);
};

#endif
