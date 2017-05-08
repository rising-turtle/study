# ifndef PLANE_SEGMENTATION_H_
#define PLANE_SEGMENTATION_H_

#include <vector>
#include <Eigen/Dense>


class PlaneSegmentation
{
private:

public:
	PlaneSegmentation();
	~PlaneSegmentation();

	inline void setAngularThreshold (double angularThreshold)
	{
		angularThreshold_ = angularThreshold;
	}

	inline void setDistanceThreshold (double distanceThreshold)
	{
		distanceThreshold_ = distanceThreshold;
	}

	inline void setPlaneThreshold (int minPlaneCount,double minPlaneArea)
	{
		minPlaneCount_ = minPlaneCount;
		minPlaneArea_ = minPlaneArea;
	}

	inline void setMaxVisDistance(double maxVisDistance)
	{
		maxVisDistance_=maxVisDistance;
	}

	//return the num of planes, classFlag sign the plane index the point belong to
	int segmentOrganized (float** cloud,float** normals,const int width,const int height,int classFlag[]);
	
	//get all planes' equition (a,b,c,d)  a*x + b*y +c*z + d = 0  
 	void gettPlaneModel(std::vector<double> modelEquition[4]);

	

protected:
	double angularThreshold_;       //拓展点与平面法向量夹角
	double distanceThreshold_;      //拓展点到平面的距离
	int minPlaneCount_;            //最少平面点数
	double minPlaneArea_;          //最小平面面积
	double maxVisDistance_;        //最远视距

	//int downSample(double** dataIn,double** dataOut,const int width,const int height,const int downStep);

	struct PlaneModel
	{
		Eigen::Vector3d centroid;
		Eigen::Vector3d normal;
	};
	std::vector<PlaneModel> planeVector_;
};


#endif