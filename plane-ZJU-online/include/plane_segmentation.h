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
	double angularThreshold_;       //��չ����ƽ�淨�����н�
	double distanceThreshold_;      //��չ�㵽ƽ��ľ���
	int minPlaneCount_;            //����ƽ�����
	double minPlaneArea_;          //��Сƽ�����
	double maxVisDistance_;        //��Զ�Ӿ�

	//int downSample(double** dataIn,double** dataOut,const int width,const int height,const int downStep);

	struct PlaneModel
	{
		Eigen::Vector3d centroid;
		Eigen::Vector3d normal;
	};
	std::vector<PlaneModel> planeVector_;
};


#endif