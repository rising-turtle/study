#ifndef CPOSE3D_H
#define CPOSE3D_H
#include <Eigen/Core>
#include <cmath>
#include <iostream>

// #define pi 3.141592654
#define R2D(r) ((r)/M_PI*180)
#define equal_thresh 1e-5

using namespace std;


class CPoint3D{
public:
	CPoint3D(){x=0;y=0;z=0;}
	~CPoint3D(){}

	inline void x_incr(double _x){ x+=_x;}
	inline void y_incr(double _y){ y+=_y;}
	inline void z_incr(double _z){ z+=_z;}

	void operator*=(double factor){
		x*=factor; y*=factor; z*=factor;
		return ;
	}

	double x,y,z;
};

class CPose3D{
public:
	CPose3D():yaw(0),pitch(0),roll(0){m_coords[0]=m_coords[1]=m_coords[2]=0;rebuildRotationMatrix();normalizeQ();}
	//CPose3D(const double x,const double  y,const double  z,const double  yaw=0, const double  pitch=0, const double roll=0);
	CPose3D(Eigen::Matrix4f &transformation )
    {
		UpdatePosewithMatrix4f(transformation);
		FromEular2Quaternions();
	}
	CPose3D(double _yaw,double _pitch,double _roll,double x,double y, double z):yaw(_yaw),pitch(_pitch),roll(_roll)
	{
		m_coords[0]=x;m_coords[1]=y;m_coords[2]=z;
		rebuildRotationMatrix();
		FromEular2Quaternions();
	}
	CPose3D(double _q[4])
    {
        m_coords[0]=m_coords[1]=m_coords[2]=0;
        for(size_t i=0;i<4;i++) q[i] = _q[i];
        rotationMatrixNoResize();
        getYawPitchRoll(yaw,pitch,roll);		
	}
	CPose3D(const CPose3D& _pose)
    {
		m_coords[0]= _pose.m_coords[0];m_coords[1]=_pose.m_coords[1];m_coords[2] = _pose.m_coords[2]; // translation
		yaw = _pose.yaw; roll = _pose.roll; pitch = _pose.pitch;									  // pose
		q[0] = _pose.q[0]; q[1] = _pose.q[1]; q[2] = _pose.q[2]; q[3] = _pose.q[3];					  // Quaternions
		m_ROT = _pose.m_ROT;
		//rebuildRotationMatrix();
	}
	~CPose3D(){}
	CPose3D operator+(const CPose3D& ); // 
	CPose3D operator-(const CPose3D& );
	CPose3D operator+=(const CPose3D& ); // 
	CPose3D operator = (const CPose3D&);
	inline bool operator ==(const CPose3D& pose){
		if( fabs(m_coords[0] - pose.m_coords[0]) < equal_thresh && fabs(m_coords[1] - pose.m_coords[1]) < equal_thresh && fabs(m_coords[2] - pose.m_coords[2]) <equal_thresh \
			&& fabs(yaw-pose.yaw) < equal_thresh && fabs(roll - pose.roll) < equal_thresh && fabs(pitch - pose.pitch) < equal_thresh)
			return true;
		return false;
	}
	inline void setxyz(double x,double y, double z){m_coords[0]=x; m_coords[1]=y; m_coords[2]=z;}
	void setrpy(double r,double p,double y){yaw=y;pitch=p;roll=r;rebuildRotationMatrix();FromEular2Quaternions();}
	inline void getXYZ(double xyz[3]){xyz[0] = m_coords[0];xyz[1]=m_coords[1];xyz[2]=m_coords[2];}
	inline void getrpy(double rpy[3]){rpy[0] = roll; rpy[1] = pitch; rpy[2] = yaw;}
	inline void normalizeQ(){q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;}

	ostream& output(ostream& out)
	{
		out<<"(x,y,z)" <<"("<<m_coords[0]<<","<<m_coords[1]<<","<<m_coords[2]<<")" <<endl;
		out<<"(r,p,y)" <<"("<<R2D(roll)<<","<<R2D(pitch)<<","<<R2D(yaw)<<")"<<endl;
		return out;
	}
	
	inline void displayROT()
	{
		for(size_t i=0;i<3;i++)
		{
			cout<<"{ ";
			for(size_t j=0;j<3;j++)
				cout<<m_ROT(i,j)<<", ";
			cout<<" }"<<endl;
		}
	}

	/*6dof (x,y,z,yaw,roll,pitch)*/
	double m_coords[3]; // location
	double yaw,roll,pitch; // posture
	double q[4]; //quaternion 
 
	Eigen::Matrix3f m_ROT; // Rotation Matrix

public:

	/*Calculate posture angles from Rotation matrix*/
	void getYawPitchRoll( double &yaw, double &pitch, double &roll );

	/** Rebuild the homog matrix from the angles. */
	void rebuildRotationMatrix();

	// Get Quaternions from Eular Angles
	void FromEular2Quaternions();

	/**  Makes "this = A (+) B"; this method is slightly more efficient than "this= A + B;" since it avoids the temporary object.
	*  \note A or B can be "this" without problems.
	*/
	void composeFrom(const CPose3D& A, const CPose3D& B );

	/**  Makes \f$ this = A \ominus B \f$ this method is slightly more efficient than "this= A - B;" since it avoids the temporary object.
	*  \note A or B can be "this" without problems.
	* \sa composeFrom, composePoint
	*/
	void inverseComposeFrom(const CPose3D& A, const CPose3D& B );

	void homogeneousMatrixInverse(const Eigen::Matrix3f  & in_R,const Eigen::Vector3f  & in_xyz, \
		Eigen::Matrix3f & out_R,Eigen::Vector3f & out_xyz);

	// obtain Matrix44(Rotation + Translation)
	void getHomogeneousMatrix(Eigen::Matrix4f & out_HM );
	
	//Update CPose3D using Matrix4f
	void UpdatePosewithMatrix4f(Eigen::Matrix4f &transformation);

	// rotate point
	void rotatePoint( double lx, double ly, double lz, double &gx,double &gy,double &gz );

	// rotate and translate point
	void transformPoint( double lx, double ly, double lz, double &gx,double &gy,double &gz);

	// build rotation matrix with q[4]
	void  rotationMatrixNoResize();
};

#endif
