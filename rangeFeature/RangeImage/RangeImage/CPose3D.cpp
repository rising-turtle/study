#include "CPose3D.h"

#define square(x) ((x)*(x))

using std::cout;
using std::endl;

/*---------------------------------------------------------------
getYawPitchRoll
---------------------------------------------------------------*/
void  CPose3D::getYawPitchRoll( double &yaw, double &pitch, double &roll ) 
{
	// Pitch is in the range [-pi/2, pi/2 ], so this calculation is enough:
	pitch =  atan2( (double)(- m_ROT(2,0)), (double)hypot( m_ROT(0,0),m_ROT(1,0) ) ); //asin( - m_ROT(2,0) );

	// Roll:
	if ( (fabs(m_ROT(2,1))+fabs(m_ROT(2,2)))<10*std::numeric_limits<double>::epsilon() )
	{
		//Gimbal lock between yaw and roll. This one is arbitrarily forced to be zero.
		//Check http://reference.mrpt.org/svn/classmrpt_1_1poses_1_1_c_pose3_d.html. If cos(pitch)==0, the homogeneous matrix is:
		//When sin(pitch)==1:
		//  /0  cysr-sycr cycr+sysr x\   /0  sin(r-y) cos(r-y)  x\.
		//  |0  sysr+cycr sycr-cysr y| = |0  cos(r-y) -sin(r-y) y|
		//  |-1     0         0     z|   |-1    0         0     z|
		//  \0      0         0     1/   \0     0         0     1/
		//
		//And when sin(pitch)=-1:
		//  /0 -cysr-sycr -cycr+sysr x\   /0 -sin(r+y) -cos(r+y) x\.
		//  |0 -sysr+cycr -sycr-cysr y| = |0 cos(r+y)  -sin(r+y) y|
		//  |1      0          0     z|   |1    0          0     z|
		//  \0      0          0     1/   \0    0          0     1/
		//
		//Both cases are in a "gimbal lock" status. This happens because pitch is vertical.

		roll = 0.0;
		if (pitch>0) yaw=atan2((double)m_ROT(1,2),(double)m_ROT(0,2));
		else yaw=atan2((double)(-m_ROT(1,2)),(double)(-m_ROT(0,2)));
	}
	else
	{
		roll = atan2( (double)(m_ROT(2,1)), (double)m_ROT(2,2) );
		// Yaw:
		yaw = atan2( (double)(m_ROT(1,0)), (double)(m_ROT(0,0)) );
	}
	//// added by SS 2012/3/19
	//double dSinB=- m_ROT(2,0);
	//double dCosA=cos(yaw);
	//double dCosB=m_ROT(0,0)/dCosA;
	//if (dSinB>=0&&dCosA>=0)
	//{

	//}
	//else if (dSinB<0&&dCosA>0)
	//{
	//}
	//else if (dSinB>0&&dCosA<0)
	//{
	//	pitch=3.1415926-pitch;
	//}
	//else
	//{
	//	pitch=-3.1415926-pitch;

	//}

}
// get Quaternions from Eular Angles
void CPose3D::FromEular2Quaternions()
{
	//updateYawPitchRoll();
	// See: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	const double	cy = cos(yaw*0.5);
	const double	sy = sin(yaw*0.5);
	const double	cp = cos(pitch*0.5);
	const double	sp = sin(pitch*0.5);
	const double	cr = cos(roll*0.5);
	const double	sr = sin(roll*0.5);

	const double ccc = cr*cp*cy;
	const double ccs = cr*cp*sy;
	const double css = cr*sp*sy;
	const double sss = sr*sp*sy;
	const double scc = sr*cp*cy;
	const double ssc = sr*sp*cy;
	const double csc = cr*sp*cy;
	const double scs = sr*cp*sy;

	q[0] = ccc+sss;
	q[1] = scc-css;
	q[2] = csc+scs;
	q[3] = ccs-ssc;
}
void CPose3D::rebuildRotationMatrix(){
	const double	cy = cos(yaw);
	const double	sy = sin(yaw);
	const double	cp = cos(pitch);
	const double	sp = sin(pitch);
	const double	cr = cos(roll);
	const double	sr = sin(roll);

	EIGEN_ALIGN16  double rot_vals[] = {
		cy*cp,      cy*sp*sr-sy*cr,     cy*sp*cr+sy*sr,
		sy*cp,      sy*sp*sr+cy*cr,     sy*sp*cr-cy*sr,
		-sp,        cp*sr,              cp*cr
	};
	for(size_t i=0;i<3;i++)
		for(size_t j=0;j<3;j++)
			m_ROT(i,j) = rot_vals[i*3 + j];
}
/**  Makes "this = A (+) B"; this method is slightly more efficient than "this= A + B;" since it avoids the temporary object.
*  \note A or B can be "this" without problems.
*/
/*---------------------------------------------------------------
this = A + B
---------------------------------------------------------------*/
void CPose3D::composeFrom(const CPose3D& A, const CPose3D& B )
{
	//Was: m_HM.multiply( A.m_HM, B.m_HM );
	m_ROT=  A.m_ROT* B.m_ROT ;

	// The translation part HM(0:3,3)
	if (this==&B)
	{
		// we need to make a temporary copy of the vector:
		const double*  B_coords = B.m_coords;
		for (int r=0;r<3;r++)
			m_coords[r] = A.m_coords[r] + A.m_ROT(r,0)*B_coords[0]+A.m_ROT(r,1)*B_coords[1]+A.m_ROT(r,2)*B_coords[2];
	}
	else
	{
		for (int r=0;r<3;r++)
			m_coords[r] = A.m_coords[r] + A.m_ROT(r,0)*B.m_coords[0]+A.m_ROT(r,1)*B.m_coords[1]+A.m_ROT(r,2)*B.m_coords[2];
	}

	//m_ypr_uptodate=false;
}
//
///** Rotate a 3D point (lx,ly,lz) -> (gx,gy,gz) as described by this quaternion
//*/
//void CPose3D::rotatePoint( double lx, double ly, double lz, double &gx,double &gy,double &gz ) 
//{
//	const double t2 = q[0]*q[1]; const double t3 = q[0]*q[2]; const double t4 = q[0]*q[3]; const double t5 =-q[1]*q[1]; const double t6 = q[1]*q[2];
//	const double t7 = q[1]*q[3]; const double t8 =-q[2]*q[2]; const double t9 = q[2]*q[3]; const double t10=-q[3]*q[3];
//	gx = 2*((t8+ t10)*lx+(t6 - t4)*ly+(t3+t7)*lz)+lx;
//	gy = 2*((t4+  t6)*lx+(t5 +t10)*ly+(t9-t2)*lz)+ly;
//	gz = 2*((t7-  t3)*lx+(t2 + t9)*ly+(t5+t8)*lz)+lz;
//}

CPose3D CPose3D::operator+(const CPose3D& current){
	CPose3D   ret;
	ret.composeFrom(*this,current);
	ret.getYawPitchRoll(ret.yaw,ret.pitch,ret.roll);
	ret.FromEular2Quaternions();
	return ret;
}
CPose3D CPose3D::operator-(const CPose3D& b)
{
	CPose3D ret;
	ret.inverseComposeFrom(*this,b);
	ret.getYawPitchRoll(ret.yaw,ret.pitch,ret.roll);
	ret.FromEular2Quaternions();
	return ret;
}
CPose3D CPose3D::operator+=(const CPose3D& current ){
	composeFrom(*this,current);
	this->getYawPitchRoll(yaw,pitch,roll);
	this->FromEular2Quaternions();
	return *this;
}
CPose3D CPose3D::operator =(const CPose3D& v){
	if(this == &v){
		return *this;
	}
	for(size_t i=0;i<3;i++)
		this->m_coords[i] = v.m_coords[i];
	for(size_t i=0;i<4;i++)
		this->q[i] = v.q[i];
	this->yaw = v.yaw; this->pitch = v.pitch; this->roll = v.roll;
	this->m_ROT = v.m_ROT;
	return *this;
}

// obtain Matrix44(Rotation + Translation)
void  CPose3D::getHomogeneousMatrix(Eigen::Matrix4f & out_HM ) 
{
	for(size_t i=0;i<3; i++)
		for(size_t j=0;j<3;j++)
			out_HM(i,j) = m_ROT(i,j);
	for (int i=0;i<3;i++) out_HM(i,3)=m_coords[i];
	out_HM(3,0)=out_HM(3,1)=out_HM(3,2)=0.; out_HM(3,3)=1.;
}

//Update CPose3D using Matrix4f
void CPose3D::UpdatePosewithMatrix4f(Eigen::Matrix4f &transformation){
	m_ROT = transformation.block<3,3>(0, 0);
	Eigen::Vector3f translation = transformation.block<3,1>(0, 3);
	m_coords[0] = translation(0); m_coords[1] = translation(1); m_coords[2] = translation(2);
	getYawPitchRoll(yaw,pitch,roll);
}

// rotate point
void CPose3D::rotatePoint( double lx, double ly, double lz, double &gx,double &gy,double &gz ){
	const double t2 = q[0]*q[1]; const double t3 = q[0]*q[2]; const double t4 = q[0]*q[3]; const double t5 =-q[1]*q[1]; const double t6 = q[1]*q[2];
	const double t7 = q[1]*q[3]; const double t8 =-q[2]*q[2]; const double t9 = q[2]*q[3]; const double t10=-q[3]*q[3];
	gx = 2*((t8+ t10)*lx+(t6 - t4)*ly+(t3+t7)*lz)+lx;
	gy = 2*((t4+  t6)*lx+(t5 +t10)*ly+(t9-t2)*lz)+ly;
	gz = 2*((t7-  t3)*lx+(t2 + t9)*ly+(t5+t8)*lz)+lz;
}

// rotate and translate point
void CPose3D::transformPoint( double lx, double ly, double lz, double &gx,double &gy,double &gz){
	rotatePoint(lx,ly,lz,gx,gy,gz);
	gx -= m_coords[0];
	gy -= m_coords[1];
	gz -= m_coords[2];
}

// build rotation matrix with q[4]
void CPose3D::rotationMatrixNoResize() 
{
	m_ROT(0,0)=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];		m_ROT(0,1)=2*(q[1]*q[2] -q[0]*q[3]);			m_ROT(0,2)=2*(q[3]*q[1]+q[0]*q[2]);
	m_ROT(1,0)=2*(q[1]*q[2]+q[0]*q[3]);				m_ROT(1,1)=q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];	    m_ROT(1,2)=2*(q[2]*q[3]-q[0]*q[1]);
	m_ROT(2,0)=2*(q[3]*q[1]-q[0]*q[2]);				m_ROT(2,1)=2*(q[2]*q[3]+q[0]*q[1]);				        m_ROT(2,2)=q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
}
void CPose3D::inverseComposeFrom(const CPose3D& A, const CPose3D& B )
{
	// this    =    A  (-)  B
	// HM_this = inv(HM_B) * HM_A
	//
	// [  R_b  | t_b ] -1   [  R_a  | t_a ]    [ R_b^t * Ra |    ..    ]
	// [ ------+-----]    * [ ------+-----]  = [ ---------- +----------]
	// [ 0 0 0 |  1  ]      [ 0 0 0 |  1  ]    [  0  0   0  |      1   ]
	//

	// XYZ part:
	Eigen::Matrix3f  R_b_inv;
	Eigen::Vector3f  t_b_inv;
	
	Eigen::Vector3f  B_m_coords;
	B_m_coords[0] = B.m_coords[0]; B_m_coords[1]= B.m_coords[1]; B_m_coords[2]= B.m_coords[2];

	homogeneousMatrixInverse(B.m_ROT,B_m_coords,  R_b_inv,t_b_inv);

	for (int i=0;i<3;i++)
		m_coords[i] = t_b_inv[i] + R_b_inv(i,0)*A.m_coords[0]+ R_b_inv(i,1)*A.m_coords[1]+ R_b_inv(i,2)*A.m_coords[2];

	// Rot part:
	m_ROT = R_b_inv * A.m_ROT;
}


void CPose3D::homogeneousMatrixInverse(
							  const Eigen::Matrix3f  & in_R,
							  const Eigen::Vector3f  & in_xyz,
							  Eigen::Matrix3f & out_R,
							  Eigen::Vector3f & out_xyz
							  )
{
	// translation part:
	const double tx = -in_xyz[0];
	const double ty = -in_xyz[1];
	const double tz = -in_xyz[2];

	out_xyz[0] = tx*in_R(0,0)+ty*in_R(1,0)+tz*in_R(2,0);
	out_xyz[1] = tx*in_R(0,1)+ty*in_R(1,1)+tz*in_R(2,1);
	out_xyz[2] = tx*in_R(0,2)+ty*in_R(1,2)+tz*in_R(2,2);

	// 3x3 rotation part: transpose
	out_R = in_R.transpose();	
}