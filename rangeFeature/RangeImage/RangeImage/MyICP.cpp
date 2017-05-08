#include "MyICP.h"
#include <vector>


template <>
void pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB>::findIntersectedPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud){
	
	std::vector<int> _indices;
	size_t total_n = pCloud->points.size();
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator it = pCloud->points.begin();

	std::vector<int> indices(1);
	std::vector<float> distances(1);

	for(size_t i=0; i< total_n; ){
		if(!searchForNeighbors(*pCloud,i,indices,distances)) // if failed to find matched point, erase this point
		{
			it = pCloud->points.erase(it);
			total_n --;
			continue;
		}
		i++;
		it++;
	}
}


template <>
void pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB>::MyicpAlign(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud, boost::shared_ptr<CPose3D> Pose)
{
	
	CPose3D lastPose(*Pose);
	
	vector<pcl::TMatchingPair> correspondences; // matching result

	int total_num = pCloud->points.size();
	std::vector<int> indices(1);
	std::vector<float> distances(1);
	/*pcl::PointXYZRGB * psrc;
	pcl::PointXYZRGB * ptar;*/
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator psrcbegin = pCloud->points.begin();
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::const_iterator pdstbegin = target_->points.begin();
	
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator psrc;
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::const_iterator ptar;


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pDummyCloud(pCloud->makeShared());

	// icp parameters
	int nCorrespondences;
	float minDis = 0.005;
	int maxIteration = 20;
	float alfa = 0.25;
	float cur_dis = 0.04;//0.2; 
	int nIterations = 0;
	bool approach = true;

	// ------------------------------------------------------
	//					The ICP loop
	// ------------------------------------------------------
	do
	{
		// 1 transform src_cloud
		cloudRegistration(Pose,pDummyCloud);

		// 2 find the matching (for a points map)
		for(size_t i=0;i<total_num; i++){
			if(tree_->nearestKSearch (*pDummyCloud, i, 1, indices, distances) != 0){ // find match pair
				if(distances[0] > cur_dis) continue; // if this pair is too far from each other
				psrc = psrcbegin + i;//&pCloud->points[i];
				ptar = pdstbegin + indices[0];//&input_->points[indices[0]];
				correspondences.push_back(TMatchingPair(indices[0],i,(*ptar).x,(*ptar).y,(*ptar).z,(*psrc).x,(*psrc).y,(*psrc).z));
			}
		}
		// 3 compute transformation matrix
		nCorrespondences = correspondences.size();
		if(!nCorrespondences){
			//approach = false ;
			return;
		}

		nCorrespondences = correspondences.size();
		leastSquareErrorRigidTransformation6D(correspondences,*Pose);

		// 4 if Pose not pretty changed and cur_dis has reached to minDis
		if(lastPose == *Pose){ // almost the same
			cur_dis *= alfa;
			if(cur_dis <= minDis)
				return ;
		}
		lastPose = *Pose;
		nIterations++;
	}while(nIterations < maxIteration);
}

template<>
bool pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB>::leastSquareErrorRigidTransformation6D(
	std::vector<pcl::TMatchingPair>	&in_correspondences,
	CPose3D							&out_transformation)
{
	if( in_correspondences.size() < 3 )
	{
		return false;
	}

	// Print the rotation matrix and translation vector
	//Eigen::Matrix3f rotation = result.block<3,3>(0, 0);
	//Eigen::Vector3f translation = result.block<3,1>(0, 3);

	CPoint3D cL, cR;
	Eigen::Matrix3f S;
	Eigen::Matrix4f N;
	Eigen::Matrix4f Z;
	Eigen::Vector4f D;

	const size_t nMatches = in_correspondences.size();
	double s; // Scale

	// Compute the centroid
	vector<pcl::TMatchingPair>::const_iterator	itMatch;

	for(itMatch = in_correspondences.begin(); itMatch != in_correspondences.end(); itMatch++)
	{
		cL.x_incr( itMatch->other_x );
		cL.y_incr( itMatch->other_y );
		cL.z_incr( itMatch->other_z );

		cR.x_incr( itMatch->this_x );
		cR.y_incr( itMatch->this_y );
		cR.z_incr( itMatch->this_z );
	}
	const double F = 1.0/nMatches;
	cL *= F;
	cR *= F;

	vector<pcl::TMatchingPair> auxList( in_correspondences );
	vector<pcl::TMatchingPair>::iterator auxIt;
	// Substract the centroid
	for( auxIt = auxList.begin(); auxIt != auxList.end(); auxIt++ )
	{
		auxIt->other_x -= cL.x;
		auxIt->other_y -= cL.y;
		auxIt->other_z -= cL.z;

		auxIt->this_x -= cR.x;
		auxIt->this_y -= cR.y;
		auxIt->this_z -= cR.z;
	}

	// Compute the S matrix of products
	for(size_t i=0;i<3;i++)
		for(size_t j=0;j<3;j++)
			S(i,j) = 0;
	for( auxIt = auxList.begin(); auxIt != auxList.end(); auxIt++ )
	{
		S(0,0) += auxIt->other_x * auxIt->this_x;
		S(0,1) += auxIt->other_x * auxIt->this_y;
		S(0,2) += auxIt->other_x * auxIt->this_z;

		S(1,0) += auxIt->other_y * auxIt->this_x;
		S(1,1) += auxIt->other_y * auxIt->this_y;
		S(1,2) += auxIt->other_y * auxIt->this_z;

		S(2,0) += auxIt->other_z * auxIt->this_x;
		S(2,1) += auxIt->other_z * auxIt->this_y;
		S(2,2) += auxIt->other_z * auxIt->this_z;
	}

	N(0,0) = S(0,0) + S(1,1) + S(2,2);
	N(0,1) = S(1,2) - S(2,1);
	N(0,2) = S(2,0) - S(0,2);
	N(0,3) = S(0,1) - S(1,0);

	N(1,0) = N(0,1);
	N(1,1) = S(0,0) - S(1,1) - S(2,2);
	N(1,2) = S(0,1) + S(1,0);
	N(1,3) = S(2,0) + S(0,2);

	N(2,0) = N(0,2);
	N(2,1) = N(1,2);
	N(2,2) = -S(0,0) + S(1,1) - S(2,2);
	N(2,3) = S(1,2) + S(2,1);

	N(3,0) = N(0,3);
	N(3,1) = N(1,3);
	N(3,2) = N(2,3);
	N(3,3) = -S(0,0) - S(1,1) + S(2,2);

	///*Just for easily catch values in N*/
	//double Tmp[4][4];
	//for(int i=0;i<4;i++)
	//	for(int j=0;j<4;j++)
	//		Tmp[i][j] = N(i,j);
	// q is the quaternion correspondent to the greatest eigenvector of the N matrix (last column in Z)

	// now we just have to pick the eigen vector with smallest eigen value
	//Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> eig(N,1);
	//Z = eig.eigenvectors();//.col(3);
	

	Eigen::EigenSolver< Eigen::Matrix4f > es(N, true);
	Z = es.eigenvectors().real(); // Keep only the real part of complex matrix
	D = es.eigenvalues().real(); // Keep only the real part of complex matrix

	int max_index = 0;
	for(size_t i=1;i<4;i++){
		if(D[i] > D[max_index])
			max_index = i;
	}
	D = Z.block<4,1>(0,max_index);

	/*Eigen::EigenSolver<Eigen::Matrix4f> eigensolver(N,true);
	Z = eigensolver.eigenvectors();*/
	/*Z = eigensolver.eigenvectors().;
	D =  eigensolver.eigenvectors().col(3);*/

	//N.eigenVectors( Z, D );
	//Z.extractCol( Z.getColCount()-1, v );

	double q[4];
	
	for(unsigned int i = 0; i < 4; i++ )			// Set out_transformation [rotation]
		q[i] = D[i];
	CPose3D _q(q);

	CPoint3D pp, aux;
	_q.rotatePoint( cL.x, cL.y, cL.z, pp.x, pp.y, pp.z );
	
	// Set out_transformation [traslation]
	_q.m_coords[0] = cR.x - pp.x;	// X
	_q.m_coords[1] = cR.y - pp.y;	// Y
	_q.m_coords[2] = cR.z - pp.z;	// Z

	out_transformation = _q;
	return true;
	/*---------------------------------------------------------------
	leastSquareErrorRigidTransformation in 6D
	---------------------------------------------------------------*/
	// Algorithm:
	// 0. Preliminary
	//		pLi = { pLix, pLiy, pLiz }
	//		pRi = { pRix, pRiy, pRiz }
	// -------------------------------------------------------
	// 1. Find the centroids of the two sets of measurements:
	//		cL = (1/n)*sum{i}( pLi )		cL = { cLx, cLy, cLz }
	//		cR = (1/n)*sum{i}( pRi )		cR = { cRx, cRy, cRz }
	//
	// 2. Substract centroids from the point coordinates:
	//		pLi' = pLi - cL					pLi' = { pLix', pLiy', pLiz' }
	//		pRi' = pRi - cR					pRi' = { pRix', pRiy', pRiz' }
	//
	// 3. For each pair of coordinates (correspondences) compute the nine possible products:
	//		pi1 = pLix'*pRix'		pi2 = pLix'*pRiy'		pi3 = pLix'*pRiz'
	//		pi4 = pLiy'*pRix'		pi5 = pLiy'*pRiy'		pi6 = pLiy'*pRiz'
	//		pi7 = pLiz'*pRix'		pi8 = pLiz'*pRiy'		pi9 = pLiz'*pRiz'
	//
	// 4. Compute S components:
	//		Sxx = sum{i}( pi1 )		Sxy = sum{i}( pi2 )		Sxz = sum{i}( pi3 )
	//		Syx = sum{i}( pi4 )		Syy = sum{i}( pi5 )		Syz = sum{i}( pi6 )
	//		Szx = sum{i}( pi7 )		Szy = sum{i}( pi8 )		Szz = sum{i}( pi9 )
	//
	// 5. Compute N components:
	//			[ Sxx+Syy+Szz	Syz-Szy			Szx-Sxz			Sxy-Syx		 ]
	//			[ Syz-Szy		Sxx-Syy-Szz		Sxy+Syx			Szx+Sxz		 ]
	//		N = [ Szx-Sxz		Sxy+Syx			-Sxx+Syy-Szz	Syz+Szy		 ]
	//			[ Sxy-Syx		Szx+Sxz			Syz+Szy			-Sxx-Syy+Szz ]
	//
	// 6. Rotation represented by the quaternion eigenvector correspondent to the higher eigenvalue of N
	//
	// 7. Scale computation (symmetric expression)
	//		s = sqrt( sum{i}( square(abs(pRi')) / sum{i}( square(abs(pLi')) ) )
	//
	// 8. Translation computation (distance between the Right centroid and the scaled and rotated Left centroid)
	//		t = cR-sR(cL)
}


template<>
void pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB>::testleastsqureerror()
{
	string filename("D:\\PCL_install_on_VS2008\\match_points.txt");
	ifstream file(filename.c_str());
	if(file.is_open()){
		char buf[100];
		file.getline(buf,100);
		vector<TMatchingPair> cors;

		double that_x,that_y,that_z,this_x,this_y,this_z;
		while(file.getline(buf,100)){
			sscanf(buf,"%lf %lf %lf %lf %lf %lf",&that_x,&that_y,&that_z,&this_x,&this_y,&this_z);
			cors.push_back(TMatchingPair(0,0,this_x,this_y,this_z,that_x,that_y,that_z));
		}
		CPose3D pose;
		double transcale;
		leastSquareErrorRigidTransformation6D(cors,pose);
		pose.output(std::cout);
	}

}