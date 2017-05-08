#include "FromPC2Triangles.h"
#include "HashTable.h"
#include "Hash_Func.h"

void CFromPC2Triangle::compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_input,\
							   boost::shared_ptr<Triangles>& m_trianlges)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_sparsePC(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<int> m_indices;
	FilterDensePC(m_input,m_sparsePC,m_indices);
	
	
}

void CFromPC2Triangle::FilterDensePC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ori,
			  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sparse, vector<int>& out_indices)
{
	/*map<IPt,int> record_map;
	vector<vector<int> > indices_collector;*/

	// using hash table to filter points
	CHashTableSet<hash_func1,IPt> tmpHash;
	double start_count=::GetTickCount();
	for(size_t i=0;i<ori->points.size();i++)
	{
		pcl::PointXYZRGB& sp=ori->points[i];

		if(_isnan(sp.x)|| _isnan(sp.y) || _isnan(sp.z))
			continue;
		IPt cur_index=get_index_point(sp);
		if(!tmpHash.find(cur_index)) // new cell point
		{
			out_indices.push_back(i); // record this index in the original PC
			tmpHash.insert(cur_index);
			sparse->points.push_back(sp);
		}
	}
}