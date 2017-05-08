#include "preheader.h"
#include "globaldefinition.h"
#include "SynchroniGroundTruth.h"

void testGroundTruth()
{
	unsigned char* m_pucRGB=new unsigned char[640*480*3];
	unsigned short* m_pusDepth=new unsigned short[640*480];
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	CSynchroniGroundTruth tmpGT;
	int count=0;
	while(count<10){
		if(tmpGT.Run(m_pucRGB,m_pusDepth,m_tmpPC)!=-1){
			cout<<"display PC "<<count<<endl;
			//DisplayPT(m_tmpPC);
		}else{
			cout<<"failed to read PC "<<count<<endl;
		}
		count++;
	}	
	delete []m_pucRGB;
	delete []m_pusDepth;
}