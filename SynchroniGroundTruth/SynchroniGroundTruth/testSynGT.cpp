#include "preheader.h"
#include "SynchroniGroundTruth.h"

void testSynGT()
{
	string file1("D:\\MyProjects\\SynchroniGroundTruth\\depth.txt");
	vector<double> timestamp;
	vector<string> file_name;
	CSynchroniGroundTruth tmpCGT;
	tmpCGT.readfile(file1,timestamp,file_name);

	for(int i=0;i<50;i++)
	{
		cout<<timestamp[i]<<" "<<file_name[i]<<endl;
	}
}

void testSynGT1()
{
	string depthf("D:\\MyProjects\\SynchroniGroundTruth\\depth.txt");
	string rgbf("D:\\MyProjects\\SynchroniGroundTruth\\rgb.txt");
	string gtf("D:\\MyProjects\\SynchroniGroundTruth\\groundtruth.txt");
	CSynchroniGroundTruth tmpCGT;
	//tmpCGT.findSynchroniMatch(depthf,rgbf,gtf);
	tmpCGT.findAllSynMatch(depthf,rgbf,gtf);

	cout<<"succeed!"<<endl;
}