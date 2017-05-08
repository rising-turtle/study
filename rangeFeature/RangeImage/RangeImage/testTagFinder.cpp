#include "TagFinder.h"

void testTagFinder(string file_name)
{
	CTagFinder tmpTF;
	tmpTF.getImage(file_name);
	//tmpTF.SobelFindEdge();
	tmpTF.FindEdge();
//	tmpTF.MidFilter();
//	tmpTF.calTagLoc();
}

void testTagFinderAll(string file_dir)
{
	CTagFinder tmpTF;
	for(int i=1;i<=10;i++){
		stringstream file_s;
		string file_n;
		file_s<<file_dir<<"i"<<i<<".png";
		file_s>>file_n;
		tmpTF.getImage(file_n);
		tmpTF.FindEdge();
		cout<<"read file: "<<i<<".png"<<endl;
		getchar();
	}
}

// �������range�ľ�ֵ�ͷ�����Ұ�ÿ���㶼��¼���ļ���
void recordMeanVari(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_pc,string file_out)
{
	int n_total = m_pc->points.size();
	// �����ֵ
	float range_z = 0;
	for(int i=0;i<m_pc->points.size();i++)
	{
		pcl::PointXYZRGB& pc = m_pc->points[i];
		range_z+=pc.z;
	}
	float mean_z = range_z/(float)n_total;
	// ���㷽��
	double variable = 0;
	for(int i=0;i<m_pc->points.size();i++)
	{
		pcl::PointXYZRGB& pc = m_pc->points[i];
		variable += (pc.z-mean_z)*(pc.z-mean_z);
	}
	variable = variable/(double)n_total;
	variable = sqrt(variable);

	// д���ļ���
	ofstream outf(file_out.c_str());
	if(outf.is_open())
	{
		outf<<"N: "<<n_total<<endl;
		outf<<"Mean: "<<mean_z<<endl;
		outf<<"Vari: "<<variable<<endl;
		for(int i=0;i<m_pc->points.size();i++)
		{
			pcl::PointXYZRGB& pt = m_pc->points[i];
			outf<<pt.x<<","<<pt.y<<","<<pt.z<<endl;
		}
		outf.close();
	}
}

// ������ץȡ��image �� depth�Ƿ�ƥ��
void testTagLocation()
{
	string file_dir("D:\\MyProjects\\KinectDeviation\\cornerloc\\exper2\\");
	CTagFinder tmpTF;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tagPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i=21;i<=34;i++)
	{
		// ��ȡ�ļ���
		stringstream imfs,defs;
		imfs<<file_dir<<"i"<<i<<".png";
		defs<<file_dir<<"d"<<i<<".png";

		string imf,def;
		imfs>>imf;
		defs>>def;

		// ��ȡ����
		cv::Mat image;
		cv::Mat depth;
		tmpTF.getImage(imf);
		tmpTF.m_image.copyTo(image);
		depth=cv::imread(def,2);
		
		//cv::imshow("tmp",image);

		tmpTF.FindEdge();

		cout<<"read "<<i<<".png"<<endl;
		cout<<"bounding box: "<<endl;
		cout<<tmpTF.m_l<<"-"<<tmpTF.m_r<<endl;
		cout<<tmpTF.m_u<<"-"<<tmpTF.m_d<<endl;
		// �ϳɵ�������
		calPCfromImageAndDepth(image,depth,m_tmpPC);
		
		for(int row=tmpTF.m_u;row<=tmpTF.m_d;row++)
			for(int col=tmpTF.m_l;col<=tmpTF.m_r;col++)
			{
				pcl::PointXYZRGB p3d = (*m_tmpPC).at(col,row);
				if ( _isnan(p3d.x) || _isnan(p3d.y) || _isnan(p3d.z)){
					continue;
				}
				m_tagPC->points.push_back(p3d);
			}
			cout<<"number of points: "<<m_tagPC->points.size()<<endl;
		
		stringstream recordfile;
		recordfile<<file_dir<<i<<".txt";
		string record_f;
		recordfile>>record_f;
		recordMeanVari(m_tagPC,record_f);
		DisplayPT(m_tagPC);
		getchar();
		m_tagPC->points.clear();
	}
}

// ����April�ṩ��λ�������ҵ���Ӧ��
void testAprilSpecified()
{
	string file_dir("D:\\MyProjects\\KinectDeviation\\cornerloc\\exper2\\");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tagPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i=31;i<=34;i++)
	{
		// ��ȡ�ļ���
		stringstream imfs,defs;
		imfs<<file_dir<<"i"<<i<<".png";
		defs<<file_dir<<"d"<<i<<".png";

		string imf,def;
		imfs>>imf;
		defs>>def;

		// ��ȡ����
		cv::Mat image;
		cv::Mat depth;
		image=cv::imread(imf);
		depth=cv::imread(def,2);

		// �ϳɵ�������
		calPCfromImageAndDepth(image,depth,m_tmpPC);

		// ��ȡApril�����ļ���
		stringstream aprilfile_s;
		aprilfile_s<<file_dir<<"i"<<i<<".txt";
		string aprilfile;
		aprilfile_s>>aprilfile;

		// �ҵ���Ӧ����ĵ�����Ϣ
		char buf[100];
		ifstream inf(aprilfile.c_str());
		float fx1,fy1,fx2,fy2,fx3,fy3,fx4,fy4;
		inf.getline(buf,100);
		sscanf(buf,"p0: %f,%f p1: %f,%f",&fx1,&fy1,&fx2,&fy2);
		inf.getline(buf,100);
		sscanf(buf,"p2: %f,%f p3: %f,%f",&fx3,&fy3,&fx4,&fy4);
		
		for(int row=(int)fy1;row<=(int)fy3;row++)
			for(int col=(int)fx1;col<=(int)fx3;col++)
			{
				pcl::PointXYZRGB p3d = (*m_tmpPC).at(col,row);
				if ( _isnan(p3d.x) || _isnan(p3d.y) || _isnan(p3d.z)){
					continue;
				}
				m_tagPC->points.push_back(p3d);
				int rcol = col*3;
				image.at<unsigned char>(row,rcol)=blue().r;
				image.at<unsigned char>(row,rcol+1)=blue().g;
				image.at<unsigned char>(row,rcol+2)=blue().b;
			}

			stringstream recordfile;
			recordfile<<file_dir<<i<<".txt";
			string record_f;
			recordfile>>record_f;
			recordMeanVari(m_tagPC,record_f);

		/*	DisplayPT(m_tagPC);
			cv::imshow("tag area",image);
			cv::waitKey();*/
			m_tagPC->points.clear();
	}
}

void testVariableRefine()
{
	string file_dir("D:\\MyProjects\\KinectDeviation\\cornerloc\\exper2\\");
	stringstream file_rec;
	file_rec<<file_dir<<"record.txt";
	string file_r;
	file_rec>>file_r;
	ofstream file_out(file_r.c_str());

	for(int i=30;i<31;i++)
	{
		// ���ļ�
		stringstream file_s;
		file_s<<file_dir<<i<<".txt";
		string file_in;
		file_s>>file_in;
		ifstream inf(file_in.c_str());
		
		if(inf.is_open()){
			
			// ��ȡ���Ƹ������ֵ
			int total_n;
			float mean_z;
			char buf[100];
			inf.getline(buf,100);
			sscanf(buf,"N: %d",&total_n);
			inf.getline(buf,100);
			sscanf(buf,"Mean: %f",&mean_z);
			inf.getline(buf,100);

			int cout=0;
			float lx,ly,lz;
			double variable=0;
			while(inf.getline(buf,100))
			{
				sscanf(buf,"%f,%f,%f",&lx,&ly,&lz);
				if(fabs(lz-mean_z)<0.2) // ���ƫ���ֵ��������Ϊ��������
				{
					cout++;
					variable+=(lz-mean_z)*(lz-mean_z);
				}
			}
			variable/=(double)(cout);
			variable=sqrt(variable);

			file_out<<i<<".txt: "<<"M: "<<mean_z<<","<<"V: "<<variable<<endl;
			inf.close();
		}
	}
	file_out.close();
	cout<<"succeed!"<<endl;
}
