#include "TagFinder.h"
#include "color_table.h"
#include <opencv2/imgproc/imgproc.hpp>

CTagFinder::CTagFinder(){
	m_IsReady=false; 
}
CTagFinder::~CTagFinder(){}

// ��ȡͼ���ļ�
void CTagFinder::getImage(string file_name)
{
	m_image = cv::imread(file_name);
	if(m_image.empty()){
		cout<<"failed to open file: "<<file_name<<endl;
		m_IsReady=false;
		return;
	}
	//calTagLoc();
	m_IsReady=true;
}
// �������
void CTagFinder::clear()
{
	m_image.deallocate();
	m_IsReady=false;
}

void CTagFinder::SobelFindEdge()
{
	cv::Mat tmpimage;
	//m_image.convertTo(tmpimage,CV_8UC1);

	// Compute Sobel X derivative
	cv::Mat sobelX;
	cv::Sobel(m_image,sobelX,CV_8U,1,0,3,0.4,128);

	// Compute Sobel Y derivative
	cv::Mat sobelY;
	cv::Sobel(m_image,sobelY,CV_8U,0,1,3,0.4,128);

	for(int row=0;row<sobelX.rows;row++)
		for(int j=0;j<sobelX.cols;j++)
		{
			int col = j*3;
			if(sobelX.at<unsigned char>(row,col)>160 || \
				sobelY.at<unsigned char>(row,col)>160)
			{
				m_image.at<unsigned char>(row,col) = blue().r;
				m_image.at<unsigned char>(row,col+1)=blue().g;
				m_image.at<unsigned char>(row,col+2)=blue().b;
			}
		}
		cv::imshow("Sobel ",m_image);
		cv::waitKey();
}

void CTagFinder::FindValidEdge(vector<vector<bool> >& edge_flag)
{
	static const int edge_length_thre=15;

	map<int, pair<int,int> > hor_edge;
	map<int, pair<int,int> > ver_edge;

	vector<int> hor_edge_len;
	vector<int> ver_edge_len;

	vector<int> hor_edge_row;
	vector<int> ver_edge_col;


	// �ҵ�ÿһ��ˮƽ�ı�
	for(int i=0;i<edge_flag.size();i++)
	{
		for(int fj=0;fj<edge_flag[i].size();)
		{
			if(!edge_flag[i][fj]){
				fj++;
				continue;
			}
			int lj=fj+1;
			for(;lj<edge_flag[i].size()-2;)
			{
				if(  edge_flag[i][lj] || edge_flag[i][lj+1] || edge_flag[i][lj+2])
				{
					lj++;
					continue;
				}
				break;
			}
			if(lj-fj>=edge_length_thre)
			{
				hor_edge_len.push_back(lj-fj);			// ��¼����ˮƽ�ߵĳ���
				hor_edge_row.push_back(i);				// ��¼����ˮƽ�ߵ���������
				hor_edge.insert(make_pair(hor_edge_len.size()-1 \
					,make_pair(fj,lj)));				//��¼����ˮƽ�߿�Խ������
				for(int k=fj;k<=lj;k++)
				{
					int col = k*3;
					m_image.at<unsigned char>(i,col) = green().r;
					m_image.at<unsigned char>(i,col+1) = green().g;
					m_image.at<unsigned char>(i,col+2) = green().b;
				}
			}
			fj=lj+1;
		}
	}
	// �ҵ�ÿһ����ֱ�ı�
	for(int j=0;j<edge_flag[0].size();j++)
	{
		for(int fi=0;fi<edge_flag.size();)
		{
			if(!edge_flag[fi][j]){
				fi++;
				continue;
			}
			int li=fi+1;
			for(;li<edge_flag.size()-2;)
			{
				if(  edge_flag[li][j] || edge_flag[li+1][j] || edge_flag[li+2][j])
				{
					li++;
					continue;
				}
				break;
			}
			if(li-fi>=edge_length_thre)
			{
				ver_edge_len.push_back(li-fi);			// ��¼������ֱ�ߵĳ���
				ver_edge_col.push_back(j);				// ��¼������ֱ�ߵ�����
				ver_edge.insert(make_pair(ver_edge_len.size()-1\
					,make_pair(fi,li)));				// ��¼������ֱ�߿�Խ������
				for(int k=fi;k<=li;k++)
				{
					int col = j*3;
					m_image.at<unsigned char>(k,col) = green().r;
					m_image.at<unsigned char>(k,col+1) = green().g;
					m_image.at<unsigned char>(k,col+2) = green().b;
				}
			}
			fi=li+1;
		}
	}

	if(hor_edge.size() <=0 || ver_edge.size()<=0)
	{
		cout<<"No edge is extracted!"<<endl;
		return ;
	}

	// �ҵ�����������ı�
	map<int, pair<int,int> >::iterator index_hor,index_ver;
	int min_len=10000;
	for(map<int, pair<int,int> >::iterator it_hor=hor_edge.begin(); \
		it_hor!=hor_edge.end();it_hor++)
	{
		for(map<int,pair<int,int> >::iterator it_ver=ver_edge.begin();\
			it_ver!=ver_edge.end();it_ver++)
		{
			int dis = abs(hor_edge_len[it_hor->first] - ver_edge_len[it_ver->first]);
			if(dis<min_len)				// �������
			{
				int hor_row = hor_edge_row[it_hor->first];
				int ver_col = ver_edge_col[it_ver->first];
				if(hor_row>= it_ver->second.first && hor_row<=it_ver->second.second \
					&& ver_col>=it_hor->second.first && ver_col<=it_hor->second.second)	// �����н���
				{	
					index_hor=it_hor;
					index_ver=it_ver;
					min_len=dis;
				}
			}
		}
	}
	// ����Ա߶�Ӧ�������ע����
	for(int row=index_ver->second.first;row<=index_ver->second.second;row++)
	{
		for(int j=index_hor->second.first;j<=index_hor->second.second;j++)
		{
			int col = j*3;
			m_image.at<unsigned char>(row,col) = blue().r;
			m_image.at<unsigned char>(row,col+1) = blue().g;
			m_image.at<unsigned char>(row,col+2) =blue().b;
		}
	}
	// bounding box has to be specified
	m_l=index_hor->second.first;
	m_r=index_hor->second.second;
	m_u=index_ver->second.first;
	m_d=index_ver->second.second;
}

void CTagFinder::FindEdge()
{
	// ��������������˹���ӣ���ȡĿ��ı���
	// Compute Laplacian 3x3
	cv::Mat laplace;
	cv::Laplacian(m_image,laplace,CV_8U,1,1,128);

	// ��ʼ������ָʾֵ
	vector<vector<bool> > edge_flag;
	edge_flag.resize(480);
	for(int i=0;i<480;i++)
		edge_flag[i].resize(640,false);

	for(int row=0;row<laplace.rows;row++)
	{
		for(int j=0;j<laplace.cols;j++)
		{
			int col=j*3;
			unsigned char lap = laplace.at<unsigned char>(row,col);
			if(lap>140) //����
			{
				m_image.at<unsigned char>(row,col) = red().r;
				m_image.at<unsigned char>(row,col+1) = red().g;
				m_image.at<unsigned char>(row,col+2) =red().b;
				
				// ����������ص�
				edge_flag[row][j]=true;
			}
		}
	}
	
	FindValidEdge(edge_flag);

	cv::imshow("Laplace ",laplace);
	cv::imshow("Edge",m_image);
	cv::waitKey();
}

void CTagFinder::MidFilter()
{
	cv::Mat tmp;
	m_image.copyTo(tmp);
	cv::medianBlur(tmp,m_image,5);
	cv::imshow("medianBlur",m_image);
	cv::waitKey();
}


// ����tag���ڵ����ؿ�
void CTagFinder::calTagLoc()
{

	// �ھ�������range��Χ��Ѱ����ڵĵ�
	int xCenter,yCenter;
	xCenter = m_image.cols>>1;
	yCenter = m_image.rows>>1;
	int range = 100;

	// ��С��r,g,b
	unsigned int min_c = -1;
	unsigned char r,g,b;
	set<unsigned int> min_set;

	for(int i=-range; i<=range;i++)
	{
		for(int j=-range;j<=range;j++)
		{
			int row = yCenter+i;
			int col = (xCenter+j)*3;
			r = m_image.at<unsigned char>(row,col);
			g = m_image.at<unsigned char>(row,col+1);
			b = m_image.at<unsigned char>(row,col+2);

			unsigned int score = r * g * b;
			if(1500>=score)
			{
				min_set.insert(score);
			}
		}
	}

	// ����ǰ�漸����Խ�С��ֵ
	/*set<unsigned int>::iterator it_ = min_set.begin();
	while(it_!=min_set.end())
	{
		if(*it_>1500)
		{
			break;
		}
		it_++;
	}
	min_set.erase(it_,min_set.end());*/

	vector<std::pair<int,int> >  location;

	// ͼ���к�ɫ�����Ϊtag���򣬱���������ڵ�����
	for(int i=0;i<m_image.rows;i++)
	{
		for(int j=0;j<m_image.cols;j++)
		{
			int row = i;
			int col = j*3;
			r = m_image.at<unsigned char>(row,col);
			g = m_image.at<unsigned char>(row,col+1);
			b = m_image.at<unsigned char>(row,col+2);
			unsigned int score = r*g*b;
			if(min_set.find(score)==min_set.end())
				continue;

			location.push_back(make_pair(row,col));
			
		}
	}

	cout<<"find tag size: "<<location.size()<<endl;

	// ����Щ����ʾ�ɺ�ɫ
	for(int i=0;i<location.size();i++)
	{
		int row = location[i].first;
		int col = location[i].second;
		m_image.at<unsigned char>(row,col) = red().r;
		m_image.at<unsigned char>(row,col+1) = red().g;
		m_image.at<unsigned char>(row,col+2) =red().b;
	}
	// ��ʾ����ͼ
	cv::imshow( "after finding tag", m_image);  
	cvWaitKey(); 

}

// ��ʾ����tag����
void CTagFinder::showTagLoc(int l,int r,int u,int d)
{

}