
/**
	ֻ����ʱ����ض���tag���г�ȡ���������ǵ������ļ��У�tag
	������ͼ�������
**/
#ifndef TAG_FINDER_H
#define TAG_FINDER_H

#include "preheader.h"
#include "globaldefinition.h"
#include "opencv2/core/core.hpp"  
#include "opencv2/highgui/highgui.hpp" 

class CTagFinder
{
public:
	CTagFinder();
	~CTagFinder();

	cv::Mat m_image;
	int m_l,m_r,m_u,m_d;
	bool m_IsReady;
	
	// ��ȡͼ���ļ�
	void getImage(string file_name);
	// ����tag���ڵ����ؿ�
	void calTagLoc();
	// ��ʾ����tag����
	void showTagLoc(int l,int r,int u,int d);
	// ��ֵ�˲�
	void MidFilter();

	// ����������˹�ұ���
	void FindEdge();
	void FindValidEdge(vector<vector<bool> >&);

	// ����Sobel�����ұ���
	void SobelFindEdge();

	// �������
	void clear();

protected:
private:
};

#endif