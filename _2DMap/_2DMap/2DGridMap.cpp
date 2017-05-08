#include "2DGridMap.h"
#include <string>
#include <time.h>


int _2DGridMap::Init2DGridMap()
{
	memset(m_cOccupyFlag,0,GRID_MAP_NUM);
	return 0;
}

int _2DGridMap::AddNewMap(float fScale,int nImgW,int nImgH,unsigned char *pucMap,char *pcMapName,float fGridResolution)
{
	int i;
	 clock_t start, finish;

	float fW=fScale*nImgW;
	float fH=fScale*nImgH;

	int nW=(int)(fW/fGridResolution+0.5);
	int nH=(int)(fH/fGridResolution+0.5);
	nW+=(nW%4);

	int nSize=nW*nH;

	for (i=0;i<GRID_MAP_NUM;i++)
	{
		if (m_cOccupyFlag[i]==0)
		{
			m_pst2DGridMapData[i]=new _2DGridMapData;

			if(strlen(pcMapName)<32)memcpy(m_pst2DGridMapData[i]->cMapName,pcMapName,strlen(pcMapName));
			else memcpy(m_pst2DGridMapData[i]->cMapName,pcMapName,32);
			
			m_pst2DGridMapData[i]->fGridResolution=fGridResolution;
			m_pst2DGridMapData[i]->n2DGridMapIdx=0;
			m_pst2DGridMapData[i]->pstSub2DGridMapData=NULL;
			m_pst2DGridMapData[i]->n2DGridMapDataLen=nSize;
			m_pst2DGridMapData[i]->nMapW=nImgW;
			m_pst2DGridMapData[i]->nMapH=nImgH;
			m_pst2DGridMapData[i]->fMapScale=fScale;
			m_pst2DGridMapData[i]->fMapActualW=fW;
			m_pst2DGridMapData[i]->fMapActualH=fH;
			m_pst2DGridMapData[i]->n2DGridMapW=nW;
			m_pst2DGridMapData[i]->n2DGridMapH=nH;
			m_pst2DGridMapData[i]->puc2DGridMapData=new unsigned char[m_pst2DGridMapData[i]->n2DGridMapDataLen];
			m_pst2DGridMapData[i]->pucAccWeight=new unsigned char [m_pst2DGridMapData[i]->n2DGridMapDataLen];

			memset(m_pst2DGridMapData[i]->puc2DGridMapData,0,m_pst2DGridMapData[i]->n2DGridMapDataLen);
	
			Convert22DGridMap(pucMap,m_pst2DGridMapData[i]);
			m_cOccupyFlag[i]=1;
			break;
		}
	}


	//m_pucAccWeight=new unsigned char [m_pst2DGridMapData[i]->n2DGridMapDataLen];
	//memset(m_pucAccWeight,0,m_pst2DGridMapData[i]->n2DGridMapDataLen);


	SeedGrowing(42432,m_pst2DGridMapData[i]);
	MergeGridsBasedOnRobotSize(0,m_pst2DGridMapData[i]);
	UpdateAccWeightMap(m_pst2DGridMapData[i]->pucAccWeight,m_pst2DGridMapData[i]);

	start = clock();
	GetOptimunPath(42432,49459,m_pst2DGridMapData[0]);

//	GetShortestPath(14032,90109,m_pst2DGridMapData[0]);


	m_pst2DGridMapData[0]->puc2DGridMapData[14032]=211;
	m_pst2DGridMapData[0]->puc2DGridMapData[90109]=212;
	finish=clock();
	printf("start time  :%d   finish  time : %d   cost time:  %d  \n",finish-start,start,finish);
	return i;
}

int _2DGridMap::Convert22DGridMap(unsigned char *pucMap,_2DGridMapData *pst2DGridMapData)
{
	int i,j,idx=0;
	int x,y;
	float fRatio=pst2DGridMapData->fMapScale/pst2DGridMapData->fGridResolution;
	
	for (i=0;i<pst2DGridMapData->nMapW;i++)
	{
		for (j=0;j<pst2DGridMapData->nMapH;j++)
		{
			idx=j*pst2DGridMapData->nMapW+i;
			if (pucMap[idx*3]<200)
			{
				x=(int)((float)i*fRatio);
				y=(int)((float)j*fRatio);

				pst2DGridMapData->puc2DGridMapData[y*pst2DGridMapData->n2DGridMapW+x]++;

			}
		}
	}
	return 0;
}


int _2DGridMap::SeedGrowing(int nStartPosIdx,_2DGridMapData *pst2DGridMapData)
{
	vector<int> vctTmp;
	int nTmpIdx,i,nNewIdx;
	vctTmp.push_back(nStartPosIdx);

	int nMask[4]={-1,1,pst2DGridMapData->n2DGridMapW,-pst2DGridMapData->n2DGridMapW};

	while (vctTmp.size()!=0)
	{
		nTmpIdx=vctTmp.back();
		vctTmp.pop_back();
		pst2DGridMapData->puc2DGridMapData[nTmpIdx]=200;

		for (i=0;i<4;i++)
		{
			nNewIdx=nTmpIdx+nMask[i];

			if(nNewIdx>=0&&nNewIdx<pst2DGridMapData->n2DGridMapDataLen)
			{
				if (pst2DGridMapData->puc2DGridMapData[nNewIdx]==0)
				{
					vctTmp.push_back(nNewIdx);
				}
			}

		}
	}
	return 0;
}

int _2DGridMap::GetShortestPath(int nStartPosIdx, int nEndPosIdx,_2DGridMapData *pst2DGridMapData)
{
	vector<int> vctPath;
	RemarkWeightMap(nStartPosIdx,nEndPosIdx,pst2DGridMapData,pst2DGridMapData->pucAccWeight,vctPath);
	return 0;
}

int _2DGridMap::RemarkWeightMap(int nStartPosIdx,int nEndPosIdx,_2DGridMapData *pst2DGridMapData,unsigned char *pucAccWeightMap,vector<int> &vctPath)
{
	int *pnWeightMap=new int[pst2DGridMapData->n2DGridMapDataLen];
	unsigned char *pucTmpGridMap=new unsigned char[pst2DGridMapData->n2DGridMapDataLen];
	vector<int> vctTmp,vctTmp2;
	int nSeedWeight,nTmpIdx,i,nNewIdx,nNewWeight,nKeepIdx;
	bool bJump=false;

	memset(pnWeightMap,0,pst2DGridMapData->n2DGridMapDataLen*4);
	memcpy(pucTmpGridMap,pst2DGridMapData->puc2DGridMapData,pst2DGridMapData->n2DGridMapDataLen);
	vctTmp.push_back(nStartPosIdx);
	int nMask[4]={-1,1,pst2DGridMapData->n2DGridMapW,-pst2DGridMapData->n2DGridMapW};
	int nMask2[8]={-1,1,
					pst2DGridMapData->n2DGridMapW-1,pst2DGridMapData->n2DGridMapW,pst2DGridMapData->n2DGridMapW+1,
					-pst2DGridMapData->n2DGridMapW-1,-pst2DGridMapData->n2DGridMapW,-pst2DGridMapData->n2DGridMapW+1
					};
	int nMinWeight=999999;
	pnWeightMap[nStartPosIdx]=1;
	
	while (!bJump)
	{
		while (vctTmp.size()!=0)
		{
			nTmpIdx=vctTmp.back();
			vctTmp.pop_back();
			pucTmpGridMap[nTmpIdx]=202;

			nSeedWeight=pnWeightMap[nTmpIdx];
			nNewWeight=nSeedWeight+1;

			if(nTmpIdx!=nEndPosIdx)
			{
				for (i=0;i<4;i++)
				{
					nNewIdx=nTmpIdx+nMask[i];

					if(nNewIdx>=0&&nNewIdx<pst2DGridMapData->n2DGridMapDataLen)
					{
						if (pucTmpGridMap[nNewIdx]==200&&pnWeightMap[nNewIdx]<nNewWeight)
						{
							vctTmp2.push_back(nNewIdx);
							pnWeightMap[nNewIdx]=nNewWeight;
						}
					}

				}
			}
			else
			{
				vctTmp.clear();
				bJump=true;
			}
		}

		if (vctTmp2.size()==0)
		{
			bJump=true;
		}
		if (!bJump)
		{
			vctTmp=vctTmp2;
			vctTmp2.clear();
		}

		
	}
	
	vctTmp.push_back(nEndPosIdx);
	vctPath.push_back(nEndPosIdx);
	while (vctTmp.size()!=0)
	{
		nTmpIdx=vctTmp.back();
		vctTmp.pop_back();
		if (nTmpIdx!=nStartPosIdx)
		{
			 nMinWeight=999999;
			for (i=0;i<8;i++)
			{
				nNewIdx=nTmpIdx+nMask2[i];
				if (pnWeightMap[nNewIdx]<nMinWeight&&pnWeightMap[nNewIdx]!=0)
				{
					nMinWeight=pnWeightMap[nNewIdx];
					nKeepIdx=nNewIdx;
				}
			}
			vctTmp.push_back(nKeepIdx);
			//pst2DGridMapData->puc2DGridMapData[nKeepIdx]=210;
			vctPath.push_back(nKeepIdx);
		}
		else
		{
			vctTmp.clear();
		}
	}

	delete [] pnWeightMap;
	delete [] pucTmpGridMap;


	return 0;
}


int _2DGridMap::MergeGridsBasedOnRobotSize(int nRobotSize,_2DGridMapData *pst2DGridMapData)
{
	int nMask[25],i,j,idx=0;
	unsigned char *pucTmpGridMap=new unsigned char[pst2DGridMapData->n2DGridMapDataLen];
	memcpy(pucTmpGridMap,pst2DGridMapData->puc2DGridMapData,pst2DGridMapData->n2DGridMapDataLen);

	for (i=-2;i<=2;i++)
	{
		for (j=-2;j<=2;j++)
		{
			nMask[idx]=j*pst2DGridMapData->n2DGridMapW+i;
			idx++;
		}
	}

	for (i=0;i<pst2DGridMapData->n2DGridMapDataLen;i++)
	{
		if(pst2DGridMapData->puc2DGridMapData[i]==200)
		{
			for (j=0;j<25;j++)
			{
				idx=i+nMask[j];
				if (idx>=0&&idx<pst2DGridMapData->n2DGridMapDataLen)
				{
					if (pucTmpGridMap[idx]!=200)
					{
						pst2DGridMapData->puc2DGridMapData[i]=255;
						break;
					}
				}
			}
		}
	}


	delete [] pucTmpGridMap;
	return 0;
}

int _2DGridMap::UpdateAccWeightMap(unsigned char *pucAccWeightMap,_2DGridMapData *pst2DGridMapData)
{
	int i,j,idx,nAccWeight;
	int nMask[25];
	idx=0;
	for (i=-2;i<=2;i++)
	{
		for (j=-2;j<=2;j++)
		{
			nMask[idx]=j*pst2DGridMapData->n2DGridMapW+i;
			idx++;
		}
	}
	for (i=0;i<pst2DGridMapData->n2DGridMapDataLen;i++)
	{
		if(pst2DGridMapData->puc2DGridMapData[i]==200)
		{
			nAccWeight=0;
			for (j=0;j<25;j++)
			{
				idx=i+nMask[j];
				if (idx>=0&&idx<pst2DGridMapData->n2DGridMapDataLen)
				{
					if (pst2DGridMapData->puc2DGridMapData[idx]!=200)
					{
						nAccWeight++;
					}
				}
			}
			pucAccWeightMap[i]=nAccWeight;
		}
		
	}
	return 0;
}

int _2DGridMap::AStar(int nStartPosIdx,int nEndPosIdx,
					  _2DGridMapData *pst2DGridMapData,
					  unsigned char *pucAccWeightMap,
					  int *pnParentMap,vector<int> &vctPath)
{
	bool bJump=false;
	int nMinWeightIdx=nStartPosIdx;
	vector<int> vctTmp;
	int i,nNewIdx,nCurWeight,nTmpWeight,nTmpIdx,nParentIdx;


	int *pnWeightMap=new int[pst2DGridMapData->n2DGridMapDataLen];
	unsigned char *pucTmpGridMap=new unsigned char[pst2DGridMapData->n2DGridMapDataLen];


	int nMask[4]={-1,1,pst2DGridMapData->n2DGridMapW,-pst2DGridMapData->n2DGridMapW};

	memset(pnWeightMap,0,pst2DGridMapData->n2DGridMapDataLen*4);
	memcpy(pucTmpGridMap,pst2DGridMapData->puc2DGridMapData,pst2DGridMapData->n2DGridMapDataLen);

	vctTmp.push_back(nStartPosIdx);
	pnWeightMap[nStartPosIdx]=1;

	while (!bJump)
	{
		nTmpIdx=GetMinWeightIdx(vctTmp,pnWeightMap);
		nMinWeightIdx=vctTmp[nTmpIdx];
		vctTmp.erase(vctTmp.begin()+nTmpIdx);
		if (nMinWeightIdx!=nEndPosIdx)
		{
			pucTmpGridMap[nMinWeightIdx]=202;
			nCurWeight=pnWeightMap[nMinWeightIdx];

			for (i=0;i<4;i++)
			{
				nNewIdx=nMinWeightIdx+nMask[i];
				if (nNewIdx>=0&&nNewIdx<pst2DGridMapData->n2DGridMapDataLen)
				{
					if (pucTmpGridMap[nNewIdx]==200)
					{
						if (pnParentMap[nNewIdx]==0)
						{
							pnWeightMap[nNewIdx]=nCurWeight+1+pucAccWeightMap[nNewIdx];
							pnParentMap[nNewIdx]=nMinWeightIdx;
							vctTmp.push_back(nNewIdx);
						}
						else
						{
							nTmpWeight=nCurWeight+1+pucAccWeightMap[nNewIdx];

							if(nTmpWeight<pnWeightMap[nNewIdx])
							{
								pnWeightMap[nNewIdx]=nTmpWeight;
								pnParentMap[nNewIdx]=nMinWeightIdx;
							}
						}
					}
				}
			}
		}
		else
		{
			bJump=true;
			vctTmp.clear();
		}
	}

	nParentIdx=nEndPosIdx;
	while (nParentIdx!=nStartPosIdx)
	{
		vctPath.push_back(nParentIdx);
		nParentIdx=pnParentMap[nParentIdx];
		pst2DGridMapData->puc2DGridMapData[nParentIdx]=210;
	}

	delete [] pnWeightMap;
	delete [] pucTmpGridMap;
	return 0;
}

int _2DGridMap::GetOptimunPath(int nStartPosIdx,int nEndPosIdx,_2DGridMapData *pst2DGridMapData)
{
	vector<int> vctPath;
	int *pnParentMap=new int [pst2DGridMapData->n2DGridMapDataLen];
	memset(pnParentMap,0,pst2DGridMapData->n2DGridMapDataLen*4);
	AStar(nStartPosIdx,nEndPosIdx,pst2DGridMapData,pst2DGridMapData->pucAccWeight,pnParentMap,vctPath);
	delete [] pnParentMap;
	pnParentMap=NULL;
	return 0;
}

int _2DGridMap::GetMinWeightIdx(vector<int> vctIdx,int *pnWeightMap)
{
	int nMinValue=9999999,i,nMinIdx;
	for (i=0;i<vctIdx.size();i++)
	{
		if (pnWeightMap[vctIdx[i]]<nMinValue)
		{
			nMinValue=pnWeightMap[vctIdx[i]];
			nMinIdx=i;
		}
	}
	return nMinIdx;
}

int _2DGridMap::GetShortestPath(int nStartPosIdx,int nEndPosIdx,int n2DGridMapIdx,vector<int> &vctPath)
{
	int *pnParentMap=new int [m_pst2DGridMapData[n2DGridMapIdx]->n2DGridMapDataLen];
	memset(pnParentMap,0,m_pst2DGridMapData[n2DGridMapIdx]->n2DGridMapDataLen*4);
	AStar(nStartPosIdx,nEndPosIdx,
			m_pst2DGridMapData[n2DGridMapIdx],
			m_pst2DGridMapData[n2DGridMapIdx]->pucAccWeight,
			pnParentMap,vctPath);

	delete [] pnParentMap;
	pnParentMap=NULL;
	return 0;
}
int _2DGridMap::GetOptimunPath(int nStartPosIdx,int nEndPosIdx,int n2DGridMapIdx,vector<int> &vctPath)
{

	RemarkWeightMap(nStartPosIdx,nEndPosIdx,m_pst2DGridMapData[n2DGridMapIdx],
		m_pst2DGridMapData[n2DGridMapIdx]->pucAccWeight,vctPath);
	return 0;
}