#include "2DGridMap.h"
#include "BMPLib.h"

int main()
{
	int nW,nH,i,j,idx;
	unsigned char *pucImg=NULL;
	_2DGridMap test;
	BMPLib CBMP;
	pucImg=CBMP.BMPLoad("B2Map.bmp",pucImg,&nW,&nH);

	for (i=0;i<nW/2;i++)
	{
		pucImg[((nH-500)*nW+i)*3]=255;
		pucImg[((nH-500)*nW+i)*3+1]=0;
		pucImg[((nH-500)*nW+i)*3+2]=0;
	}


//	memset(pucImg+nW*20*3,0,nW*50*3);
	CBMP.BMPSave("testSave1.bmp",pucImg,nW,nH);
	

	test.Init2DGridMap();
	test.AddNewMap(20,nW,nH,pucImg,"song",100);


	nW=test.m_pst2DGridMapData[0]->n2DGridMapW;
	nH=test.m_pst2DGridMapData[0]->n2DGridMapH;

	//nW=111;
	//nH=341;
	unsigned char *pucSaveImg=new unsigned char [nW*nH*3];
	memset(pucSaveImg,255,nW*nH*3);
	idx=0;

//	memset(pucSaveImg+nW*20*3,0,nW*50*3);
	for (j=0;j<nH;j++)
	{
		for (i=0;i<nW;i++)
		{
			idx=j*nW+i;
			if(test.m_pst2DGridMapData[0]->puc2DGridMapData[idx]==0)
			{
			//	pucSaveImg[idx*3]=255;
			//	pucSaveImg[idx*3+1]=255;
			//	pucSaveImg[idx*3+2]=255;
			}
			else if (test.m_pst2DGridMapData[0]->puc2DGridMapData[idx]==200)
			{
				pucSaveImg[idx*3]=0;
				pucSaveImg[idx*3+1]=150;
				pucSaveImg[idx*3+2]=0;
			}
			else if (test.m_pst2DGridMapData[0]->puc2DGridMapData[idx]==210)
			{
				pucSaveImg[idx*3]=255;
				pucSaveImg[idx*3+1]=0;
				pucSaveImg[idx*3+2]=0;
			}
			else if (test.m_pst2DGridMapData[0]->puc2DGridMapData[idx]==211)
			{
				pucSaveImg[idx*3]=255;
				pucSaveImg[idx*3+1]=0;
				pucSaveImg[idx*3+2]=255;
			}else if (test.m_pst2DGridMapData[0]->puc2DGridMapData[idx]==212)
			{
				pucSaveImg[idx*3]=0;
				pucSaveImg[idx*3+1]=0;
				pucSaveImg[idx*3+2]=255;
			}
			else
			{
			//	pucSaveImg[idx*3]=125;
			//	pucSaveImg[idx*3+1]=125;
			//	pucSaveImg[idx*3+2]=125;
				pucSaveImg[idx*3]=test.m_pst2DGridMapData[0]->puc2DGridMapData[idx];
				pucSaveImg[idx*3+1]=test.m_pst2DGridMapData[0]->puc2DGridMapData[idx];
				pucSaveImg[idx*3+2]=test.m_pst2DGridMapData[0]->puc2DGridMapData[idx];
			}

			
		}
	}
	CBMP.BMPSave("testSave.bmp",pucSaveImg,nW,nH);


	return 0;
}