#include <vector>
using namespace std;
#define GRID_SIZE 0.1 //0.1m
#define GRID_NUM 1000 //100m

#define GRID_MAP_NUM 10
#define GRID_MAP_MEM_POOL_SIZE 100*1024*1024  //100M


typedef struct _2DGridMapData
{
	float fGridResolution;
	char cMapName[32];
	float fMapScale;
	int nMapW;
	int nMapH;

	float fMapActualW;
	float fMapActualH;

	int n2DGridMapW;
	int n2DGridMapH;
	int n2DGridMapIdx;
	int n2DGridMapDataLen;
	unsigned char *puc2DGridMapData;
	_2DGridMapData *pstSub2DGridMapData;
	unsigned char *pucAccWeight;
}_2DGridMapData;







class _2DGridMap
{
public:

	int Init2DGridMap();
	int AddNewMap(float fScale,int nImgW,int nImgH,unsigned char *pucMap,char *pcMapName,float fGridResolution);


	int GetShortestPath(int nStartPosIdx,int nEndPosIdx,int n2DGridMapIdx,vector<int> &vctPath);
	int GetOptimunPath(int nStartPosIdx,int nEndPosIdx,int n2DGridMapIdx,vector<int> &vctPath);


	_2DGridMapData *m_pst2DGridMapData[GRID_MAP_NUM];

private:

	char m_cOccupyFlag[GRID_MAP_NUM];

	int Convert22DGridMap(unsigned char *pucMap,_2DGridMapData *pst2DGridMapData);
	int AvailableMoveAreaDetec(int nStartPosIdx);
	int SeedGrowing(int nStartPosIdx,_2DGridMapData *pst2DGridMapData);

	int RemarkWeightMap(int nStartPosIdx,int nEndPosIdx,_2DGridMapData *pst2DGridMapData,unsigned char *pucAccWeightMap,vector<int> &vctPath);
	int MergeGridsBasedOnRobotSize(int nRobotSize,_2DGridMapData *pst2DGridMapData);
	int UpdateAccWeightMap(unsigned char *pucAccWeightMap,_2DGridMapData *pst2DGridMapData);

	int GetShortestPath(int nStartPosIdx,int nEndPosIdx,_2DGridMapData *pst2DGridMapData);
	int GetOptimunPath(int nStartPosIdx,int nEndPosIdx,_2DGridMapData *pst2DGridMapData);

	int AStar(int nStartPosIdx,int nEndPosIdx,
				_2DGridMapData *pst2DGridMapData,
				unsigned char *pucAccWeightMap,
				int *pnParentMap,vector<int> &vctPath);
	int GetMinWeightIdx(vector<int> vctIdx,int *pnWeightMap);
protected:
};