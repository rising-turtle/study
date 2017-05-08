#include "preheader.h"
#include "TestSet.h"

// divide vector va into two segment, [low, mid)< pivot < [mid, high), 
// the first segment is lower than pivot while the second bigger 
int QSort(vector<int>& va, int low, int high, int pivot)
{
	if(low>=high ) high;
	int i=low;
	int j=high;
	int tmp;
	while(i<j)
	{
		if(va[i]>pivot)
		{
			tmp=va[i]; va[i]=va[j-1]; va[j-1]=tmp;
			j--;
		}
		else
		{
			i++;
		}
	}
	return i;
}

// Divide va and vb into two corresponding sets, 
// Together to obtain a full sequence
void QSortTwice(vector<int>& va, vector<int>& vb, int low,int high)
{
	if(low>=high) return;
	int range=high-low;
	srand((unsigned int)time(NULL));
	int selected=rand()%range+low;
	
	// [low,mid)<pivot<[mid,high)
	int mid = QSort(va,low,high,vb[selected]);

	// only the one element is filterd
	if(mid==high){
		int vb1=low;
		int va1=low;
		while(vb1<high)
		{
			if(vb1==selected) {vb1++;continue;}
			if(va[va1]<vb[vb1]) va1++;
			else vb1++;
		}
		int tmp=va[high-1]; va[high-1]=va[va1]; va[va1]=tmp;
		tmp=vb[high-1]; vb[high-1]=vb[selected]; vb[selected]=tmp;
		QSortTwice(va,vb,low,high-1);
		return;
	}

	// find the two elements in va that is exactly va1 < pivot< va2
	int va1=low;
	int va2=mid;
	int vb1=low;
	while(vb1<high)
	{
		if(vb1==selected) {vb1++;continue;}
		if((vb[vb1]< va[va1] && vb[vb1]<va[va2])||(vb[vb1]>va[va1] && vb[vb1]>va[va2]))
		{
			vb1++;		
		}
		else if(vb[vb1]>va[va1] && vb[vb1]<va[va2])
		{
			if(va1<mid-1) va1++;
			else if(va2<high-1) va2++;
		}
	}
	// then we get final adj elements in va of vb[selected]
	// we have to use this element to divide vb
	int mid2 = QSort(vb,low,high,va[va2]);

	if(mid2!=mid)
		mid2 = QSort(vb,low,high,va[va1]);

	assert(mid==mid2);
	QSortTwice(va,vb,low,mid);
	QSortTwice(va,vb,mid,high);
}

void testQSort(int n)
{
	vector<int> even;
	vector<int> odd;
	int m=n;
	if(m%2==0) m++; 
	for(int i=0;i<m;i+=2)
	{
		even.push_back(i);
		odd.push_back(i+1);
	}

	// Random rank these vectors
	srand((unsigned int)time(NULL));
	int range=odd.size();
	int index,tmp;
	for(int i=0;i<m;i+=2)
	{
		index=rand()%range;
		tmp=even[range-1]; even[range-1]=even[index]; even[index]=tmp;

		index=rand()%range;
		tmp=odd[range-1]; odd[range-1]=odd[index]; odd[index]=tmp;

		range--;
	}

	display(even);
	display(odd);

	QSortTwice(even,odd,0,even.size());

	display(even);
	display(odd);
}

