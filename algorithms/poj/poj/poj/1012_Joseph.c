#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "malloc.h"


int IsValidCounting(int* guys, int*used, int N, int count)
{
	int last_good_guy=(N>>1)-1;
	int i=0;
	int bad_num=0;
	int all_bad_num = last_good_guy+1;
	while(1)
	{
		int cout=0;
		while(cout<count)
		{
			i%=N;
			if(!used[i]) cout++;
			if(cout==count) break;
			i++;
		}
		if( i>last_good_guy )
		{
			bad_num++;
		}
		else if(bad_num<all_bad_num)
		{
			return 0;
		}
		if(bad_num>=all_bad_num)
			return 1;
		used[i]=1;
		i++;
	}
	return 1;
}

void showResult(int * guys, int N, int count)
{
	int* used = (int*)malloc(N*sizeof(int));
	int i=0;
	int num = 0;
	while(1)
	{
		int cout = 0;
		while(cout<count)
		{
			i=i%N;
			if(!used[i]) cout++;
			if(cout==count) break;
			i++;
		}
		printf("%d ",guys[i]);
		num++;
		used[i] = 1;
		if(num==N) break;
	}
	printf("\n");
	return ;
}
int calNumberCounting(int N)
{
	int first_bad_guy = N>>1;
	int* guys =(int*)malloc(N*sizeof(int));
	int i;
	for(i=0;i<N;i++)
		guys[i]=i;
	int* used =(int*)malloc(N*sizeof(int));
	int least_count = first_bad_guy;
	int max_count = 100;
	for(i=least_count;i<max_count;i++)
	{
		memset(used,0,N*sizeof(int));
		if(IsValidCounting(guys,used,N,i))
		{
			printf("Counting num: %d\n",i);
			showResult(guys,N,i);
			break;
		}
	}
	if(i==max_count){
		printf("failed to find rignt counting number!\n");
	}

	free(used);
	free(guys);
}

int main()
{	
	int good_guys;
	calNumberCounting(6);
	/*while(scanf("%d",&good_guys)!=EOF)
	{
		
	}*/
	return 0;
}
