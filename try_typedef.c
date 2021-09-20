#include <stdio.h>
#include <malloc.h>

typedef struct _A
{
  int a; 
  char* c; 
  struct _B* b;
}A;

typedef struct _B
{
    int ib;
}B; 

int main()
{
  A * pa = (A*)malloc(sizeof(A)); 
  pa->a = 10;
  pa->b = (B*)malloc(sizeof(B));
  pa->b->ib = 1; 
  printf("finished!\n");
  return 0;
}
