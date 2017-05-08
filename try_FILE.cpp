/*
 * David Z, 
 * FILE operation
 *
 * */

#include <stdlib.h>
#include <stdio.h>

int main(int argc , char* argv[])
{
  FILE* fin;
  fin = fopen("num.log", "r+");
  if(fin == NULL)
  {
    fin = fopen("num.log", "w+");
    fprintf(fin, "1 \n");
    fclose(fin);
    return 0;
  }
  int num = 1;
  char buf[256] = {0,};
  while(fgets(buf, 256, fin)!=NULL)
  {
  }
  sscanf(buf, "%d \n", &num);
  printf("last line is %s and num is %d\n", buf, num);
  num++;
  // FILE* fout = fopen("num.log", "a");
  fprintf(fin, "%d \n", num);
  fclose(fin);
  return 0;
}
