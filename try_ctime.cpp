#include <time.h>
#include <utmp.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

// #define UTMP_FILE "/var/run/utmp"

int main(int argc, char* argv[])
{
  struct utmp res; 
  unsigned int utmp_size = sizeof(struct utmp); 
  int fd; 

  if(fd = open(UTMP_FILE, O_RDONLY) == -1)
  {
    printf("failed to open %s\n", UTMP_FILE); 
    return -1; 
  }
  
  int read_sz = read(fd, &res, utmp_size); 
  if(read_sz != utmp_size)
  {
    printf("failed to read from utmp_file\n");
    return -1;
  }

  time_t tmp_t; 
  // if(time(&tmp_t) != -1)
  if(time((long int*)(&res.ut_tv.tv_sec)) != -1)
  {
    printf("return is true!\n");
  }else
  {
    printf("return is false\n");
  }

  close(fd);
  return 0;

}
