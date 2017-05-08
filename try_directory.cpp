/*
 * David Z, 
 * if the directory is not exist, create it with authority 0700
 *
 * */

#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
  struct stat dir;
  string path("./tmp");
  if(stat(path.c_str(), &dir) == 0 && S_ISDIR(dir.st_mode))
  {
    cout<<"dir "<<path<<" exist!"<<endl;
  }else
  {
    // create this directory 
    cout<<"dir "<<path<<" not exist, create it now!"<<endl;
    int result = mkdir(path.c_str(), 0700);
    cout<<" mkdir return : "<<result<<endl;
  }
  return 0; 
}
