#include <iostream>
#include <dirent.h> 

using namespace std;

bool DirExist(const char* pDir)
{
  if(pDir == 0) return false; 
  
  DIR *pD; 
  bool bExist = false; 
  
  pD = opendir(pDir); 
  if(pD != 0)
  {
    bExist = true; 
    closedir(pD); 
  }

  return bExist;
}

int main(int argc, char* argv[])
{
  char* pDir = "./data";
  if(argc >= 1)
    pDir = argv[1]; 
  

  return 0; 
}
