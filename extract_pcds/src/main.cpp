#include <iostream>
#include <string>
#include "pcl/io/pcd_io.h"
// #include "pcl/cuda/point_types.h"
#include "pcl/point_types.h"
#include "extract_pcds.h"

using namespace std;

string path_in("/home/iotrobot/Data/b502_20140212");
string path_out("/home/iotrobot/Data/b502_20140212/pcds");

int main(int argc, char* argv[])
{
    int start_port = 9012;
    int end_port = 9012;
    if(argc == 3)
    {
        start_port = atoi(argv[1]);
        end_port = atoi(argv[2]);
    }
    assert(end_port>=start_port);
    assert(end_port<=9014);
    assert(start_port>=9009);
    CExtractPCDs ext(start_port,end_port);
    ext(path_in.c_str(), path_out.c_str());

    cout<<"finished!"<<endl;
    
    return 0;
}
