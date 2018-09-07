

#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
    string fname("");
    if(argc >=2) fname = string(argv[1]); 
    cv::FileStorage fs2;
    fs2.open(fname.c_str(), cv::FileStorage::READ && cv::FileStorage::FORMAT_YAML, "");
    if(!fs2.isOpened()){
	// throw std::runtime_error(config_name+" not opened");
	cout <<"failed to read yaml file: "<<fname<<endl;
    }else
    {
	cout<<"succeed to read yaml file: "<<fname<<endl; 
    }
    return 0; 
}
