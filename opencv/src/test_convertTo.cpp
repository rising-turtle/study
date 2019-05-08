# include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

int main()
{
    float data[9] = {1.0, -0.1, 0.33, 2.5, 10.2, -100.2, 0.1, -0.5, 0.9}; 
    Mat d(3, 3, CV_32FC1, data); 

    cout <<"before convert "<<endl<<d<<endl;
    d.convertTo(d, CV_16UC1, 1000); 
    cout <<"after convert "<<endl<<d<<endl;
    return 0; 
}
