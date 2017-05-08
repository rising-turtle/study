#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <fstream>

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O

using namespace std;
using namespace cv;

void testMat2();
void record(const char* fname, Mat& mat, int w, int h);
void showMatInfo(Mat& mat)
{
  cout<<"mat info: "<<endl;
  cout<<"type: "<<mat.type()<<endl;
  cout<<"depth: "<<mat.depth()<<endl;
  cout<<"channels: "<<mat.channels()<<endl;
  cout<<"step1: "<<mat.step1()<<endl;
  cout<<"total: "<<mat.total()<<endl;
  cout<<"elemSize: "<<mat.elemSize()<<endl;
  cout<<"elemSize1: "<<mat.elemSize1()<<endl;
}

int main(int argc, char* argv[])
{
  if(argc < 2)
  {
    // cout<<"No input video source!"<<endl;
    testMat2();
    return 0;
  }
  const string fNAME("output2.avi"); 
  
  // input video
  VideoCapture captInput(argv[1]); 
  if(!captInput.isOpened())
  {
    cout<<" Could not open source video: "<<fNAME<<endl;
    return -1;
  }
  
  Size refs = Size((int) captInput.get(CAP_PROP_FRAME_WIDTH), 
                   (int) captInput.get(CAP_PROP_FRAME_HEIGHT));
  
  // output video
  VideoWriter outputVideo;
  typedef union{int ex; char ext_[5];} EXT;
  EXT ext; 
  ext.ex = static_cast<int>(captInput.get(CAP_PROP_FOURCC));     // Get Codec Type- Int form
  ext.ext_[4] = '\0';
  cout << "Source frame resolution : Width = "<<refs.width<<" Height = "<<refs.height 
       << " of nr#: "<<captInput.get(CAP_PROP_FRAME_COUNT)<<" codec type: "<<ext.ext_<<endl;
  cout << "fps for "<< ext.ext_<<" is "<<captInput.get(CAP_PROP_FPS)<<endl;
  // outputVideo.open(fNAME, ext.ex, captInput.get(CAP_PROP_FPS), refs, true);
  // outputVideo.open(fNAME.c_str(), VideoWriter::fourcc('P','I','M','1'), captInput.get(CAP_PROP_FPS), refs, true);
  // outputVideo.open("output.avi", VideoWriter::fourcc('P','I','M','1') , 20, refs, true);
  // outputVideo.open(fNAME, VideoWriter::fourcc('M','P','4','2'), captInput.get(CAP_PROP_FPS), refs, true);
  // outputVideo.open(fNAME, VideoWriter::fourcc('D','I','V','3'), captInput.get(CAP_PROP_FPS), refs, true);

  // outputVideo.open(fNAME, VideoWriter::fourcc('X','V','I','D'), 20, refs, true);
  outputVideo.open("tmp.avi", VideoWriter::fourcc('X','V','I','D'), 20, refs, true);

  // outputVideo.open(fNAME, VideoWriter::fourcc('F','L','V','1'), captInput.get(CAP_PROP_FPS), refs, true);
  // outputVideo.open(fNAME, -1, captInput.get(CAP_PROP_FPS), refs, true);
  // outputVideo.open("video.avi", -1, 29, Size(480, 640));

  if(!outputVideo.isOpened())
  {
    cout<< "Could not open output video for write: "<<fNAME<<endl;
    return -1;
  }else
  {
    cout<< "Succeed to open output video!"<<endl;
  }

  const char* WIN_SRC = "SRC_VIDEO"; 
  
  // Windows 
  namedWindow(WIN_SRC, WINDOW_AUTOSIZE);
  moveWindow(WIN_SRC, 400, 0); 
  
  cout << "Source frame resolution : Width = "<<refs.width<<" Height = "<<refs.height 
       << " of nr#: "<<captInput.get(CAP_PROP_FRAME_COUNT)<<" codec type: "<<ext.ext_<<endl;
  Mat frameSource ; 
  int frameNum = 0;
  int delay = 10;
  bool first = true;
  int nframe = 0;
  for(;;) // get the input frame
  {
    captInput >> frameSource ; 
    if(frameSource.empty() )
    {
      cout<<" < < <  Game over! > > >"<<endl;
      break;
    }
    if(first)
    {
      showMatInfo(frameSource);
      first = false;
    }
    if(++nframe == 20)
    {
      record("first1.log", frameSource, refs.width, refs.height);
    }
    // cout<<"Frame: "<< ++frameNum<< "# ";
    
    // write into a video 
    // outputVideo << frameSource;

  ////////////////// Show Image /////////////////////////
  imshow(WIN_SRC, frameSource); 

  char c = (char)waitKey(delay); 
  if(c == 27) break;
  }
  cout<<"Finish writing "<<endl;
  
  // write it again 
  captInput.release();
  outputVideo.release();
  captInput.open(fNAME);
  // outputVideo.open("output2.avi", VideoWriter::fourcc('X','V','I','D'), 20, refs, true);
  outputVideo.open("tmp2.avi", VideoWriter::fourcc('X','V','I','D'), 20, refs, true);

  first = true;
  nframe = 0;
  for(;;)
  {
    captInput >> frameSource; 
    if(frameSource.empty())
    {
      cout<<" over again! "<<endl;
      break;
    }
    if(first)
    {
      showMatInfo(frameSource);
      first = false;
    }

    if(++nframe == 20)
    {
      record("first2.log", frameSource, refs.width, refs.height);
    }

    outputVideo << frameSource;

    imshow(WIN_SRC, frameSource);
    char c = (char)waitKey(delay); 
    if(c == 27) break;
  }
  cout<<"Finish writing again!"<<endl;

  return 0;
}

void testMat2()
{
  int w = 100; 
  int h = 100;
  int n = w*h;
  cv::Mat rgb(h, w, CV_8UC3);
  
  int k = 0;
  int Module = 256;

  for(int i=0; i<h; i++)
    for(int j=0; j<w; j++)
    {
      rgb.at<cv::Vec3b>(i, j)[0] = (k%Module); // i%h + i;
      rgb.at<cv::Vec3b>(i, j)[1] = (k%Module); //(i%h)*2 + i;
      rgb.at<cv::Vec3b>(i, j)[2] = (k%Module); //(i%h)*3 + i;
      k++;
    }
  
  // write the frame into video
  cv::VideoWriter v_writer("tmp.avi", cv::VideoWriter::fourcc('X','V','I','D'), 15, cv::Size(w,h), true);
  // cv::VideoWriter v_writer("tmp.avi", -1, 15, cv::Size(w,h), true);

  // cv::VideoWriter v_writer("tmp.avi", cv::VideoWriter::fourcc('P','I','M','1'), 20, cv::Size(w,h), true);
  
  // cv::VideoWriter v_writer("tmp.avi", cv::VideoWriter::fourcc('D','I','v','3'), 20, cv::Size(w,h), true);

  if(!v_writer.isOpened())
  {
    cout<<"record_video.cpp: failed to open tmp.avi"<<endl;
    return ;
  }
  // v_writer.write(rgb);
  v_writer << rgb;
  showMatInfo(rgb);
  v_writer.release(); 

  // read the avi 
  cv::VideoCapture v_reader;
  v_reader.open("tmp.avi");
  if(!v_reader.isOpened())
  {
    cout<<"record_video.cpp: failed to read tmp.avi"<<endl;
    return ;
  }
  cv::Mat rgb_copy;
  cv::Mat rgb_yuv;
  v_reader>>rgb_copy;
  // v_reader.read(rgb_copy);
  // cv::cvtColor(rgb_yuv, rgb_copy, CV_YUV2BGR);
  // cv::cvtColor(rgb_yuv, rgb_copy, CV_BGR2YUV);

  float Wr = 0.299; 
  float Wg = 0.587; 
  float Wb = 0.114;
  
  uint8_t r, g, b;
  uint8_t y, u, v;

  for(int i=0; i<h; i++)
    for(int j=0; j<w; j++)
    {
      
    }

  // write into file 
  ofstream logf("comp_mat.log");
  for(int i=0; i<h; i++)
    for(int j=0; j<w; j++)
    {
      logf<< (int)rgb.at<cv::Vec3b>(i, j)[0] << " "<< (int)rgb_copy.at<cv::Vec3b>(i, j)[0]<<endl; 
      logf<< (int)rgb.at<cv::Vec3b>(i, j)[1] << " "<< (int)rgb_copy.at<cv::Vec3b>(i, j)[1]<<endl;
      logf<< (int)rgb.at<cv::Vec3b>(i, j)[2] << " "<< (int)rgb_copy.at<cv::Vec3b>(i, j)[2]<<endl;    
    }
  return ;
}


void record(const char* fname, Mat& rgb, int w, int h)
{
  ofstream logf(fname);
  for(int i=0; i<h; i++)
    for(int j=0; j<w; j++)
    {
     logf<< (int)rgb.at<cv::Vec3b>(i, j)[0] << " " << (int)rgb.at<cv::Vec3b>(i, j)[1] << " "<< (int)rgb.at<cv::Vec3b>(i, j)[2]<<endl;  
    }
}


