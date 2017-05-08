#include <stdlib.h>
#include <iostream>
#include <string>
#include<fstream>  
//openni
#include <stdio.h>
#include <OpenNI.h>

//#include "OniSampleUtilities.h"

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

//opencv
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <opencv/cvwimage.h>



using namespace std;
using namespace cv;
using namespace openni;


int main( int argc, char** argv )
{



	// for opencv Mat

	int fps = 30;
	int img_h = 480;
	int img_w = 640;
	Mat  m_depth16u( 480,640,CV_16UC1);
	Mat  m_rgb8u( 480,640,CV_8UC3);
	Mat  m_DepthShow( 480,640,CV_8UC1);
	Mat  m_ImageShow( 480,640,CV_8UC3);
	cvNamedWindow("depth");
	cvNamedWindow("image");

	char key=0;

	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}

	Device device;
	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK)
	{
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 2;
	}


	openni::VideoStream			m_depthStream;
	openni::VideoStream			m_colorStream;

	openni::VideoMode depthVideoMode;
	openni::VideoMode colorVideoMode;

	depthVideoMode.setFps(fps);
	depthVideoMode.setResolution(img_w, img_h);
	//depthVideoMode.setPixelFormat(PIXEL_FORMAT_DEPTH_100_UM);
	depthVideoMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	colorVideoMode.setFps(fps);
	colorVideoMode.setResolution(img_w, img_h);
	colorVideoMode.setPixelFormat(PIXEL_FORMAT_RGB888);

	if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		rc = m_depthStream.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}



		//set depth mode
		m_depthStream.setVideoMode(depthVideoMode);




		rc = m_depthStream.start();
		if (rc != STATUS_OK)
		{
			printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
			return 4;
		}

	}


	rc = m_colorStream.create(device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		//first set mode
		m_colorStream.setVideoMode(colorVideoMode);
		//second start
		rc = m_colorStream.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
			m_colorStream.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	if (!m_depthStream.isValid() || !m_colorStream.isValid())
	{
		printf("SimpleViewer: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}

	openni::VideoStream**		m_streams;
	m_streams = new openni::VideoStream*[2];
	m_streams[0] = &m_depthStream;
	m_streams[1] = &m_colorStream;

	openni::VideoFrameRef		m_depthFrame;
	openni::VideoFrameRef		m_colorFrame;

	//registration
	if(device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		if(device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR)!=STATUS_OK)
		{
			printf("Can't hardware registration \n");
			return 0;
		}
	}

	//synchronization
	rc = device.setDepthColorSyncEnabled(true);
	if (rc != openni::STATUS_OK)
	{
		printf("Can't frame sync \n");
		return 0;
	}

	if(m_depthStream.getMirroringEnabled())
	{
		m_depthStream.setMirroringEnabled(false);

	}
	if(m_colorStream.getMirroringEnabled())
	{
		m_colorStream.setMirroringEnabled(false);

	}

	//
	unsigned int pair_id = 0;
	std::string frame = "camera";
	char savePathDep[255];
	char savePathRGB[255];
	char png[255];
	ofstream outf_rgbd("rgbd.txt");
	printf("start to capture \n");
	while( key!=27 )
	{

		int changedIndex;
		openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
		if (rc != openni::STATUS_OK)
		{
			printf("Wait failed\n");
			return 0;
		}

		printf("changedIdx: %d \n", changedIndex);

		switch (changedIndex)
		{
		case 0:
			m_depthStream.readFrame(&m_depthFrame); break;
		case 1:
			m_colorStream.readFrame(&m_colorFrame); break;
		default:
			printf("Error in wait\n");
		}

		if (m_depthFrame.isValid())
		{
			printf("depth is valid \n");
			DepthPixel* pDepth = (DepthPixel*)m_depthFrame.getData();
			memcpy(m_depth16u.data,(void*)pDepth,m_depthFrame.getHeight()*m_depthFrame.getWidth()*2);
			m_depth16u.convertTo(m_DepthShow,CV_8U,255/2096.0);
		}

		if (m_colorFrame.isValid())
		{
			printf("rgb is valid \n");
			RGB888Pixel* pColor = (RGB888Pixel*)m_colorFrame.getData();
			memcpy(m_rgb8u.data,pColor,m_depthFrame.getHeight()*m_depthFrame.getWidth()*3);
			cvtColor(m_rgb8u,m_ImageShow,CV_RGB2BGR);
		}

		/*
		double t = MOpenni2::getRosTime();
		sprintf(png, "%f.png", t);
		outf_rgbd<<std::fixed<<std::setprecision(6)<<t<<" "<<png<<endl;

		sprintf(savePathDep, "../data/depth/%f.png", t);
		imwrite(savePathDep, m_depth16u);
		sprintf(savePathRGB, "../data/rgb/%f.png", t);
		imwrite(savePathRGB, m_ImageShow);//seems imwrite only work with bgr now
		*/

		imshow("depth", m_DepthShow);
		imshow("image", m_ImageShow);
		key=cvWaitKey(10);


	}

	// 10. stop
	m_depthStream.stop();
	m_colorStream.stop();
	m_depthStream.destroy();
	m_colorStream.destroy();
	device.close();
	OpenNI::shutdown();
	return 0;
} 
