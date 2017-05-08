/*henYue 2013.2.1 
* VideoStream 的模式設定 （解析度）
**/
#include<iostream>
#include <OpenNI.h>
using namespace std;
using namespace openni;

int main()
{
    Status rc = OpenNI::initialize();
    if(rc != STATUS_OK){
	cout << "初始化失败"<<OpenNI::getExtendedError()<< endl;
	return 1;
    }

    Device device;
    if(device.open(ANY_DEVICE) != STATUS_OK){
	cout << "打开设备失败" << OpenNI::getExtendedError() << endl;
	return 2;
    }

    VideoStream depth;
    if(device.getSensorInfo(SENSOR_DEPTH) != NULL){
	if(depth.create(device,SENSOR_DEPTH) != STATUS_OK){
	    cout << "创建深度影像传感器失败" << OpenNI::getExtendedError() << endl;
	    return 3;
	}
    }

    /*
       VideoMode 它包含除了影像的寬、高、FPS 外，
       也還有每一個像素的格式；以深度影像來說，
       主要應該就是使用以 1mm 為單位的 PIXEL_FORMAT_DEPTH_1_MM 或以 100um 為單位的 PIXEL_FORMAT_DEPTH_100_UM（另外還有兩種以 shift 為名的格式），
       彩色影像的話，則是以 PIXEL_FORMAT_RGB888、PIXEL_FORMAT_YUV422 為主
     */

    //读取目前的videostream的模式，并输出
    VideoMode depthMode = depth.getVideoMode(); //获取
    cout << "video mode is " << depthMode.getResolutionX();
    cout << " * " << depthMode.getResolutionY();
    cout << " @ " << depthMode.getFps()<<" FPS ";

    switch(depthMode.getPixelFormat()){

	case PIXEL_FORMAT_DEPTH_100_UM :
	    cout << ", unit is 100um" <<endl;
	    break;
	case PIXEL_FORMAT_DEPTH_1_MM:
	    cout << ", unit is 1mm" <<endl;
	    break;
	default:
	    //code
	    break;
    }

    //设置目前的videostream的模式，并输出
    depthMode.setFps(30);
    depthMode.setResolution(600,800);
    depthMode.setPixelFormat(PIXEL_FORMAT_DEPTH_100_UM);

    if(depth.setVideoMode(depthMode) == STATUS_OK){  //判断是否成功设置
	cout << "video mode is " << depthMode.getResolutionX();
	cout << " * " << depthMode.getResolutionY();
	cout << " @ " << depthMode.getFps() << " fps ";
	switch(depthMode.getPixelFormat()){
	    case PIXEL_FORMAT_DEPTH_100_UM:
		cout << ", unit is 100um" <<endl;
		break;
	    case PIXEL_FORMAT_DEPTH_1_MM:
		cout << ", unit is 1mm" <<endl;
		break;
	    default:
		//code
		break;
	}
    }
    depth.stop();
    depth.destroy();
    device.close();
    OpenNI::shutdown();
    return 0;
}
