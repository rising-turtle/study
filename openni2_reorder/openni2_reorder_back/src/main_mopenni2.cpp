

#include "mopenni2.h"
#include <WinSock2.h>
#include <WinSock.h>

void test_openni();
void test_socket();

SOCKET gs_; // socket handle

bool ConnectToHost(int PortNo, char * IPAddress);
void SendToHost();
void CloseConnection();

int main()
{
	test_socket();
	return 0;
}

void SendToHostXtion()
{
	MOpenni2 mopenni;
	bool openni_ok = mopenni.init();
	Mat rgb, depth;
	
	static const int width = 640; 
	static const int height = 480; 
	static const int n_pixel = width*height;
	static const int header_size = 8; // timestamp
	static const int depth_size = 2 * n_pixel;
	static const int rgb_size = 3 * n_pixel;
	static const int total_size = header_size + depth_size + rgb_size;
	char* pbuf = new char[total_size];
	memset(pbuf, 0, total_size);

	if(openni_ok)
	{
		printf("openni ok \n");
		char key=0;

		char savePathDep[255];
		char savePathRGB[255];
		char png[255];
		ofstream outf_rgbd("rgbd.txt");
		printf("start to capture \n");
		while(key!='c')
		{
			if(mopenni.getFrame(rgb, depth))
			{
				//save
				double t = mopenni.getRosTime();
				sprintf(png, "%f.png", t);
				outf_rgbd<<std::fixed<<std::setprecision(6)<<t<<" "<<png<<endl;
				sprintf(savePathDep, "../data/depth/%f.png", t);
				if(imwrite(savePathDep, depth))
				{
					cout<<"successful write dpt file: "<<savePathDep<<endl;
				}else{
					cout<<"failed to write dpt file: "<<savePathDep<<endl;
				}
				sprintf(savePathRGB, "../data/rgb/%f.png", t);
				if(imwrite(savePathRGB, rgb))//seems imwrite only work with bgr now
				{
					cout<<"successful write rgb file: "<<savePathRGB<<endl;
				}else
				{
					cout<<"failed to write rgb file: "<<savePathRGB<<endl;
				}
				// construct buf 
				// 1 header timestamp
				memcpy(pbuf, &t, sizeof(double)); 
				// 2 depth value 
				memcpy(pbuf+header_size, depth.data, depth_size);
				// 3 rgb value 
				memcpy(pbuf+header_size+depth_size, rgb.data, rgb_size);

				// send this value through socket
				int iResult = send(gs_, pbuf, total_size, 0);
				if(iResult < total_size )
				{
					cout<<"send partial data: "<<iResult<<" < "<<total_size<<endl;
				}else 
				{
					cout<<"send all data: "<<iResult<<" = "<<total_size<<endl;
				}
				break;
			}
			else
				break;
			
		}
		mopenni.close();
	}
	else
		printf("can not init mopenni2 \n");
	delete []pbuf;
}

void SendToHost()
{
	int iResult ;
	char buf[] = {"abcde"};
	iResult = send(gs_,buf,sizeof(buf)-1, 0);
	cout<<"send "<<iResult<<" bytes!"<<endl;
	memset(buf, 0, sizeof(buf));
	buf[0] = '#';
	iResult = send(gs_,buf,sizeof(buf)-1, 0);
	cout<<"send "<<iResult<<" bytes!"<<endl;
	iResult = shutdown(gs_, SD_SEND);
	if(iResult == SOCKET_ERROR)
	{
		cout<<"shutdown failed "<<iResult<<endl;
		return ;
	}
	char recv_buf[128];
	do{
		iResult = recv(gs_, recv_buf, sizeof(recv_buf)-1, 0);
		if(iResult > 0)
		{
			cout<<"Bytes rece "<<iResult<<endl;
		}else if (iResult == 0)
		{
			cout<<"connection closed!"<<endl;
		}else
			cout<<"recv failed "<<iResult<<endl;
	}while(iResult >0);
	return ;
}

bool ConnectToHost(int PortNo, char * IPAddress)
{
	// Start up Winsock ...
	WSADATA wsadata;
	int error = WSAStartup(0x0202, &wsadata);
	
	if(error)
	{
		cout<<"error to start up WSA!"<<endl;
		return false;
	}
	
	// Did we get the right Winsock version
	if(wsadata.wVersion != 0x0202)
	{
		cout<<"wsa get the version version: "<<std::hex<<wsadata.wVersion<<endl;
		WSACleanup();
		return false;
	}
	
	// Fill out the information needed to initialize a socket...
	SOCKADDR_IN target; // Socket address information
	target.sin_family = AF_INET; // Address family
	target.sin_port = htons(PortNo); // Port to connect to
	target.sin_addr.s_addr = inet_addr(IPAddress);
	
	gs_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); // create socket
	if(gs_ == INVALID_SOCKET)
	{
		cout<<"failed to create socket!"<<endl;
		return false;
	}

	// Try connecting ... 
	if(connect(gs_, (SOCKADDR*)&target, sizeof(target)) == SOCKET_ERROR)
	{
		cout<<"failed to connect to socket!"<<endl;
		return false;
	}else
	{
		cout<<"succeed to connect to socket!"<<endl;
	}
	return true;
}

void CloseConnection()
{
	if(gs_)
	{
		closesocket(gs_);
	}
	WSACleanup(); // Clean up Winsock
}

void test_socket()
{
	if(ConnectToHost(6060,"127.0.0.1"))
	{
		cout<<"succeed to connect to host!"<<endl;
		// SendToHost();
		SendToHostXtion();
		cout<<"after send data!"<<endl;
	}else{
		cout<<"failed to connect to host!"<<endl;
	}
	int k;
	cin>>k;
	CloseConnection();
}

void test_openni()
{
	MOpenni2 mopenni;
	bool openni_ok = mopenni.init();
	Mat rgb, depth;

	if(openni_ok)
	{
		printf("openni ok \n");
		cvNamedWindow("depth");
		cvNamedWindow("image");
		char key=0;

		char savePathDep[255];
		char savePathRGB[255];
		char png[255];
		ofstream outf_rgbd("rgbd.txt");
		printf("start to capture \n");
		while(key!=27)
		{
			if(mopenni.getFrame(rgb, depth))
			{

				//save
				double t = mopenni.getRosTime();
				sprintf(png, "%f.png", t);
				outf_rgbd<<std::fixed<<std::setprecision(6)<<t<<" "<<png<<endl;
				outf_rgbd.flush();
				sprintf(savePathDep, "../data/depth/%f.png", t);
				if(imwrite(savePathDep, depth))
				{
					cout<<"successful write dpt file: "<<savePathDep<<endl;
				}else{
					cout<<"failed to write dpt file: "<<savePathDep<<endl;
				}
				sprintf(savePathRGB, "../data/rgb/%f.png", t);
				if(imwrite(savePathRGB, rgb))//seems imwrite only work with bgr now
				{
					cout<<"successful write rgb file: "<<savePathRGB<<endl;
				}else
				{
					cout<<"failed to write rgb file: "<<savePathRGB<<endl;
				}

				imshow("depth", depth);
				imshow("image", rgb);
				key=cvWaitKey(10);
			}
			else
				break;

		}

		mopenni.close();
	}
	else
		printf("can not init mopenni2 \n");
}