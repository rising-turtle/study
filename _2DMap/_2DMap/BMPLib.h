#pragma once
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>


#define WIDTHBYTES(bits) (((bits)+31)/32*4)

typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned long DWORD;
typedef long LONG;



typedef struct libBITMAPFILEHEADER {

//	WORD bfType;
	DWORD bfSize; //�ļ���С
	WORD bfReserved1; //�����֣�������
	WORD bfReserved2; //�����֣�ͬ��
	DWORD bfOffBits; //ʵ��λͼ���ݵ�ƫ���ֽ�������ǰ�������ֳ���֮��
} libBITMAPFILEHEADER;


//��ϢͷBITMAPINFOHEADER��Ҳ��һ���ṹ���䶨�����£�

typedef struct libBITMAPINFOHEADER{
	//public:
	DWORD biSize; //ָ���˽ṹ��ĳ��ȣ�Ϊ40
	LONG biWidth; //λͼ��
	LONG biHeight; //λͼ��
	WORD biPlanes; //ƽ������Ϊ1
	WORD biBitCount; //������ɫλ����������1��2��4��8��16��24���µĿ�����32
	DWORD biCompression; //ѹ����ʽ��������0��1��2������0��ʾ��ѹ��
	DWORD biSizeImage; //ʵ��λͼ����ռ�õ��ֽ���
	LONG biXPelsPerMeter; //X����ֱ���
	LONG biYPelsPerMeter; //Y����ֱ���
	DWORD biClrUsed; //ʹ�õ���ɫ�������Ϊ0�����ʾĬ��ֵ(2^��ɫλ��)
	DWORD biClrImportant; //��Ҫ��ɫ�������Ϊ0�����ʾ������ɫ������Ҫ��
} libBITMAPINFOHEADER;


//��ɫ��Palette����Ȼ�������Ƕ���Щ��Ҫ��ɫ���λͼ�ļ����Եġ�24λ��32λ�ǲ���Ҫ��ɫ��ġ�
//���ƺ��ǵ�ɫ��ṹ���������ʹ�õ���ɫ������

typedef struct libRGBQUAD {
	//public:
	BYTE rgbBlue; //����ɫ����ɫ����
	BYTE rgbGreen; //����ɫ����ɫ����
	BYTE rgbRed; //����ɫ�ĺ�ɫ����
	BYTE rgbReserved; //����ֵ
} libRGBQUAD;


class BMPLib
{
public:
	BMPLib(void);
	~BMPLib(void);


	void showBmpHead(libBITMAPFILEHEADER* pBmpHead);
	void showBmpInforHead(libBITMAPINFOHEADER* pBmpInforHead);
	void showRgbQuan(libRGBQUAD* pRGB);

	unsigned char* BMPLoad(char *pFile,unsigned char *pucData,int *pnWidth,int *pnHeight);
	int BMPSave(const char *pFile,const unsigned char *pucData,const int nWidth,const int nHeight);
};
