#ifndef COLOR_TABLE
#define COLOR_TABLE

typedef struct _COLOR{
	unsigned char r;
	unsigned char g;
	unsigned char b;
	struct _COLOR(unsigned char _r,unsigned char _g,unsigned char _b):r(_r),g(_g),b(_b){}
}COLOR;

extern int g_color_number; //一共所有的颜色数目
extern COLOR& colorindex(int index);	// 按数字索引颜色
extern COLOR& yellow();
extern COLOR& red();
extern COLOR& green();
extern COLOR& golden();
extern COLOR& orange();
extern COLOR& blue();
extern COLOR& grey();
extern COLOR& white();
extern COLOR& black();
extern COLOR& purple();

#endif