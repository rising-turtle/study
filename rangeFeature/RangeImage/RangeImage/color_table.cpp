#include "preheader.h"
#include "color_table.h"

typedef COLOR& (*color_func)();

int g_color_number=8;//10; // Ŀǰֻ֧����10����ɫ

COLOR& colorindex(int index)
{
	if(index <0 || index>=g_color_number)
	{
		cout<<"error color index: "<<index<<endl;
		return COLOR(0,0,0);
	}
	static map<int, color_func> color_func_table;
	if(color_func_table.size()<=0)
	{
		static int index_color=0;
		color_func_table.insert(make_pair(index_color++,yellow)); // ��ɫ
		color_func_table.insert(make_pair(index_color++,red));	  // ��ɫ
		color_func_table.insert(make_pair(index_color++,green));  // ��ɫ
		color_func_table.insert(make_pair(index_color++,golden)); // ��ɫ
		color_func_table.insert(make_pair(index_color++,orange)); // ��ɫ
		color_func_table.insert(make_pair(index_color++,blue));   // ��ɫ
		color_func_table.insert(make_pair(index_color++,grey));   // ��ɫ
		//color_func_table.insert(make_pair(index_color++,white));  // ��ɫ
		//color_func_table.insert(make_pair(index_color++,black));  // ��ɫ
		color_func_table.insert(make_pair(index_color++,purple)); // ��ɫ
	}
	return (*color_func_table[index])();
}

COLOR& yellow(){
	static COLOR yellow(255,255,0);
	return yellow;
}
COLOR& red(){
	static COLOR red(255,0,0);
	return red;
}
COLOR& green(){
	static COLOR green(0,255,0);
	return green;
}
COLOR& golden(){
	static COLOR golden(255,215,0);
	return golden;
}
COLOR& orange(){
	static COLOR orange(255,97,0);
	return orange;
}
COLOR& blue(){
	static COLOR blue(0,0,255);
	return blue;
}
COLOR& grey(){
	static COLOR grey(192,192,192);
	return grey;
}
COLOR& white(){
	static COLOR white(255,255,255);
	return white;
}
COLOR& black(){
	static COLOR black(0,0,0);
	return black;
}
COLOR& purple(){
	static COLOR purple(160,32,240);
	return purple;
}
