#include "preheader.h"
#include "color_table.h"

typedef COLOR& (*color_func)();

int g_color_number=8;//10; // 目前只支持了10种颜色

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
		color_func_table.insert(make_pair(index_color++,yellow)); // 黄色
		color_func_table.insert(make_pair(index_color++,red));	  // 红色
		color_func_table.insert(make_pair(index_color++,green));  // 绿色
		color_func_table.insert(make_pair(index_color++,golden)); // 金色
		color_func_table.insert(make_pair(index_color++,orange)); // 橙色
		color_func_table.insert(make_pair(index_color++,blue));   // 蓝色
		color_func_table.insert(make_pair(index_color++,grey));   // 灰色
		//color_func_table.insert(make_pair(index_color++,white));  // 白色
		//color_func_table.insert(make_pair(index_color++,black));  // 黑色
		color_func_table.insert(make_pair(index_color++,purple)); // 紫色
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
