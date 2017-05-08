#ifndef COPYTRY_H
#define COPYTRY_H

class Test
{
public:
	Test(){}
	Test(const Test& other){}
	Test(int a ){}
	Test operator +(Test& other)
	{
		Test tmp;
		return tmp;
	}
	Test operator =(const Test& other)
	{
		return other;
	}
};

#endif