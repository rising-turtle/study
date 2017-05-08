#include <iostream>

using namespace std;

template<unsigned long N>
struct binary
{
	static const unsigned int value = binary<N/10>::value << 1 | N%10;
};

template<>
struct binary<0>
{
	static const unsigned int value =0;
};

int main(int argc, char* argv[])
{
	cout<<"b<101>: "<<binary<101>::value<<endl;
	cout<<"b<1111>: "<<binary<1111>::value<<endl;
	return 0;
}
