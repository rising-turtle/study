#include <iostream>
#include <string>

#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>

using namespace std;
namespace mpl = boost::mpl;

template<typename T>
struct type_descriptor{
	const char* operator()()
	{
		return "non_scalar";
	}
private:
	string info;
};

template<typename T>
struct type_descriptor<T*>{
	const char* operator()()
	{
		info = " pointer to";
		info += type_descriptor<T>()();
		return info.c_str();
	}
private:
	string info;
};

template<typename T>
struct type_descriptor<T volatile>{
	const char* operator()(){
		info = " volatile";
		info += type_descriptor<T>()();
		return info.c_str();	
	}
private:
	string info;
};

template<typename T>
struct type_descriptor<T&>{
	const char* operator()(){
		info =" reference of";
		info += type_descriptor<T>()();
		return info.c_str();
	}
private:
	string info;
};

template<>
struct type_descriptor< int >{
	const char* operator()()
	{
		return " int";
	}	
};
template<>
struct type_descriptor<const int>{
	const char* operator()()
	{
		return " const int";
	}
};

template<typename T>
struct type_descriptor<T[]>{
	const char* operator()(){
		info = " array of";
		info += type_descriptor<T>()();
		return info.c_str();
	}
	string info;
};

template<typename T, int N>
struct type_descriptor<T[N]>{
    const char* operator()()
    {
        info = " N array of";
        info += type_descriptor<T>()();
        return info.c_str();
    }
    string info;
};

template<typename T>
struct type_descriptor<T(*)()>{
	const char* operator()(){
		info = " function returning to";
		info += type_descriptor<T>()();
		return info.c_str();
	}
	string info;
};

template<typename T, typename U>
struct type_descriptor<T(*)(U)>{
	const char* operator()(){
		info = " function with argument type";
		info += type_descriptor<U>()();
		info += " returning to";
		info += type_descriptor<T>()();
		return info.c_str();
	}
	string info;
};

int main()
{
	cout<<type_descriptor< int* >()()<<endl;
	cout<<type_descriptor<int*&>()()<<endl;
	cout<<type_descriptor<const int*&>()()<<endl;
	cout<<type_descriptor<volatile int>()()<<endl;
	cout<<type_descriptor<int *(*[])()>()()<<endl;
	cout<<type_descriptor<int& (*[])(int)>()()<<endl;
    cout<<type_descriptor<int [][5]>()()<<endl;
	return 0;
}

