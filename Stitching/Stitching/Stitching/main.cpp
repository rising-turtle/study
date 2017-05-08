#include <iostream>

extern void testWarp();
extern int mytest();
extern int test_estimateH();
extern int test_robustmatching();


int main(){
	// mytest();
	// test_estimateH();
	 testWarp();
	 std::cout<<"finished!"<<std::endl;
	return 0;
}