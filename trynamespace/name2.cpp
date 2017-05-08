#include "name.h"
namespace name2{
	int at_c(){
		static int i=0;
		return ++i;
	}
	struct Point{
		int x;
	};
}
