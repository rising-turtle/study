#include "preheader.h"
#include "globaldefinition.h"

#include "Macros3ds.h"
#include "Writer.h"

void test3ds(string file_in, string file_out)
{
	C3DWriter writ;
	writ.Load(file_in.c_str());
	writ.write(file_out);
	cout<<"finished!"<<endl;
}
void testLoad3ds(string file_in)
{
	C3DWriter writ;
	writ.Load(file_in.c_str());
	writ.output(std::cout);
//	writ.testWrite(file_in);
}