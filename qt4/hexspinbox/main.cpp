#include <QApplication>
#include "hexspinbox.h"

int main(int argc, char* argv[])
{
	QApplication app(argc,argv);
	HexSpinBox * pHexSpinBox = new HexSpinBox;
	pHexSpinBox->show();
	return app.exec();
}
