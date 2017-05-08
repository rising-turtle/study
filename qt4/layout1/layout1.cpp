#include <QApplication>
#include <QHBoxLayout>
#include <QSlider>
#include <QSpinBox>

int main(int argc, char* argv[])
{
	QApplication app(argc,argv);

	QWidget * window = new QWidget;
	window->setWindowTitle("Enter your ege!");

	QSpinBox *spinbox = new QSpinBox;
	QSlider *splider = new QSlider(Qt::Horizontal);
	spinbox->setRange(0,130);
	splider->setRange(0,130);

	QObject::connect(spinbox,SIGNAL(valueChanged(int)),
			splider,SLOT(setValue(int)));
	QObject::connect(splider,SIGNAL(valueChanged(int)),
			spinbox,SLOT(setValue(int)));
	spinbox->setValue(40);

	QHBoxLayout *layout = new QHBoxLayout;
	layout->addWidget(spinbox);
	layout->addWidget(splider);
	window->setLayout(layout);

	window->show();

	return app.exec();

}
