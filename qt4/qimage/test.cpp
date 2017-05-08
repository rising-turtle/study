#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <QBitmap>

class PainterWidget : public QWidget {
	protected:
		void paintEvent(QPaintEvent*);
};

#define INDEX_RANGE 1000
#define IMAGE_RANGE 1000

void PainterWidget::paintEvent(QPaintEvent *event) {
	unsigned char* array;
	unsigned char* base;
	array = (unsigned char*)malloc(INDEX_RANGE*INDEX_RANGE*sizeof(unsigned char));
	base = array;
	int k,s;
	s = 0;
	for(k=0;k<INDEX_RANGE*INDEX_RANGE;k++){       
		*array = s;
		array++;
		s++;
		if(s == 255) s=0; 
	}
	array = base;
	QPainter painter(this);
	QImage image(IMAGE_RANGE, IMAGE_RANGE, QImage::Format_RGB32);
	QRgb value;

	int i,j;
	for(i=0;i<IMAGE_RANGE;i++)
		for(j=0;j<IMAGE_RANGE;j++){
			value = qRgb((int)(*array),(int)(*array), (int)(*array));
			image.setPixel(i, j, value);
			array++;
		}
	painter.drawImage(0,0,image);
}

int main(int argc, char *argv[]) {
	QApplication app(argc, argv);

	PainterWidget pWidget;
	pWidget.setWindowTitle("QPixmap & QBitmap");
	pWidget.resize(400, 150);
	pWidget.show();

	return app.exec();
}
