#include "imagemap.h"
#include <QWheelEvent>
#include <QPaintEvent>
#include <QPainter>
#include <sstream>

ImageMapWidget::ImageMapWidget(const QString& filename, QWidget* parent)
{
	m_pm = new QImage(filename);
	zoomFactor = 1.2; 
	setMinimumSize(m_pm->width()*zoomFactor, m_pm->height()*zoomFactor);
}
ImageMapWidget::~ImageMapWidget(){}

ImageMapWidget::ImageMapWidget(QWidget* parent)
{
	int sx = 0; int sy=0;
	if(parent!=0){
		sx = parent->width();
		sy = parent->height();
	}
	sx=sx>100?sx:100;
	sy=sy>100?sy:100;
	setImageSize(sx,sy);
	zoomFactor = 1.2;
	setMinimumSize(m_pm->width()*zoomFactor, m_pm->height()*zoomFactor);
}

void ImageMapWidget::setImageSize(int size_x, int size_y){
	if(size_x< 32 ) size_x = 32;
	if(size_y< 32 ) size_y = 32;
	m_pm = new QImage(size_x,size_y, 8, 256);
	m_px = new QPixmap(size_x, size_y);
	for(int i=0; i<256; i++){
		m_pm->setColor(i,qRgb(i,i,i));
	}
	m_pm->setColor(0,qRgb(200,200,255));
	m_pm->setColor(1,qRgb(255,0,0));
}

void ImageMapWidget::clearMap()
{
	std::cout<<"In clearMap!"<<std::endl;
	setImageSize(m_pm->width(),m_pm->height());
}
void ImageMapWidget::updateMap(CMap2D* map)
{
	setImageSize(map->m_size_x,map->m_size_y);
	int maxx = map->m_size_x;
	int maxy = map->m_size_y;
	
	for(int x=0;x<maxx;x++){
		for(int y=0;y<maxy;y++){
			if(map->m_mapsum[x][y]>0){
			/*if(map->m_mapsum[x][y]>0.95){
				m_pm->setPixel(x,y,(int)(1));
				}else{*/
				m_pm->setPixel(x,y,
				(int)(255-253*map->m_mapprob[x][y]));
			}else{
				m_pm->setPixel(x,y,0);
			}
		}
	}
	m_px->convertFromImage(*m_pm);
	repaint();
	
}

void ImageMapWidget::dumpscreen(void){
	static int n = 0;
	stringstream ss;
	ss<<++n<<".png";
	m_px->save(ss.str().c_str(),"PNG");
	cout<<"after dump!"<<endl;
}

void ImageMapWidget::setZoomFactor(float factor)
{
	int w,h;
	if(factor == zoomFactor)
		return ;
	zoomFactor = factor;
	emit(zoomFactorChanged(zoomFactor));
	w = m_pm->width()*zoomFactor;
	h = m_pm->height()*zoomFactor;
	setMinimumSize(w,h);

	QWidget *p = dynamic_cast<QWidget*>(parent());
	if(p!=0)
		resize(p->width(),p->height());	
	repaint();
}

// void ImageMapWidget::zoomFactorChanged(float factor)

void ImageMapWidget::paintEvent(QPaintEvent* event)
{
	int xoffset = 0;
	int yoffset = 0;
	int pixwidth = m_pm->width()*zoomFactor;
	int pixheight = m_pm->height()*zoomFactor;
	bool drawBoarder = false;
	if( width() > pixwidth )
	{
		xoffset = (width()-pixwidth)/2;
		drawBoarder = true;
	}
	if( height() > pixheight)
	{
		yoffset = (height()-pixheight)/2;
		drawBoarder = true;
	}
	
	QPainter p(this);
	p.save(); // save current transformation matrix
	p.translate(xoffset,yoffset);
	p.scale(zoomFactor,zoomFactor);
	p.drawImage(0,0,*m_pm);
	p.restore();
	if(drawBoarder){
		p.setPen(Qt::black);
		p.drawRect(xoffset-1,yoffset-1,pixwidth,pixheight);
	}
}

void ImageMapWidget::wheelEvent(QWheelEvent* event)
{
	float f;
	f = zoomFactor + 0.001*event->delta();
	if(f< 32.0/m_pm->width())
		f = 32.0/m_pm->width();
	setZoomFactor(f);
}
