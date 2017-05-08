#include "pixmap.h"
#include <QWheelEvent>
#include <QPaintEvent>
#include <QPainter>

PixmapWidget::PixmapWidget(const QString& filename, QWidget* parent)
{
	// m_pm = new QPixmap(filename);
	m_pm = new QImage(filename);
	zoomFactor = 1.2; 
	setMinimumSize(m_pm->width()*zoomFactor, m_pm->height()*zoomFactor);
}
PixmapWidget::~PixmapWidget(){}

void PixmapWidget::setZoomFactor(float factor)
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

// void PixmapWidget::zoomFactorChanged(float factor)

void PixmapWidget::paintEvent(QPaintEvent* event)
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
	// p.drawPixmap(0,0,*m_pm);
	p.drawImage(0,0,*m_pm);
	p.restore();
	if(drawBoarder){
		p.setPen(Qt::black);
		p.drawRect(xoffset-1,yoffset-1,pixwidth,pixheight);
	}

	// draw a ellipse
	/*p.setRenderHint(QPainter::Antialiasing, true);
	p.setPen(QPen(Qt::black, 12, Qt::DashDotLine, Qt::RoundCap));
	p.setBrush(QBrush(Qt::green, Qt::SolidPattern));
	p.drawEllipse(xoffset, yoffset, pixwidth, pixheight);*/
}


void PixmapWidget::wheelEvent(QWheelEvent* event)
{
	float f;
	f = zoomFactor + 0.001*event->delta();
	if(f< 32.0/m_pm->width())
		f = 32.0/m_pm->width();
	setZoomFactor(f);
}
