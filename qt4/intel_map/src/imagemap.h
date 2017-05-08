#ifndef _PIXMAP_H_
#define _PIXMAP_H_

#include <QWidget>
#include <QPixmap>
#include <QObject>
#include <QImage>
#include "map2d.h"

class ImageMapWidget : public QWidget{
	Q_OBJECT
public:
	ImageMapWidget(const QString& filename, QWidget *parent=0);
	ImageMapWidget(QWidget* parent = 0);
	~ImageMapWidget();

public Q_SLOTS:
	void setZoomFactor(float);
	void updateMap(CMap2D*);
	void clearMap();
	void dumpscreen();
Q_SIGNALS:
	void zoomFactorChanged(float);
public:
	void setImageSize(int, int);
protected:
	void paintEvent(QPaintEvent*);
	void wheelEvent(QWheelEvent*);
public:
	QImage *m_pm;
	QPixmap * m_px;
	float zoomFactor;
};

#endif
