#ifndef _PIXMAP_H_
#define _PIXMAP_H_

#include <QWidget>
#include <QPixmap>
#include <QImage>
#include <QObject>

class PixmapWidget : public QWidget{
	Q_OBJECT
public:
	PixmapWidget(const QString& filename, QWidget *parent=0);
	~PixmapWidget();

public Q_SLOTS:
	void setZoomFactor(float);
Q_SIGNALS:
	void zoomFactorChanged(float);

protected:
	void paintEvent(QPaintEvent*);
	void wheelEvent(QWheelEvent*);
private:
	// QPixmap * m_pm;
	QImage * m_pm;
	float zoomFactor;

};

#endif
