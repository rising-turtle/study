#ifndef OVEN_TIME_H
#define OVEN_TIME_H

#include <QWidget>
#include <QTimer>
#include <QDateTime>

class QPaintEvent;
class QPainter;
class QMouseEvent;

class OvenTime : public  QWidget 
{
	Q_OBJECT
public:
	OvenTime(QWidget* parent = 0);

	void setDuration(int secs);
	int duration() const;
	void draw(QPainter* painter);
signals:
	void timeout();
protected:
	void paintEvent(QPaintEvent* event);
	void mousePressEvent(QMouseEvent *event);
private:
	QDateTime finishTime;
	QTimer *updateTimer;
	QTimer *finishTimer;
};

#endif
