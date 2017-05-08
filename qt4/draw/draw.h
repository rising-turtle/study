#ifndef DRAW_H
#define DRAW_H

#include <QWidget>

class QPushButton;

class MyWidget : public QWidget
{
	Q_OBJECT
public:
	MyWidget(QWidget* parent = 0);
public slots:
	void paint();
	QPushButton * m_pPaintBtn;

};
#endif
