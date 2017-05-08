#ifndef _MAINWIN_H_
#define _MAINWIN_H_

#include <QMainWindow>
#include <QPushButton>
#include <QWidget>
#include <QScrollArea>
#include "pixmap.h"

class MyWin : public QWidget//QMainWindow
{
	Q_OBJECT
public:
	MyWin();
	~MyWin();
public: 
	// QWidget* m_left;
	QPushButton* m_left;
	PixmapWidget* m_pw;
	QPushButton* m_cancle;
	QScrollArea* m_sa;
};

#endif
