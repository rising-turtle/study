#ifndef _MAIN_WINDOW_H
#define _MAIN_WINDOW_H

#include <QObject>
#include <QWidget>
#include <QPushButton>
#include <QScrollArea>
#include "points.h"
#include "map2d.h"
#include "gen_points.h"
#include "graphics.h"

class CMainWindow : public QWidget
{
	Q_OBJECT
public:
	CMainWindow(QWidget* parent=NULL);
	~CMainWindow();
	void inner_connections();
	void resizeWindowSize(int width, int height);
public:
	QPushButton* m_btn_start;
	QPushButton* m_btn_finalmap;
	QPushButton* m_btn_quit;
	QPushButton* m_btn_dump;
	CPoints* m_points_dis; 
	CMap2D* m_final_map; 
	MapPainter* m_map_painter;
};


#endif
