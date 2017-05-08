#ifndef _MAPPAINTER_H_
#define _MAPPAINTER_H_
#include <qapplication.h>
#include <qpainter.h>
#include <qimage.h>
#include <qfileinfo.h>
#include <qscrollarea.h>
#include <qframe.h>
#include <qstring.h>

#include "imagemap.h"
#include "map2d.h"

class MapPainter : public QScrollArea {
  Q_OBJECT
    
 public:
  MapPainter( QWidget* parent = 0, const char *name = 0 );
  ~MapPainter( void ){};

  // void centerView( CMap2D* map, logtools_rpos2_t pos );  
  // void drawContents( QPainter *p, int cx, int cy, int cw, int ch );
  // void setSize( int size_x, int size_y );
  void update( CMap2D* map );
  void refresh( CMap2D* map );
  // void showscan( MAP2 map, logtools_lasersens2_data_t lsens, double maxrange );
  // void drawrobot( MAP2 map, logtools_rpos2_t pos, int color );
  // void drawparticles( MAP2 map, SAMPLE_SET, int color, int showpath );
  // void dumpscreen( void );
  void doPaint();
 public Q_SLOTS:
 	void receMap(CMap2D* map);

 public:
  ImageMapWidget * imagewidget;
  QImage      * image;
  QPainter    * pt;
  QPixmap     * pix;
  QPixmap     * pm;

};
#endif
