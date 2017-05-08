#include "map2d.h"
#include <cmath>

void
grid_line_core( ivector2d start, ivector2d end,
		vector<ivector2d>& line, int& cnt)
{
  int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;

  // int cnt = 0;
  cnt = 0;
  line.clear();

  dx = abs((float)(end.x-start.x)); dy = abs((float)(end.y-start.y));
  
  if (dy <= dx) {
    d = 2*dy - dx; incr1 = 2 * dy; incr2 = 2 * (dy - dx);
    if (start.x > end.x) {
      x = end.x; y = end.y;
      ydirflag = (-1);
      xend = start.x;
    } else {
      x = start.x; y = start.y;
      ydirflag = 1;
      xend = end.x;
    }
    line[cnt].x=x;
    line[cnt].y=y;
    // line.push_back(ivector2d(x,y));
    cnt++;
    if (((end.y - start.y) * ydirflag) > 0) {
      while (x < xend) {
	x++;
	if (d <0) {
	  d+=incr1;
	} else {
	  y++; d+=incr2;
	}
	line[cnt].x=x;
	line[cnt].y=y;
	// line.push_back(ivector2d(x,y));
	cnt++;
      }
    } else {
      while (x < xend) {
	x++;
	if (d <0) {
	  d+=incr1;
	} else {
	  y--; d+=incr2;
	}
	line[cnt].x=x;
	line[cnt].y=y;
	// line.push_back(ivector2d(x,y));
	cnt++;
      }
    }		
  } else {
    d = 2*dx - dy;
    incr1 = 2*dx; incr2 = 2 * (dx - dy);
    if (start.y > end.y) {
      y = end.y; x = end.x;
      yend = start.y;
      xdirflag = (-1);
    } else {
      y = start.y; x = start.x;
      yend = end.y;
      xdirflag = 1;
    }
    line[cnt].x=x;
    line[cnt].y=y;
    // line.push_back(ivector2d(x,y));
    cnt++;
    if (((end.x - start.x) * xdirflag) > 0) {
      while (y < yend) {
	y++;
	if (d <0) {
	  d+=incr1;
	} else {
	  x++; d+=incr2;
	}
	line[cnt].x=x;
	line[cnt].y=y;
	// line.push_back(ivector2d(x,y));
	cnt++;
      }
    } else {
      while (y < yend) {
	y++;
	if (d <0) {
	  d+=incr1;
	} else {
	  x--; d+=incr2;
	}
	line[cnt].x=x;
	line[cnt].y=y;
	// line.push_back(ivector2d(x,y));
	cnt++;
      }
    }
  }
}

void
grid_line( ivector2d start, ivector2d end,
	   vector<ivector2d>& line, int& cnt) {
  int i,j;
  int half;
  ivector2d v;
  grid_line_core( start, end, line, cnt );
  if ( start.x!=line[0].x ||
       start.y!=line[0].y ) {
    half = line.size()/2;
    for (i=0,j=line.size() - 1;i<half; i++,j--) {
      v = line[i];
      line[i] = line[j];
      line[j] = v;
    }
  }
}


