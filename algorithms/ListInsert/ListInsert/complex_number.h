#ifndef COMPLEX_NUMBER
#define COMPLEX_NUMBER

class Real{
public:
	Real(int up, int down){}
	~Real(){}

	Real operator + (const Real& l, const Real& r){
		int up = l.up* r.down + l.down * r.up;
		int down = l.down * r.down;
		return Real(up,down);
	}

	int up;
	int down;
};


#endif

