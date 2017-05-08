#ifndef COLOR_TABLE_H
#define COLOR_TABLE_H

template<unsigned char Red, unsigned char Green, unsigned char Blue>
struct tm_color{
    static unsigned char const R=Red;
    static unsigned char const G=Green;
    static unsigned char const B=Blue;
};

typedef tm_color<255,0,0> tmRed;
typedef tm_color<0,255,0> tmGreen;
typedef tm_color<0,0,255> tmBlue;

struct gl_color{
    gl_color(unsigned char r, unsigned char g, unsigned char b)
        : r_(r), g_(g), b_(b)
    {}
    gl_color(const gl_color& rhs)
    {r_ = rhs.r_; g_=rhs.g_; b_=rhs.b_; }
    unsigned char r_;
    unsigned char g_;
    unsigned char b_;
};

extern gl_color& getRed();
extern gl_color& getGreen();
extern gl_color& getBlue();


#endif
