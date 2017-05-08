#include "colortable.h"

gl_color& getRed()
{
    static gl_color red(tmRed::R, tmRed::G, tmRed::B);
    return red;
}
gl_color& getGreen()
{
    static gl_color green(tmGreen::R, tmGreen::G, tmGreen::B);
    return green;
}
gl_color& getBlue()
{
    static gl_color blue(tmBlue::R, tmBlue::G, tmBlue::B);
    return blue;
}
