#include "point_conversion.h"

void PointXYZRGBtoXYZHSV(pcl::PointXYZRGB& in, pcl::PointXYZHSV& out)
{
	float min;

	out.x = in.x; out.y = in.y; out.z = in.z;

	out.v = std::max (in.r, std::max (in.g, in.b));
	min = std::min (in.r, std::min (in.g, in.b));

	if (out.v != 0)
		out.s = (out.v - min) / out.v;
	else
	{
		out.s = 0;
		out.h = -1;
		return;
	}

	if (in.r == out.v)
		out.h = (in.g - in.b) / (out.v - min);
	else if (in.g == out.v)
		out.h = 2 + (in.b - in.r) / (out.v - min);
	else 
		out.h = 4 + (in.r - in.g) / (out.v - min);
	out.h *= 60;
	if (out.h < 0)
		out.h += 360;
}