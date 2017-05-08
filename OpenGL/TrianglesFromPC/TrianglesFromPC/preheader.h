#ifndef PREHEADER_H
#define PREHEADER_H

#pragma once
#include <windows.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <fstream>

#define NOMINMAX

#include <GL\gl.h>
#include <GL\glu.h>
#include <GL\glut.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

#endif
