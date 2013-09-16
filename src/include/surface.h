/*
 * surface.h
 * Generate some structs to define parameters related to a surface
 * definition
 * Part of Floor Segmentation project
 * Copyright (C) 2013 Jose Pardeiro <jose.pardeiro@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SURFACE_H
#define SURFACE_H

#include <vector>
#include <pcl/io/pcd_io.h>

struct Surface
{
  double a,b,c,d; // plane parameters
  pcl::PointCloud<pcl::PointXYZRGBA> segmented_cloud; // segmented cloud on PointXYZRGBA format
};

struct Intersection
{
	std::vector <double> points; // intersection points
	std::vector <double> vectorial_products; // vectorial product of intersection points
};

struct Line
{
	double b, a; // line parameters: a=slope and b=y-intercept of the line
};

struct LinePoint
{
	std::vector<double> x, y; // points parameters
};

struct PlanePosition
{
	std::vector <int> indices; // index of planes
};

#endif // SURFACE_H