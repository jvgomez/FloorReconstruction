/*
 * roi.h
 * Interface to calculate the Region of Interest of a floor
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
#ifndef ROI_H
#define ROI_H

#include "surface.h"

class ROI
{
public:
ROI();
~ROI();

/**
	* Function to obtain the pixels which pertains to a Region of Interest
	* @param surfaces vector of surfaces
	* @return vector with two lines of ROI definition
	* */
std::vector <Line> roi(std::vector<Surface> surfaces);

/**
	* Function to detect if the input surfaces has left and rigth wall
	* @param a1 a parameter of input surface 1
	* @param d1 d parameter of input surface 1
	* @param a2 a parameter of input surface 2
	* @param d2 d parameter of input surface 2
	* @return -1 if two walls are avaliable and 1 if it does not
	* */
int detectWalls (double a1, double d1, double a2, double d2);

/**
	* Function to calcule intersection of two planes
	* @param surfaces surfaces vector
	* @return Intersection vector with planes intersection data
	* */
std::vector <Intersection> intersectionPlanes(std::vector<Surface> surfaces);

/**
	* Function to calcule intersection lines on 2 dimensions
	* @param intersections Intersection vector with planes intersection data
	* @return lines data vector
	* */
std::vector<Line> calculeLines(std::vector<Intersection> intersections);

/**
	* Function to extract pixels between two lines
	* @param lines lines data vector
	* @return vector of pixels located between the two lines
	* */
std::vector<int> extractIndex(std::vector<Line> lines);

/**
	* Function to extract pixels which pertains to the floor
	* @param a a parameter of input surface
	* @param b b parameter of input surface
	* @param c c parameter of input surface
	* @param d d parameter of input surface 
	* @param cloud_blob input cloud on PointXYZRGBA format
	* @return vector of pixels which pertains to the floor
	* */
std::vector <int> extractFloorIdx(double a, double b, double c, double d, pcl::PointCloud< pcl::PointXYZRGBA >::ConstPtr cloud);

/**
	* Function to calcule two points of a line
	* @param lines lines data vector
	* @return vector with two points to define a line
	* */
std::vector <LinePoint> calculeLinePoints(std::vector<Line> lines);

};

#endif // ROI_H
