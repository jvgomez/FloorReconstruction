/*
 * map.h
 * Set of functions to apply pinhole model and generate a map reconstruction
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

#ifndef MAP_H
#define MAP_H

#include "matrix.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class Map
{
public:
Map();
~Map();

/**
	* Function to generate a map reconstruction on Mat format. The function asumes that each pixel of the image represents a square of evironment of 5cm width and 5cm height
	* @param roi vector which contains the pixels which pertains to Region of Interest
	* @param b b parameter of input surface
	* @param d d parameter of input surface
	* @param yValues height camera value vector
	* @return matrix of vectors which contains real distances
	* */
std::vector <Matrix_double> mapDistances(std::vector <int> roi, double b, double d, std::vector <double> yValues);
};

#endif // MAP_H
