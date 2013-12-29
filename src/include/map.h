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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class Map
{
public:
Map();
~Map();

/**
    * Function to generate a map reconstruction on Mat format.
	* @param roi vector which contains the pixels which pertains to Region of Interest
    * @param a a parameter of input surface
    * @param b b parameter of input surface
    * @param c c parameter of input surface
    * @param d d parameter of input surface
	* @return matrix of vectors which contains real distances
	* */
std::vector <Matrix_double> mapDistances(std::vector <int> roi, double a, double b, double c, double d);

/**
    * Function to generate a map reconstruction on PCD format.
    * @param distances vector which contains the real distance of pixels which pertains to Region of Interest
    * @param roi vector which contains the pixels which pertains to Region of Interest
    * @param cloud input cloud to be enhanced
    * @param labeled vector which contains which pixels are labeled as floor
    * @param a a parameter of input surface
    * @param b b parameter of input surface
    * @param c c parameter of input surface
    * @param d d parameter of input surface
    * @return matrix of vectors which contains real distances
    * */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mapEnhance(std::vector <Matrix_double> distances, std::vector <int> roi, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector <int> labeled, double a, double b, double c, double d);
};

#endif // MAP_H
