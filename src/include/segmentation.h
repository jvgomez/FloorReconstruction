/*
 * segmentation.h
 * Interface to segment a point cloud
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

#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "../include/surface.h"

class Segmentation
{
public:
Segmentation();
~Segmentation();

/**
	* Function to calcule the existing main planes of a point cloud on PointXYZRGBA format
	* @param cloud input cloud
	* @return Surface vector with cloud segmented sort as: Position 1 Floor, Position 2 Rigth wall, Position 3 Left wall, Position 4 Roof
	* */
std::vector <Surface> segment (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

/**
	* Function to segment a point cloud on PointXYZRGBA format
	* @param cloud input cloud
	* @return Surface vector with all cloud segmented
	* */
std::vector <Surface> segmentCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

/**
	* Function to analyze the position of segmented cloud
	* @param surfaces Surface vector which contains all segmented surfaces
	* @return Plane position vector which contains the position of each surface
	* */
std::vector <PlanePosition> analyzePosition (std::vector <Surface> surfaces);

/**
	* Function to analyze size of surfaces
	* @param plane_positions Plane position vector which contains the position of each surface
	* @param surfaces Surface vector which contains all segmented surfaces
	* @return Surface vector with existing main planes (floor, roof, left wall, right wall)
	* */
std::vector <Surface> analyzeSize(std::vector<PlanePosition> planepositions, std::vector <Surface> surfaces);
};

#endif // SEGMENTATION_H
