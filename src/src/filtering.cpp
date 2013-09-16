/*
 * filtering.cpp
 * Interface to filter a point cloud
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

#include "../include/filtering.h"

Filtering::Filtering() {}

Filtering::~Filtering() {}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Filtering::filter (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
	sor.setInputCloud (cloud);
	// Set voxel size
	sor.setLeafSize (0.03f, 0.03f, 0.03f);
	
	// Perform the filtering
	sor.filter (*cloud_filtered);
	
	std::cout << "Filtering finished" << std::endl;
	
	return (cloud_filtered);
}

