/*
 * save.cpp
 * Interface to save an image or a point cloud
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

#include "../include/save.h"
#include <iostream>

Save::Save() {}

Save::~Save() {}

void Save::saveImage(cv::Mat image, char name[50])
{
	cv::imwrite(name, image);
	
	std::cout << "Save image finished" << std::endl;
}

void Save::saveCloud(pcl::PointCloud<pcl::PointXYZRGBA> cloud, std::string name)
{
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGBA> (name, cloud, false);
	
	std::cout << "Cloud saved" << std::endl;
}