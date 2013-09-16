/*
 * save.h
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

#ifndef SAVE_H
#define SAVE_H

#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class Save
{
public:
Save();
~Save();
/**
	* Function to save a Mat image on the Hard Disk
	* @param image input image on Mat format
	* @param name image name
	* @return Nothing
	* */
void saveImage(cv::Mat image, char name[50]);

/**
	* Function to save a PointXYZRGBA point cloud on the Hard Disk
	* @param cloud input point cloud on PointXYZRGBA format
	* @param name image name
	* @return Nothing
	* */
void saveCloud(pcl::PointCloud<pcl::PointXYZRGBA> cloud, std::string name);
};

#endif // SAVE_H
