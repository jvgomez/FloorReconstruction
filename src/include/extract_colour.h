/*
 * extract_colour.h
 * Interface to extract colour parameters from an image or a point cloud on 
 * RGB, HSV or Lab space colours
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

#ifndef EXTRACT_COLOUR_H
#define EXTRACT_COLOUR_H

#include <pcl/io/pcd_io.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "matrix.h"

class ExtractColour
{
public:
ExtractColour();
~ExtractColour();

/**
	 * Function to extract colour parameters from a point cloud on PointXYZRGBA on RGB space colour
	 * @param cloud input cloud on PointXYZRGBA format
	 * @return colour parameters on RGB space colour on Mat format
	 * */
cv::Mat pcd2colourRGB(pcl::PointCloud<pcl::PointXYZRGBA> cloud);

/**
	 * Function to extract colour parameters of only points determinated by the user from a point cloud on PointXYZRGBA format on RGB space colour
	 * @param cloud input cloud on PointXYZRGBA format
	 * @param index vector of index of pixels to extract colour parameters
	 * @return Matrix of colour parameters on RGB space colour
	 * */
std::vector<Matrix_double> indexPCD2VectorRGB(pcl::PointCloud< pcl::PointXYZRGBA > cloud, std::vector<int> index);

/**
	 * Function to extract colour parameters from a point cloud on PointXYZRGBA format on HSV space colour
	 * @param cloud input cloud on PointXYZRGBA format
	 * @return colour parameters on HSV space colour on Mat format
	 * */
cv::Mat pcd2colourHSV(pcl::PointCloud<pcl::PointXYZRGBA> cloud);

/**
	 * Function to extract colour parameters of only points determinated by the user from a point cloud on PointXYZRGBA format on HSV space colour
	 * @param cloud input cloud on PointXYZRGBA format
	 * @param index vector of index of pixels to extract colour parameters
	 * @return Matrix of colour parameters on HSV space colour on Mat format
	 * */
std::vector<Matrix_double> indexPCD2VectorHSV(pcl::PointCloud< pcl::PointXYZRGBA > cloud, std::vector<int> index);

/**
	 * Function to extract colour parameters from a point cloud on PointXYZRGBA format on Lab space colour
	 * @param cloud input cloud on PointXYZRGBA format
	 * @return colour parameters on Lab space colour on Mat format
	 * */
cv::Mat pcd2colourLab(pcl::PointCloud<pcl::PointXYZRGBA> cloud);

/**
	 * Function to extract colour parameters of only points determinated by the user from a point on PointXYZRGBA format cloud on Lab space colour
	 * @param cloud input cloud on PointXYZRGBA format
	 * @param index vector of index of pixels to extract colour parameters
	 * @return Matrix of colour parameters on Lab space colour on Mat format
	 * */
std::vector<Matrix_double> indexPCD2VectorLab(pcl::PointCloud< pcl::PointXYZRGBA > cloud, std::vector<int> index);

/**
	 * Function to extract an image on Mat format from a point cloud on PointXYZRGBA format
	 * @param cloud input cloud on PointXYZRGBA format
	 * @return image on Mat format
	 * */
cv::Mat pcd2image(pcl::PointCloud<pcl::PointXYZRGBA> cloud);
};

#endif // EXTRACT_COLOUR_H
