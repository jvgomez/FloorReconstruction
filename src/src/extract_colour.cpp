/*
 * extract_colour.cpp
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
#include <omp.h>

#include "../include/extract_colour.h"
#include "../include/conversion.h"

ExtractColour::ExtractColour() {}

ExtractColour::~ExtractColour() {}

cv::Mat ExtractColour::pcd2colourRGB(pcl::PointCloud<pcl::PointXYZRGBA> cloud)
{
	cv::Mat colour_extracted(cloud.height*cloud.width, 3, CV_32F);
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudPtr (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud)); 
	
	#pragma omp parallel 
	{
		#pragma omp for shared (colour_extracted, cloud) private (i, j) reduction(+: sum) 
		{
			for(int i=0; i<cloudPtr->width; i++)
				for(int j = 0; j <cloudPtr->height; j++)
				{
					colour_extracted.at<float>(i + j*cloudPtr->width, 1) = cloudPtr->points[i].r; // save R parameter
					colour_extracted.at<float>(i + j*cloudPtr->width, 2) = cloudPtr->points[i].g; // save G parameter
					colour_extracted.at<float>(i + j*cloudPtr->width, 3) = cloudPtr->points[i].b; // save B parameter
				}
		}
	}
		
	std::cout << "Colour extraction finished" << std::endl;
		
	return colour_extracted;
}

std::vector<Matrix_double> ExtractColour::indexPCD2VectorRGB(pcl::PointCloud<pcl::PointXYZRGBA> cloud, std::vector<int> index)
{
	std::vector <Matrix_double> colours(index.size());
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudPtr (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud)); 
	
	#pragma omp parallel 
	{
		#pragma omp for shared (colours, index) private (i) reduction(+: sum) 
		{
			for (int i=0; i<index.size(); i++)
			{
				colours[i].columns.push_back(cloudPtr->points[index[i]].r); // save R parameter
				colours[i].columns.push_back(cloudPtr->points[index[i]].g); // save G parameter
				colours[i].columns.push_back(cloudPtr->points[index[i]].b); // save B parameter
			}
		}
	}
	
	return colours;
}

cv::Mat ExtractColour::pcd2colourHSV(pcl::PointCloud<pcl::PointXYZRGBA> cloud)
{
	cv::Mat colour_extracted(cloud.height*cloud.width, 3, CV_32F);
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudPtr (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));
	
	Conversion conversion;
	
	#pragma omp parallel 
	{
		#pragma omp for shared (colour_extracted, cloud) private (i, j) reduction(+: sum) 
		{
			for(int i=0; i<cloudPtr->width; i++)
				for(int j = 0; j <cloudPtr->height; j++)
				{
					std::vector<double> hsv=conversion.rgb2hsv(cloudPtr->points[i].r,cloudPtr->points[i].g,cloudPtr->points[i].b); // perform RGB to HSV conversion
					
					colour_extracted.at<float>(i + j*cloudPtr->width, 1) = hsv[0]; // save H parameter
					colour_extracted.at<float>(i + j*cloudPtr->width, 2) = hsv[1]; // save S parameter
					colour_extracted.at<float>(i + j*cloudPtr->width, 3) = hsv[2]; // save V parameter
				}
		}
	}
		
	return colour_extracted;
}

std::vector<Matrix_double> ExtractColour::indexPCD2VectorHSV(pcl::PointCloud<pcl::PointXYZRGBA> cloud, std::vector<int> index)
{
	std::vector <Matrix_double> colours(3);
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudPtr (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

	Conversion conversion;
	
	#pragma omp parallel 
	{
		#pragma omp for shared (index, cloud, colours) private (i) reduction(+: sum) 
		{
			for (int i=0; i<index.size(); i++)
			{
				std::vector<double> hsv=conversion.rgb2hsv(cloudPtr->points[index[i]].r,cloudPtr->points[index[i]].g,cloudPtr->points[index[i]].b); // perform RGB to HSV conversion
				
				colours[i].columns.push_back(hsv[0]); // save H parameter
				colours[i].columns.push_back(hsv[1]); // save S parameter
				colours[i].columns.push_back(hsv[2]); // save V parameter
			}
		}
	}
	
	return colours;
}

cv::Mat ExtractColour::pcd2colourLab(pcl::PointCloud<pcl::PointXYZRGBA> cloud)
{
	cv::Mat colour_extracted(cloud.height*cloud.width, 3, CV_32F);
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudPtr (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));
	
	#pragma omp parallel 
	{
		#pragma omp for shared (cloud, colour_extracted) private (i, j) reduction(+: sum) 
		{
			for(int i=0; i<cloudPtr->width; i++)
				for(int j = 0; j <cloudPtr->height; j++)
				{
					Conversion conversion;
					std::vector<int> Lab=conversion.rgb2lab(cloudPtr->points[i].r,cloudPtr->points[i].g,cloudPtr->points[i].b); // perform RGB to Lab conversion
					
					colour_extracted.at<float>(i + j*cloudPtr->width, 1) = Lab[0]; // save L parameter
					colour_extracted.at<float>(i + j*cloudPtr->width, 2) = Lab[1]; // save a parameter
					colour_extracted.at<float>(i + j*cloudPtr->width, 3) = Lab[2]; // save b parameter
				}
		}
	}
		
	std::cout << "Colour extraction finished" << std::endl;
		
	return colour_extracted;
}

std::vector<Matrix_double> ExtractColour::indexPCD2VectorLab(pcl::PointCloud<pcl::PointXYZRGBA> cloud, std::vector<int> index)
{
	std::vector <Matrix_double> colours(3);
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudPtr (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));
	
	Conversion conversion;
	
	#pragma omp parallel 
	{
		#pragma omp for shared (cloud, index, colours) private (i) reduction(+: sum) 
		{
			for (int i=0; i<index.size(); i++)
			{
				std::vector<int> lab=conversion.rgb2lab(cloudPtr->points[index[i]].r,cloudPtr->points[index[i]].g,cloudPtr->points[index[i]].b); // perform RGB to Lab conversion
				
				colours[i].columns.push_back(lab[0]); // save L parameter
				colours[i].columns.push_back(lab[1]); // save a parameter
				colours[i].columns.push_back(lab[2]); // save b parameter
			}
		}
	}
	
	return colours;
}

cv::Mat ExtractColour::pcd2image(pcl::PointCloud<pcl::PointXYZRGBA> cloud)
{
	unsigned char *rgb_buffer = new unsigned char[ cloud.height*cloud.width * 3]; // Create a vector of unsigned char values for save RGB values
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudPtr (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

	int j=0;
	
	// fill the vector
	#pragma omp parallel 
	{
		#pragma omp for shared (cloud, rgb_buffer, j) private (i) reduction(+: sum) 
		{
			for (int i=0; i<640*480; i++)
			{
				rgb_buffer[j]=cloudPtr->points[i].b; // save B parameter
				rgb_buffer[j+1]=cloudPtr->points[i].g; // save G parameter
				rgb_buffer[j+2]=cloudPtr->points[i].r; // save R parameter
				j=j+3;
			}
		}
	}

	cv::Mat image(cv::Size(640,480), CV_8UC3, rgb_buffer, cv::Mat::AUTO_STEP);
	
	std::cout << "Image creation finished" << std::endl;
		
	return image;
}
