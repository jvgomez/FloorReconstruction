/*
 * visualize.h
 * Set of functions to add elements to a image and show it in a new window
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

#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "surface.h"
#include "matrix.h"

class Visualize
{
public:
Visualize();
~Visualize();

/**
	* Function to draw lines on an image on Mat format
	* @param image input image on Mat format
	* @param line_points Line_point vector with two line points, begin line point and end line point
	* @param colourName name of lines colour
	* @return image with lines drawed on Mat format
	* */
cv::Mat drawLines (cv::Mat image, std::vector<LinePoint> linePoints, std::string colourName);

/**
	* Function to draw pixels on an image on Mat format
	* @param index vector with pixels to be drawed
	* @param image input image on Mat format
	* @param colourName name of lines colour
	* @return image with pixels drawed on Mat format
	* */
cv::Mat drawFloor (std::vector<int> index, cv::Mat image, std::string colourName);

/**
	* Function to convert pixel name on RGB colour parameters
	* @param colour colour name
	* @return vector with colour name RGB colours parameters
	* */
std::vector<double> colour (std::string colour);

/**
	* Function to show an image on Mat format
	* @param image input image on Mat format
	* @param windowName name of window
	* @return Nothing
	* */
void visualize (cv::Mat image, std::string windowName);

/**
	* Function to generate a map reconstruction on Mat format. The function asumes that each pixel of the image represents a square of evironment of 5cm width and 5cm height
	* @param image input image on Mat format
	* @param label vector which contains the ROI's pixels labeled as floor encoded as 0 (not labeled) and 1 (labeled)
	* @param distances vector which contains the real positions of pixels
	* @param colourNameFloor name of colour to print floor zones on map
	* @param colourNameObstacle name of colour to print obstacle zones on map
	* @return output image on Mat format
	* */
cv::Mat mapGeneration(cv::Mat image, std::vector <int> label, std::vector <Matrix_double> distances, std::string colourNameFloor, std::string colourNameObstacle);

};

#endif // VISUALIZE_H
