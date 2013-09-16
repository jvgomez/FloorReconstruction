/*
 * visualize.cpp
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

#include "../include/visualize.h"
#include "../include/matrix.h"

Visualize::Visualize() {}

Visualize::~Visualize() {}

std::vector <double> Visualize::colour (std::string colour)
{
	std::vector <double> colours(3);
	
	// select colour name string and return RGB values
	if (colour=="black")
	{
		colours[0]=0;
		colours[1]=0;
		colours[2]=0;
	}
			
	else
	{
		if (colour=="white")
		{
			colours[0]=255;
			colours[1]=255;
			colours[2]=255;
		}
			
		else
		{
			if (colour=="red")
			{
				colours[0]=255;
				colours[1]=0;
				colours[2]=0;
			}
			
			else
			{
				if (colour=="green")
				{
					colours[0]=0;
					colours[1]=255;
					colours[2]=0;
				}
				
				else
				{
					if (colour=="blue")
					{
						colours[0]=0;
						colours[1]=0;
						colours[2]=255;
					}
					
					else
					{
						if (colour=="yellow")
						{
							colours[0]=255;
							colours[1]=255;
							colours[2]=0;
						}
						
						else
						{
							if (colour=="cyan")
							{
								colours[0]=0;
								colours[1]=255;
								colours[2]=255;
							}
							
							else
							{
								if (colour=="magenta")
								{
									colours[0]=255;
									colours[1]=0;
									colours[2]=255;
								}
							}
						}
					}
				}
			}
		}
	}
	
	return colours;
}


cv::Mat Visualize::drawLines(cv::Mat image, std::vector<LinePoint> linePoints, std::string colourName)
{
	std::vector <double> colours =colour (colourName); // Obtain colour parameters from colour name

	IplImage im = image;
	
	// Draw lines on image
	cvLine (&im, cvPoint(linePoints[0].x[0], linePoints[0].y[0]), cvPoint(linePoints[0].x[1],linePoints[0].y[1]), cvScalar(colours[2],colours[1],colours[0]), 4, 8);
	cvLine (&im, cvPoint(linePoints[1].x[0], linePoints[1].y[0]), cvPoint(linePoints[1].x[1],linePoints[1].y[1]), cvScalar(colours[2],colours[1],colours[0]), 4, 8);
	
	image=&im;
	
	return image;
}

cv::Mat Visualize::drawFloor (std::vector<int> index, cv::Mat image, std::string colourName)
{
	std::vector <double> colours =colour (colourName); // Obtain colour parameters from colour name
	
	#pragma omp parallel 
	{
		#pragma omp for shared (index, image) private (i, x, y) reduction(+: sum) 
		{
			// Change colour of pixels defined on input index vector
			for (int i=0; i<index.size(); i++)
			{
				int x=index[i]/640;
				int y=index[i]%640;
			
				image.at<cv::Vec3b>(x,y)[3] = colours[2];
				image.at<cv::Vec3b>(x,y)[2] = colours[0];
				image.at<cv::Vec3b>(x,y)[1] = colours[1];
			}
		}
	}
	
	
	return image;
}


cv::Mat Visualize::mapGeneration(cv::Mat image, std::vector <int> label, std::vector <Matrix_double> distances, std::string colourNameFloor, std::string colourNameObstacle)
{
	std::vector<double> floor=colour(colourNameFloor); // obtain colour parameters of input name
	std::vector<double> obstacle=colour(colourNameObstacle); // obtain colour parameters of input name
	
	/* Perform map reconstruction
	 * The system asumes that each pixel of image represents a square of evironment of 5cm width and 5cm height
	 * */
	#pragma omp parallel 
	{
		#pragma omp for shared (image, label) private (i, j) reduction(+: sum) 
		{
			for (int i=0; i<label.size(); i++)
			{
				//If the pixel is located in a safe zone (in a distance between 0 an 40m in absolute value) print the pixel
				if (distances[i].columns[0]!=-999999 && distances[i].columns[1]!=-999999)
				{
					int x_aux1=700-abs(distances[i].columns[1])/5;
			
					int y_aux1=250+(distances[i].columns[0])/5;

					std::vector <double> colours (3,0);
					
					if (label[i]==1)
						colours=floor;
					
					else
						colours=obstacle;
					
					for (int j=x_aux1; j<=x_aux1+10; j++)
					{
							image.at<cv::Vec3b>(j,y_aux1)[3] = colours[2];
							image.at<cv::Vec3b>(j,y_aux1)[2] = colours[0];
							image.at<cv::Vec3b>(j,y_aux1)[1] = colours[1];
					}
				}
			}		
		}
	}

	
	return image;
}

void Visualize::visualize (cv::Mat image, std::string windowName)
{
	cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);// Create a window for display.
    cv::imshow(windowName, image);  // Show an image inside the window
}
