/*
 * colour_analysis.cpp
 * Interface to analyse colour parameters
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
#include<iostream>
#include<omp.h>

#include "../include/colour_analysis.h"
#include "../include/extract_colour.h"
#include "../include/algebra.h"

ColourAnalysis::ColourAnalysis() {}

ColourAnalysis::~ColourAnalysis() {}

cv::Mat ColourAnalysis::clusterLabels (int k, cv::Mat colourExtracted)
{
       
	cv::Mat labels;
	int attemps = 2*k;
	cv::Mat centers;
	cv::kmeans(colourExtracted, k, labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), attemps, cv::KMEANS_PP_CENTERS, centers); // k-means cluster performed by openCV
	
    return labels;
}

std::vector<Matrix_int>ColourAnalysis::labels2vector (int k, cv::Mat labels)
{
	std::vector<Matrix_int>clusters(k);
	
	#pragma omp parallel 
	{
		#pragma omp for shared (labels, clusters) private (i) reduction(+: sum) 
		{
			for (int i=0; i<labels.rows; i++)
			{
				clusters[labels.at<int>(i,0)].columns.push_back(i); // divide labels on k clusters vectors
			}
		}
	}
	return clusters;
}

Gauss ColourAnalysis::calculeGaussiansParameters(std::vector<Matrix_double> colors)
{
	std::vector<Gauss>gaussians;
	std::vector<Matrix_double>matrix_datas(colors.size());
	
	Gauss gauss;
	Statistics statistics;
    Algebra algebra;
	
	#pragma omp parallel 
	{
		#pragma omp for shared (colors) private (i, j) reduction(+: sum) 
		{
			for (int i=0; i<colors[0].columns.size(); i++)
			{
				std::vector<double>datas;
				Matrix_double m;
				
				for (int j=0; j<colors.size(); j++)
				{
					datas.push_back(colors[j].columns[i]);
				}
				
				gauss.means.push_back(statistics.mean(datas)); // calcule means values
			}
		}
	}

	std::vector<Matrix_double>covariances=statistics.covMeans(colors, gauss.means); // calcule covariances
	gauss.covariances=covariances;
    gauss.covariances_invert=algebra.inv(covariances); // calcule covariances_invert
	
	gauss.mass=colors.size(); // calcule mass
	
	return gauss;	
}

std::vector<Gauss>ColourAnalysis::gaussiansParameters(int k, cv::Mat labels, pcl::PointCloud<pcl::PointXYZRGBA> cloud)
{
	std::vector<Matrix_int>indices=labels2vector(k, labels);
	std::vector<Gauss>gaussians;
	
	ExtractColour extractcolour;
	
	#pragma omp parallel 
	{
		#pragma omp for shared (indices) private (i, colors) reduction(+: sum) 
		{
			for (int i=0; i<k; i++)
			{
				std::vector<Matrix_double>colors=extractcolour.indexPCD2VectorRGB(cloud, indices[i].columns); // extract colout parameters of every cluster
				gaussians.push_back(calculeGaussiansParameters(colors));
			}
		}
	}
	
	return gaussians;
}

int ColourAnalysis::detectFloor(double b, double d)
{
	int value;
	if (0.9<std::abs(b) && std::abs(b)<1) 
	{
		// It is floor
		if(d>0)
			return 1;

		// It is not floor
		else 
			return -1;
	}
}

std::vector<Gauss>ColourAnalysis::analyzeColourCluster (int k, pcl::PointCloud<pcl::PointXYZRGBA>cloud, double b, double d, cv::Mat colourExtracted)
{
	int value=detectFloor(b, d); // detect if the input plane is a floor
	
	if (value==1)
	{		
		cv::Mat labels=clusterLabels(k, colourExtracted); // calcule cluster

		std::vector<Gauss>gaussians=gaussiansParameters(k, labels, cloud); // calcule gaussians parameters
		
		std::cout<<"Cluster finished"<<std::endl;
		
		return gaussians;
	}
	
	else
	{
		std::cout<<"Error, no floor detected"<<std::endl; // show an error
		
		std::exit(EXIT_FAILURE);
	}

}

double ColourAnalysis::mahalanobisDistance(std::vector<double>colours, int k, std::vector<Gauss>gaussians, double distance_min)
{
	Statistics statistics;
	
	#pragma omp parallel 
	{
		#pragma omp for shared (gaussians, mahalanobis) private (j) reduction(+: sum) 
		{
			for (int j=0; j<gaussians.size(); j++)
			{
				
				double mahalanobis_aux=statistics.mahalanobisDistance2Point(gaussians[j].covariances_invert, gaussians[j].means, colours); // calcule Mahalanobis distance
				if (mahalanobis_aux<distance_min)
					return 1;
			}
		}
	}
	
	return 0;
}

std::vector<int>ColourAnalysis::labelIndex(std::vector<Matrix_double>colours, int k, std::vector<Gauss>gaussians, double distance_min)
{
	std::vector<int>index(colours.size(),0);
	#pragma omp parallel 
	{
		#pragma omp for shared (colours, index) private (i, mahalanobis) reduction(+: sum) 
		{
			for (int i=0; i<colours.size(); i++)
				index[i]=mahalanobisDistance(colours[i].columns, k, gaussians, distance_min); // set value to this index vector position. 1 if the point is close to a gaussian or 0 if it is not

		}
	}
	
	return index;
}

std::vector<int>ColourAnalysis::labelPixel(std::vector<int>index_labeled, std::vector<int>index, std::vector<Matrix_double>colours)
{
	std::vector<int>pixelsLabeled ;
	
	#pragma omp parallel 
	{
		#pragma omp for shared (pixelsLabeled) private (i) reduction(+: sum) 
		{
			for (int i=0; i<colours.size(); i++)
			{
				if(index_labeled[i]==1)
					pixelsLabeled.push_back(index[i]);
			}
		}
	}
	return pixelsLabeled;
}

std::vector<int>ColourAnalysis::analyzeColourImage (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, int k, std::vector<Gauss>gaussians, std::vector<int>roi, double distance_min, std::vector<Matrix_double>colours)
{
	std::vector<int>index_labeled=labelIndex(colours, k, gaussians, distance_min);
	std::vector<int>pixelsLabeled=labelPixel(index_labeled, roi, colours);
	
	std::cout<<"Pixels labeled"<<std::endl;
	
	return pixelsLabeled;
}
