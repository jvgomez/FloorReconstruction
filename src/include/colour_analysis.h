/*
 * colour_analysis.h
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

#ifndef CLUSTER_H
#define CLUSTER_H

#include "statistics.h"
#include "extract_colour.h"

#include <pcl/io/pcd_io.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class ColourAnalysis
{
public:
ColourAnalysis();
~ColourAnalysis();

/**
	* Function to perform a cluster k-means of colour parameters
	* @param k number of clusters
	* @param colourExtracted colour parameters on Mat format
	* @return Mat vector which contains the number of cluster of each pixel
	* */
cv::Mat clusterLabels (int k, cv::Mat colourExtracted);

/**
	 * Function to convert labels from cluster k-means to a matrix of index related to each cluster
	 * @param k number of clusters
	 * @param labels Mat vector which contains the number of cluster of each pixel
	 * @return Matrix vector which contains pixels related to each cluster
	 * */
std::vector<Matrix_int> labels2vector (int k, cv::Mat labels);

/**
	 * Function to convert labels from cluster k-means to a matrix of index related to each cluster
	 * @param k number of clusters
	 * @param labels Mat vector which contains the number of cluster of each pixel
	 * @return Matrix vector which contains pixels related to each cluster
	 * */
Gauss calculeGaussiansParameters(std::vector<Matrix_double> colors);

/**
	 * Function to perform all necessary funtions to obtain Gaussians parameters using a cluster
	 * @param k number of clusters
	 * @param cloud input cloud on PointXYZRGBA format
	 * @param b parameter of input plane
	 * @param d parameter of input plane
	 * @param colourExtracted input colour parameters on Mat format
	 * @return Gauss vector of Gaussians parameters
	 * */
std::vector <Gauss> analyzeColourCluster(int k, pcl::PointCloud<pcl::PointXYZRGBA> cloud, double b, double d, cv::Mat colourExtracted);

/**
	 * Function to transform input colour data on gaussians parameters
	 * @param k number of clusters
	 * @param labels Mat vector which contains the number of cluster of each pixel
	 * @param cloud input cloud on PointXYZRGBA format
	 * @return Gauss vector of Gaussians parameters
	 * */
std::vector<Gauss> gaussiansParameters(int k, cv::Mat labels, pcl::PointCloud<pcl::PointXYZRGBA> cloud);

/**
	 * Function to detect if the input plane is a floor
	 * @param b parameter of input plane
	 * @param d parameter of input plane
	 * @return -1 if the plane is not a floor and 1 if the plane is a floor
	 * */
int detectFloor (double b, double d);

/**
	 * Function to perform all necessary funtions to obtain a Gaussians comparation 
	 * @param cloud input cloud on PointXYZRGBA format
	 * @param k number of clusters
	 * @param gaussians input vector of Gaussians parameters
	 * @param index input vector of pixels to be analysed
	 * @param distance_min minimal Mahalanobis distance allowed
	 * @param colours matrix vector which contains colour parameters
	 * @return vector with pixels labeled as floor
	 * */
std::vector <int> analyzeColourImage (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, int k, std::vector<Gauss> gaussians, std::vector<int> index, double distance_min, std::vector<Matrix_double> colours);

/**
	 * Function to perform the pixels label
	 * @param colours Matrix vector which contains colour parameters
	 * @param k number of clusters
	 * @param gaussians input Gauss vector of Gaussians parameters
	 * @param mahalanobisDistance_min minimal Mahalanobis distance allowed
	 * @return vector which positions could be 0 (not labeled) and 1 (labeled)
	 * */
std::vector <int> labelIndex(std::vector<Matrix_double> colours, int k, std::vector<Gauss> gaussians, double mahalanobisDistance_min);

/**
	 * Function to calculate minimal Mahalanobis distance between a point and every Gaussian
	 * @param colours Matrix vector which contains colour parameters
	 * @param k number of clusters
	 * @param gaussians input Gauss vector of Gaussians parameters
	 * @return 1 if the point is near to one cluster or 0 if it is not
	 * */
double mahalanobisDistance (std::vector<double> colours, int k, std::vector<Gauss> gaussians, double distance_min);

/**
	 * Function to convert a vector encoded as 0 (not labeled) and 1 (labeled) to pixels value
	 * @param index_labeled input vector encoded as 0 (not labeled) and 1 (labeled)
	 * @param roi input vector with all pixels which could be floor
	 * @param colours Matrix vector which contains colour parameters
	 * @return vector with pixels labeled as floor
	 * */
std::vector <int> labelPixel(std::vector<int> index_labeled, std::vector<int> roi, std::vector<Matrix_double> colours);
};

#endif // CLUSTER_H
