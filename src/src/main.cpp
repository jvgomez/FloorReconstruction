/*
 * main.cpp
 * Interface to call all necessary funtions to perfrom the algorithm
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
#include <iostream>

#include <pcl/console/time.h>

#include "../include/filtering.h"
#include "../include/segmentation.h"
#include "../include/colour_analysis.h"
#include "../include/roi.h"
#include "../include/learning.h"
#include "../include/visualize.h"
#include "../include/save.h"
#include "../include/map.h"

int main(int argc, char** argv) 
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	int k=atoi(argv[1]);
	double min_mah_dist=atof(argv[2]);
	int max_gauss=atoi(argv[3]);
	
	std::vector <Gauss> gaussians_learned;

	pcl::PCDReader reader;
	reader.read (argv[4], *cloud); 
	
	pcl::console::TicToc timer;
	
	timer.tic();
	
	// Filtering
	Filtering filtering;
	cloudFiltered=filtering.filter(cloud);
	
	// Segmentation
	Segmentation segmentation;
	std::vector <Surface> surfaces=segmentation.segment (cloudFiltered);
	
	// Colour extraction from point cloud
	ExtractColour extractcolour;
	cv::Mat samples=extractcolour.pcd2colourRGB(surfaces[0].segmented_cloud);
	
	// Point cloud analysis
	ColourAnalysis colouranalysis;
	std::vector <Gauss> gaussians = colouranalysis.analyzeColourCluster(k, surfaces[0].segmented_cloud, surfaces[0].b, surfaces[0].d, samples);
	
	// Learning
	Learning learning;
	gaussians_learned=learning.learnGaussians(gaussians, gaussians_learned, max_gauss);
	
	// ROI
	ROI roi;
	std::vector<Line> lines=roi.roi(surfaces);
	std::vector<int> index=roi.extractIndex(lines);
	
	// Colour extraction from image
	std::vector<Matrix_double> colours=extractcolour.indexPCD2VectorRGB(*cloud, index);
	
	// Image analysis
	std::vector <int> pixels_labeled=colouranalysis.labelIndex(colours, k, gaussians_learned, min_mah_dist);
	
	// Map reconstruction
	Map map;
	std::vector<Matrix_double> distancesLabeled=map.mapDistances(index, surfaces[0].a ,surfaces[0].b, surfaces[0].c, surfaces[0].d);
	
	std::cout << "Part 1 finished" << std::endl;
	timer.toc_print();
	
	timer.tic();
	// Generate image
	cv::Mat environment=extractcolour.pcd2image(*cloud);
	
	// Calcule begin and end points for a line
	std::vector <LinePoint> linePoints=roi.calculeLinePoints(lines);
	
	// Draw lines
	Visualize visualize;
	environment=visualize.drawLines (environment, linePoints, "green");
	
	// Print floor
	std::vector<int> pixels=colouranalysis.labelPixel(pixels_labeled, index, colours);
	environment=visualize.drawFloor (pixels, environment, "cyan");
	
	std::vector <int> kinectIndex=roi.extractFloorIdx(surfaces[0].a, surfaces[0].b, surfaces[0].c, surfaces[0].d, cloud);
	environment=visualize.drawFloor (kinectIndex, environment, "blue");
	
	// Reconstruct kinect floor
	std::vector <int> kinect (kinectIndex.size(), 1);
	std::vector<Matrix_double> distancesKinect=map.mapDistances(kinectIndex, surfaces[0].a ,surfaces[0].b, surfaces[0].c, surfaces[0].d);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudEnhanced=map.mapEnhance(distancesLabeled, index, cloud, pixels_labeled, surfaces[0].a ,surfaces[0].b, surfaces[0].c, surfaces[0].d);
	
	// Generate map reconstruction image
	cv::Mat map_reconstruction(cv::Size(400,650),CV_8UC3);
	map_reconstruction=cv::Scalar(0,0,0);
	map_reconstruction=visualize.mapGeneration(map_reconstruction, pixels_labeled, distancesLabeled, "blue", "white");
	map_reconstruction=visualize.mapGeneration(map_reconstruction, kinect, distancesKinect, "red", "yellow");
	
	// Show images on a window
	std::string name_window="Floor window";
    visualize.visualizeImage (environment, name_window);
	name_window="Map window";
    visualize.visualizeImage (map_reconstruction, name_window);
    visualize.visualizeCloud(cloudEnhanced, "Cloud");
	
	// Save images
	Save save;
	char name[50]="map.png";
	save.saveImage(map_reconstruction, name);
	char name2[50]="environment.png";
	save.saveImage(environment, name2);
	
	// Save cloud
	std::string names= "cloud.pcd";
	save.saveCloud(surfaces[0].segmented_cloud, names);
		
	std::cout << "Part 2 finished" << std::endl;
	timer.toc_print();
	cv::waitKey();
	
	return 0;
}

