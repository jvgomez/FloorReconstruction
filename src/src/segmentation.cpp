/*
 * segmentation.cpp
 * Interface to segment a point cloud
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

#include "../include/segmentation.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/time.h>

Segmentation::Segmentation() {}

Segmentation::~Segmentation() {}

std::vector <Surface> Segmentation::segmentCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
	std::vector<Surface> surfaces;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.02);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

    int nr_points = (int) cloud->points.size ();
    // While 30% of the original cloud is still there
    while (cloud->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        Surface surf;

        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);

        surf.segmented_cloud = *cloud_p;
        surf.a = coefficients->values[0];
        surf.b = coefficients->values[1];
        surf.c = coefficients->values[2];
        surf.d = coefficients->values[3];

        surfaces.push_back(surf);
		
        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);   
    }
    
    return surfaces;
}

std::vector <PlanePosition> Segmentation::analyzePosition (std::vector <Surface> surfaces)
{
	std::vector <PlanePosition> planePositions (4);
	
	/* Position 1: Floor
	 * Position 2: Rigth wall
	 * Position 3: Left wall
	 * Position 4: Roof
	 */
	
	#pragma omp parallel 
	{
		#pragma omp for shared (surfaces, planePositions) private (i) reduction(+: sum) 
		{
			for (int i=0; i<surfaces.size(); i++)
			{
				// Vertical plane
				if (0.9 < std::abs(surfaces[i].a) && std::abs(surfaces[i].a) < 1)
				{			
					// it is left
					if(surfaces[i].d < 0) 
					{
						planePositions[2].indices.push_back(i);
					} 

					//it is right
					else 
					{   
						planePositions[1].indices.push_back(i);
					}
				}
				
				else
				{
					if (0.9 < std::abs(surfaces[i].b) && std::abs(surfaces[i].b) < 1) 
					{
						// It is down
						if(surfaces[i].d > 0)
						{
							planePositions[0].indices.push_back(i);
						} 

						// It is up
						else 
						{
							planePositions[3].indices.push_back(i);
						}
					}
				}
			}
		}
	}
	
	return planePositions;
}

std::vector <Surface> Segmentation::analyzeSize(std::vector<PlanePosition> planePositions, std::vector <Surface> surfaces)
{
	std::vector <Surface> positions;
	
	#pragma omp parallel 
	{
		#pragma omp for shared (surfaces, planePositions, positions) private (i, index, max) reduction(+: sum) 
		{
			for (int i=0; i<planePositions.size(); i++) // Analyse all plane positions
			{
				if (planePositions[i].indices.size()>0)
				{
					int index=0;
					double max=0;
					if (planePositions[i].indices.size()>1) // If this plane position exists perform the mass sort
					{
						for (int j=0; j<planePositions[i].indices.size(); j++)
						{
							if (surfaces[planePositions[i].indices[j]].segmented_cloud.height*surfaces[planePositions[i].indices[j]].segmented_cloud.width>max) 
							{
								max=surfaces[planePositions[i].indices[j]].segmented_cloud.height*surfaces[planePositions[i].indices[j]].segmented_cloud.width; // Save the maximum mass
								index=j; // Save the index
							}
						}				
					}
				
					positions.push_back(surfaces[planePositions[i].indices[index]]); // Save the surface
				}
			}
		}
	}
	
	return positions;
}

std::vector <Surface> Segmentation::segment (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	std::vector <Surface> surfaces=segmentCloud (cloud);
	std::vector <PlanePosition> planePositions= analyzePosition (surfaces);
	std::vector <Surface> positions=analyzeSize(planePositions, surfaces);
	
	std::cout << "Segmentation finished" << std::endl;
	
	return positions;
	
}