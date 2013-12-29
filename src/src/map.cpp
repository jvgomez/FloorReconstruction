/*
 * map.cpp
 * Set of functions to apply pinhole model and generate a map reconstruction
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
#include <omp.h>
#include <cmath>

#include "../include/map.h"
#include "../include/pinhole.h"
#include "../include/algebra.h"

Map::Map() {}

Map::~Map() {}

std::vector <Matrix_double> Map::mapDistances(std::vector <int> roi, double a, double b, double c, double d)
{
	std::vector <Matrix_double> distances (roi.size());
	Pinhole pinhole;
    Algebra algebra;
	
	std::vector <double> wpi (3,0);
	wpi[1]=-1;
	std::vector <double> wpr (3,0);
	wpr[0]=a;
	wpr[1]=b;
	wpr[2]=c;
	
    double alpha=algebra.angleVectors(wpi, wpr);

	#pragma omp parallel 
	{
		#pragma omp for shared (image, label) private (i, j) reduction(+: sum) 
		{
			for (int i=0; i<roi.size(); i++)
			{
                std::vector <double> pixels (3,1);
                pixels[1]=roi[i]/640;
                pixels[0]=roi[i]%640;
				
                std::vector <double> pixelsRotated=algebra.productMatrix3x1(algebra.rotz(-alpha),pixels);
				
                distances[i].columns=pinhole.pinholeInverse(pixelsRotated[0], pixelsRotated[1], b, d);
			}
		}
	}
	
	return distances;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Map::mapEnhance(std::vector <Matrix_double> distances, std::vector <int> roi, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector <int> labeled, double a, double b, double c, double d)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudEnhanced=cloud;
    Algebra algebra;

    std::vector <double> wpi (3,0);
    wpi[1]=-1;
    std::vector <double> wpr (3,0);
    wpr[0]=a;
    wpr[1]=b;
    wpr[2]=c;

    double alpha=algebra.angleVectors(wpi, wpr);

    #pragma omp parallel
    {
        #pragma omp for shared (cloudEnhanced, a, b, c, d, alpha, roi) private (i) reduction(+: sum)
        {
            for (int i=0; i<roi.size(); i++)
            {
                if (!pcl_isfinite(cloudEnhanced->points[roi[i]].x) && !pcl_isfinite(cloudEnhanced->points[roi[i]].y) && !pcl_isfinite(cloudEnhanced->points[roi[i]].z) && labeled[i]==1)
                {
                    if (std::abs(distances[i].columns[1]) <25)
                    {
                        std::vector <double> distances_aux=distances[i].columns;
                        distances_aux.push_back(1);

                        std::vector <double> distancesRotated=algebra.productMatrix3x1(algebra.rotz(alpha), distances_aux);

                        cloudEnhanced->points[roi[i]].x=distancesRotated[0];
                        cloudEnhanced->points[roi[i]].z=distancesRotated[1];
                        cloudEnhanced->points[roi[i]].y=(-d-a*cloudEnhanced->points[roi[i]].x-c*cloudEnhanced->points[roi[i]].z)/b;
                    }
                }
            }
        }
    }
	
	return cloudEnhanced;
}
