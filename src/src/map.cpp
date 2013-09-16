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

#include "../include/map.h"
#include "../include/pinhole.h"

Map::Map() {}

Map::~Map() {}

std::vector <Matrix_double> Map::mapDistances(std::vector <int> roi, double b, double d, std::vector <double> yValues)
{
	std::vector <Matrix_double> distances (roi.size());
	Pinhole pinhole;

	#pragma omp parallel 
	{
		#pragma omp for shared (image, label) private (i, j) reduction(+: sum) 
		{
			for (int i=0; i<roi.size(); i++)
			{
				int y=roi[i]/640;
				int x=roi[i]%640;
				
				distances[i].columns=pinhole.pinholeInverse(x, y, b, d, yValues[x]);

			}
		}
	}
	
	return distances;
}