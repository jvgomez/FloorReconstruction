/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2013  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include <cmath> 
#include <omp.h>

#include "../include/pinhole.h"

Pinhole::Pinhole() {}

Pinhole::~Pinhole() {}

std::vector<double> Pinhole::pinholeInverse (double x, double y, double b, double d, double y_aux)
{
	std::vector <double> distances(2,-999999);
	
	// Define pinhole parameters	
	double fx=6.1521434211923247*100;
	double fy=6.1304053696870778*100;

	double cx=3.2930780975300314*100;
	double cy=2.3273913761751615*100;
	
	// Calcule pinhole model
	double z_aux=fy*y_aux/(y-cy);
    double x_aux=(x-cx)*z_aux/fx;
	
	// return pinhole value if it is between 0 an 40m in absolute value
	if (std::abs(z_aux)<y_aux*20)
	{
		distances[0]=x_aux*100;
		distances[1]=z_aux*100;
	}

	return distances;
}

std::vector<double> Pinhole::pinhole (double x, double y, double z)
{
	std::vector <double> pixels(2);
	
	// Define pinhole parameters	
	double fx=6.1521434211923247*100;
	double fy=6.1304053696870778*100;

	double cx=3.2930780975300314*100;
	double cy=2.3273913761751615*100;
	
	// Calcule pinhole model
	pixels[0]=fx*x/z+cx;
	pixels[1]=fy*y/z+cy; 
	
	return pixels;
}
