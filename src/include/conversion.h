/*
 * conversion.h
 * Coversion from RGB space colour to HSV or Lab
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

#ifndef CONVERSION_H
#define CONVERSION_H

#include <iostream>
#include <vector>

class Conversion
{
public:
Conversion();
~Conversion();
/**
	 * Function to convert RGB space colour to HSV
	 * @param r R value of RGB space colour
	 * @param g G value of RGB space colour
	 * @param b B value of RGB space colour
	 * @return vector with HSV values
	 * */
std::vector<double> rgb2hsv(double r, double g, double b);

/**
	 * Function to convert RGB space colour to Lab
	 * @param r R value of RGB space colour
	 * @param g G value of RGB space colour
	 * @param b B value of RGB space colour
	 * @return vector with Lab values
	 * */
std::vector<int> rgb2lab(double r, double g, double blue);

};

#endif // CONVERSION_H
