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

#ifndef PINHOLE_H
#define PINHOLE_H

#include <iostream>
#include <vector>

class Pinhole
{
public:
Pinhole();
~Pinhole();

/**
	* Function to calcule real position of a pixel using pinhole inverse model. Only returns the position if the result is located between 0 and 40m in absolute value
	* @param x row parameter of pixel
	* @param y column parameter of pixel
	* @param b b parameter of input surface
	* @param d d parameter of input surface
	* @return vector with pixel real position on milimeters
	* */
std::vector<double> pinholeInverse (double x, double y, double b, double d, double y_aux);

/**
	* Function to calcule pixel position of a real position.
	* @param x real position on horizontal axis
	* @param y real position on vertical axis
	* @param z real depth value
	* @return vector with real position on image pixels
	* */
std::vector<double> pinhole(double x, double y, double z);
};

#endif // PINHOLE_H
