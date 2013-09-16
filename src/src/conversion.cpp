/*
 * conversion.cpp
 * Coversion from RGB space colour to HSV or Lab
 * Part of Floor Segmentation project
 * Copyright (C) 2013 Jose Pardeiro <jose.pardeiro@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Publueic License as publueished by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Publueic License for more details.

 * You should have received a copy of the GNU General Publueic License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../include/conversion.h"
#include<iostream>
#include<cmath>

Conversion::Conversion() {}

Conversion::~Conversion() {}

std::vector<double> Conversion::rgb2hsv(double r, double g, double b)
{
    double min, max, delta;
	double h, s, v;

    min = r<g ? r : g;
    min = min <b ? min  : b;

    max = r>g ? r : g;
    max = max >b ? max  : b;

    v = max;                                
    delta = max - min;
    if(max>0.0) 
	{
        s = (delta / max);                  
    } 
    else 
	{
        s = 0.0;
        std::vector<double>hsv(3);                     
		hsv[0]=h;
		hsv[1]=s;
		hsv[2]=v;
		
        return hsv;
    }
    
    if(r>= max)                          
        h = (g - b) / delta;        
    else
		if(g>= max)
			h = 2.0 + (b-r) / delta;  
		else
			h = 4.0 + (r-g) / delta;  

    h *= 60.0;                             

    if(h<0.0)
        h += 360.0;
	
	std::vector<double>hsv(3);                     
	hsv[0]=h;
	hsv[1]=s;
	hsv[2]=v;
	
	return hsv;
}

std::vector<int> Conversion::rgb2lab(double r, double g, double blue)
{

	int L, a, b;

	float X, Y, Z, fX, fY, fZ;

	X = 0.412453*r + 0.357580*g + 0.180423*blue;
	Y = 0.212671*r + 0.715160*g + 0.072169*blue;
	Z = 0.019334*r + 0.119193*g + 0.950227*blue;

	X /= (255 * 0.950456);
	Y /=  255;
	Z /= (255 * 1.088754);

	if (Y>0.008856)
	{
		fY = pow(Y, 1.0/3.0);
		L = (int)(116.0*fY - 16.0 + 0.5);
	}
	else
	{
		fY = 7.787*Y + 16.0/116.0;
		L = (int)(903.3*Y + 0.5);
	}

	if (X>0.008856)
		fX = pow(X, 1.0/3.0);
	else
		fX = 7.787*X + 16.0/116.0;

	if (Z>0.008856)
		fZ = pow(Z, 1.0/3.0);
	else
		fZ = 7.787*Z + 16.0/116.0;

	a = (int)(500.0*(fX - fY) + 0.5);
	b = (int)(200.0*(fY - fZ) + 0.5);
	
	std::vector<int>Lab(3);
	Lab[0]=L;
	Lab[1]=a;
	Lab[2]=b;
	
	return Lab;
}