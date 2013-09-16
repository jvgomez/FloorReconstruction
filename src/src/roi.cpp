/*
 * roi.cpp
 * Interface to calculate the Region of Interest of a floor
 * Part of Floor Segmentation project
 * Copyright (C) 2013 Jose Pardeiro <jose.pardeiro@gmailinepoints.com>
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
#include "../include/roi.h"
#include "../include/pinhole.h"

ROI::ROI() {}

ROI::~ROI() {}

int ROI::detectWalls (double a1, double d1, double a2, double d2)
{
	if ((0.9 < std::abs(a1) && std::abs(a1) < 1) && (0.9 < std::abs(a2) && std::abs(a2) < 1))
	{			
		if(d2 < 0 && d1 > 0) 
			return 1;

		else 
			return -1;
	}
	
	else
		return -1;
}

std::vector <Intersection> ROI::intersectionPlanes(std::vector<Surface> surfaces)
{
	std::vector <Intersection> intersections(2);
	
	// Calcule intersection points of planes
	#pragma omp parallel 
	{
		#pragma omp for shared (surfaces, intersections) private (i) reduction(+: sum) 
		{
			for (int i=0; i<3; i++)
			{
				intersections[0].vectorial_products.push_back(0);
				intersections[0].points.push_back(0);
				
				intersections[1].vectorial_products.push_back(0);
				intersections[1].points.push_back(0);
			}
		}
	}
	
	#pragma omp parallel 
	{
		#pragma omp for shared (intersections, surfaces) private (i ) reduction(+: sum) 
		{
	
			for (int i=0; i<2; i++)
			{
				intersections[i].vectorial_products[0] = surfaces[0].b*surfaces[i+1].c-surfaces[0].c*surfaces[i+1].b;
				intersections[i].vectorial_products[1] = surfaces[0].c*surfaces[i+1].a-surfaces[0].a*surfaces[i+1].c;
				intersections[i].vectorial_products[2] = surfaces[0].a*surfaces[i+1].b-surfaces[0].b*surfaces[i+1].a;

				double num=((surfaces[0].d*(surfaces[i+1].a-surfaces[i+1].b))/(surfaces[0].a-surfaces[0].b))-surfaces[i+1].d;
				double den=(surfaces[i+1].c-(surfaces[0]).c*(surfaces[i+1].a-surfaces[i+1].b)/(surfaces[0].a-surfaces[0].b)); 
				intersections[i].points[2]= num/den;
				intersections[i].points[0]=(-surfaces[0].d-surfaces[0].c*intersections[i].points[2])/(surfaces[0].a-surfaces[0].b);
				intersections[i].points[1]=-intersections[i].points[0];
			}
		}
	}

	return intersections;
}

std::vector<Line> ROI::calculeLines(std::vector<Intersection> intersections)
{
	std::vector<Line> lines(2);

	// Calcule bi-dimensional projection of 3D lines
	#pragma omp parallel 
	{
		#pragma omp for shared (fx, fy, cx, cy, lines, intersections) private (i, positions1, positions2, x1, y1, z1, x2, y2, z2, t1, t2, m, b ) reduction(+: sum) 
		{
			for (int i=0; i<2; i++)
			{
				std::vector <double> positions1, positions2;
				
				#pragma omp parallel sections // starts a new team
				{
					
					#pragma omp section
					{
						// Calcule a bidimensional line using pinhole model from a random point of a 3D line
						double t1=5;
						
						double x1=intersections[i].points[0]+intersections[i].vectorial_products[0]*t1;
						double y1=intersections[i].points[1]+intersections[i].vectorial_products[1]*t1;
						double z1=intersections[i].points[2]+intersections[i].vectorial_products[2]*t1;
						
						Pinhole p;
						positions1=p.pinhole(x1, y1, z1);
					}

					#pragma omp section
					{
						// Calcule a bidimensional line using pinhole model from a random point of a 3D line
						double t2=15;

						
						double x2=intersections[i].points[0]+intersections[i].vectorial_products[0]*t2;
						double y2=intersections[i].points[1]+intersections[i].vectorial_products[1]*t2;
						double z2=intersections[i].points[2]+intersections[i].vectorial_products[2]*t2;
						
						Pinhole p;
						positions2=p.pinhole(x2, y2, z2);
					}

				}
				
				// Calcule 2D line parameters
				double m=(positions1[1]-positions2[1])/(positions1[0]-positions2[0]);
				double b=positions1[1]-m*positions1[0];

				lines[i].a=m;
				lines[i].b=b;
			}
		}
	}
	
	return lines;
}

std::vector<int> ROI::extractIndex(std::vector<Line> lines)
{
	// Calcule lines intersection
	std::vector <int> index;
	double xIntersection=(lines[1].b-lines[0].b)/(lines[0].a-lines[1].a);
	
	double y1=lines[0].a*xIntersection+lines[0].b;
	
	double y_max;
	
	if (y1>0 && y1<480)
	{
		y_max=y1;
	}
	
	else
		y_max=0;
	
	// Calcule pixels located between two lines
	#pragma omp parallel 
	{
		#pragma omp for shared (y_max, index) private (i, y, x1, x ) reduction(+: sum) 
		{		
			for (int i=0; i<640*480; i++)
			{
				int y=i/640;
				int x1=i%640;
				
				if (y>y_max)
				{
					double x_max, x_min;
					std::vector <double> x (2,0);
					
					for (int k=0; k<2; k++)
					{
						x[k]=(y-lines[k].b)/lines[k].a;
					}

					if (x[0]>x[1])
					{
						x_max=x[0];
						x_min=x[1];
					}

					else
					{
						x_max=x[1];
						x_min=x[0];
					}
					
					if (x1<x_max && x1>x_min)
					{
						index.push_back(i);
					}
				
				}
			}
		}
	}
	
	return index;
}

std::vector <Line> ROI::roi(std::vector<Surface> surfaces)
{
	int value=detectWalls(surfaces[1].a, surfaces[1].d, surfaces[2].a, surfaces[2].d);
	
	if (value==1)
	{
		std::vector<Intersection> intersections=intersectionPlanes(surfaces);
		std::vector<Line> lines=calculeLines(intersections);

		std::cout << "ROI finished" << std::endl;
		
		return lines;
	}
	else
	{
		std::cout << "Error, no walls detected" << std::endl;
		
		std::exit(EXIT_FAILURE);
	}
}

std::vector <int> ROI::extractFloorIdx(double a, double b, double c, double d, pcl::PointCloud< pcl::PointXYZRGBA >::ConstPtr cloud)
{
	float distance;
    
    std::vector <int> index;

	#pragma omp parallel 
	{
		#pragma omp for shared (distance, index, cloud) private (i) reduction(+: sum) 
		{
			for (int i=0; i<cloud->width * cloud->height; i++)
			{
				distance = (std::abs(a*cloud->points[i].x+b*cloud->points[i].y+c*cloud->points[i].z+d)/sqrt(a*a+b*b+c*c));

				// If distance is <0.1 suposse it is part of floor
				if (distance < 0.1)
				{
					index.push_back(i);
				}
			}
		}
	}
    
    return index;
}

std::vector <LinePoint> ROI::calculeLinePoints(std::vector<Line> lines)
{
	std::vector <LinePoint> linePoints;
	
	float xIntersection=(lines[1].b-lines[0].b)/(lines[0].a-lines[1].a);
	
	// Calcule intersection point between two lines
	float y1=lines[0].a*xIntersection+lines[0].b;
	
	// Calcule begin and end point of a line. The function diferences between an intersection point located inside the image and an intersection point located outside
	if (y1>0 && y1<480)
    {
		#pragma omp parallel 
		{
			#pragma omp for shared (xIntersection, y1, lines) private (i) reduction(+: sum) 
			{
				for (int i=0; i<2; i++)
				{
					LinePoint linepoint;

					linepoint.y.push_back(y1);
					linepoint.x.push_back((y1-lines[i].b)/lines[i].a);
					
					linepoint.y.push_back(480);
					linepoint.x.push_back((480-lines[i].b)/lines[i].a);

					linePoints.push_back(linepoint);
				}
			}
		}
    }

    else
    {
		#pragma omp parallel 
		{
			#pragma omp for shared (xIntersection, y1, lines) private (i) reduction(+: sum) 
			{
				for (int i=0; i<2; i++)
				{
					LinePoint linepoint;

					linepoint.y.push_back(480);
					linepoint.x.push_back((480-lines[i].b)/lines[i].a);
					
					linepoint.y.push_back(0);
					linepoint.x.push_back((0-lines[i].b)/lines[i].a);

					linePoints.push_back(linepoint);
				}
			}
		}
    }
    
    return linePoints;
}