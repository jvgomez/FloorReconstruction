/*
 * learning.cpp
 * Set of functions to perfrom the analysis and learning of Gaussians functions
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

#include "../include/learning.h"
#include "../include/statistics.h"
#include "../include/algebra.h"

#include <iostream>
#include <algorithm>

Learning::Learning() {}

Learning::~Learning() {}

Gauss Learning::updateGaussians(std::vector<Matrix_double> covariances_invert1, std::vector<Matrix_double> covariances1, std::vector<double> means1, double mass1, std::vector<Matrix_double> covariances_invert2, std::vector<Matrix_double> covariances2, std::vector<double> means2, double mass2)
{
	Gauss gauss;
	std::vector<Matrix_double> covariances_invert;
	std::vector<Matrix_double> covariances;
	std::vector <double> means(means1.size());
	
	/* If Guassian learned is higher than 5000 points the Gaussian is considered saturated and it won't be updated, so it is necessary to difference 
	 * between a not saturated Gaussian (mass2<5000) and it have to be updated or saturated and it have not
	 * */
	
	if (mass2<5000)
	{
		double mass=mass1+mass2;
		#pragma omp parallel 
		{
			#pragma omp for shared (gauss, covariances_invert, covariances, means, mass) private (i, j, m1, m2 ) reduction(+: sum) 
			{
				for (int i=0; i<covariances_invert1.size(); i++)
				{
					Matrix_double m1;
					for (int j=0; j<covariances_invert1[i].columns.size(); j++)
						m1.columns.push_back((covariances1[i].columns[j]+covariances2[i].columns[j])/mass); // update covariances matrix
					covariances.push_back(m1);
				}
			}
		}
        Algebra algebra;
        covariances_invert=algebra.inv(covariances); // update covariances_invert
		
		#pragma omp parallel 
		{
			#pragma omp for shared (means, means1, means2) private (i) reduction(+: sum) 
			{
				for (int i=0; i<means1.size(); i++)
				{
					means[i]=(means1[i]+means2[i])/mass; // update means
				}
			}
		}
		
		// If update mass is higher than saturation mass it will be saturated
		if (mass>5000)
			mass=5000;
		
		// Define object
		gauss.means=means;
		gauss.covariances=covariances;
		gauss.covariances_invert=covariances_invert;
		gauss.mass=mass;
	}
	
	else
	{
		// Define object with saturated Gaussian
		gauss.means=means2;
		gauss.covariances=covariances2;
		gauss.covariances_invert=covariances_invert2;
		gauss.mass=mass2;
	}
	
	return gauss;
}

std::vector <Gauss> Learning::sortGaussians(std::vector <Gauss> gaussians, int gaussians_max)
{	
	std::vector <double> mass(gaussians.size());
	
	// sort mass values
	#pragma omp parallel 
	{
		#pragma omp for shared (mass, gaussians) private (i) reduction(+: sum) 
		{
			for (int i=0; i<gaussians.size(); i++)
			{
				mass[i]=gaussians[i].mass;
			}
		}
	}
	
	std::sort (mass.begin(), mass.end());
	
	int limit;
	
	if(gaussians.size()>gaussians_max)
		limit=gaussians_max;
	else
		limit=gaussians.size();
	
	std::vector <Gauss> gaussians_learned (limit);
	int cont=0;
	
	// sort gaussians position from higher to lower mass
	#pragma omp parallel 
	{
		#pragma omp for shared (gaussians, gaussians_learned, mass, limit) private (i, j) reduction(+: sum) 
		{
			for (int i=0; i<gaussians.size(); i++)
			{
				for (int j=0; j<limit; j++)
				{
					if (gaussians[i].mass==mass[j])
					{
						gaussians_learned[cont]=gaussians[i];
						cont++;
					}
				}
			}
		}
	}
	
	return gaussians_learned;
}

std::vector <Gauss> Learning::compareGaussians(std::vector<Gauss> gaussians, std::vector<Gauss> gaussians_learned)
{
	std::vector<Gauss> gaussians_final;
	
	#pragma omp parallel 
	{
		#pragma omp for shared (gaussians, gaussians_learned, gaussians_final) private (i,j ) reduction(+: sum) 
		{
			for (int i=0; i<gaussians.size(); i++)
			{
				for (int j=0; j<gaussians_learned.size(); j++)
				{
					Statistics statistics;
					if (statistics.mahalanobisDistance(gaussians[i].covariances_invert, gaussians[i].means, gaussians_learned[j].covariances_invert, gaussians_learned[j].means)<0.6) // calcule if the Gaussians are close
					{
						gaussians_final.push_back(updateGaussians(gaussians[i].covariances_invert, gaussians[i].covariances, gaussians[i].means, gaussians[i].mass, gaussians_learned[j].covariances_invert, gaussians_learned[j].covariances, gaussians_learned[j].means, gaussians_final[j].mass)); // update Gaussians
					}
				}
			}
		}
	}
	
	return gaussians_final;
}

std::vector <Gauss> Learning::initLearning(std::vector<Gauss> gaussians, int gaussians_max)
{
	std::vector<Gauss> gaussians_final;
	
	std::vector<double> mass (gaussians.size());
	
	// Perform the first learning
	#pragma omp parallel 
	{
		#pragma omp for shared (mass, gaussians) private (i) reduction(+: sum) 
		{
			for (int i=0; i<gaussians.size(); i++)
			{
				mass[i]=gaussians[i].mass;
			}
		}
	}
	
	std::sort (mass.begin(), mass.end(), std::greater<int>());
	
	if(gaussians.size()<gaussians_max)
	{
		#pragma omp parallel 
		{
			#pragma omp for shared (gaussians, gaussians_learned, mass) private (i, j) reduction(+: sum) 
			{
				for (int i=0; i<gaussians.size(); i++)
				{
					if (gaussians[i].mass>0.1*mass[0])
					{
						gaussians_final.push_back(gaussians[i]);
					}
				}
			}
		}
	}
	
	else
	{
		#pragma omp parallel 
		{
			#pragma omp for shared (gaussians, gaussians_final, mass, gaussians_max) private (i, j) reduction(+: sum) 
			{
				for (int i=0; i<gaussians_max; i++)
				{
					for (int j=0; j<gaussians.size(); j++)
					{
						if(gaussians[j].mass==mass[i])
						{
							gaussians_final.push_back(gaussians[j]);
						}
					}
				}
			}
		}
	}
	
	return gaussians_final;
}

std::vector <Gauss> Learning::learnGaussians(std::vector<Gauss> gaussians, std::vector <Gauss> gaussians_learned, int gaussians_max)
{
	std::vector<Gauss> gaussians_final;
	
	if (gaussians_learned.size()==0)
	{
		gaussians_final=initLearning(gaussians, gaussians_max);
	}
	
	else
	{
		std::vector <Gauss> gaussians_aux=compareGaussians(gaussians, gaussians_learned);
		gaussians_final=sortGaussians(gaussians_aux, gaussians_max);
	}
	
	std::cout << "Gaussians learning finished" << std::endl;
	
	return gaussians_final;
}
