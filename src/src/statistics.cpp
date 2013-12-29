/*
 * statistics.cpp
 * Set of functions designed to perform the statistic analysis
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

#include "../include/statistics.h"
#include "../include/algebra.h"

#include <numeric>
#include <cmath>
#include <iostream>
#include <omp.h>

Statistics::Statistics() {}

Statistics::~Statistics() {}

double Statistics::mean (std::vector <double> datas)
{
	double mean = std::accumulate(datas.begin(), datas.end(), 0.0)/ datas.size();
	
	return mean;
}

double Statistics::stdev (std::vector <double> datas, double mean)
{
	double temp=0;
	
	#pragma omp parallel 
	{
		#pragma omp for shared (datas, temp, mean) private (i) reduction(+: sum) 
		{
			for(int i=0; i<datas.size(); i++)
			{
					temp+=(datas[i]-mean)*(datas[i]-mean) ;
			}
		}
	}
	double stdev=sqrt(temp/(datas.size()-1));

	return stdev;
}

double Statistics::covLine (std::vector <Matrix_double> data_inputs)
{
	double temp=0;
	
	std::vector <double> means (2);
	
	#pragma omp parallel 
	{
		#pragma omp for shared (data_inputs, temp, means) private (i, j) reduction(+: sum) 
		{
			for (int j=0; j<data_inputs.size(); j++)
			{
				std::vector <double> datas;
				for (int i=0; i<data_inputs[j].columns.size(); i++)
				{
					datas.push_back(data_inputs[j].columns[i]);
				}
				
				means[j]=mean(datas);

			}
		}
	}
	
	#pragma omp parallel 
	{
		#pragma omp for shared (data_inputs, temp, means) private (i) reduction(+: sum) 
		{
			for(int i=0; i<data_inputs[0].columns.size(); i++)
			{
					temp+=(data_inputs[0].columns[i]-means[0])*(data_inputs[1].columns[i]-means[1]) ;
			}
		}
	}
	double line=temp/(data_inputs[0].columns.size()-1);
	
	return line;
}

std::vector<Matrix_double> Statistics::cov (std::vector <Matrix_double> datas)
{
	std::vector <Matrix_double> covariances;

	#pragma omp parallel 
	{
		#pragma omp for shared (datas, covariances) private (i, j, aux, data_inputs) reduction(+: sum) 
		{
			for (int i=0; i<datas[0].columns.size(); i++)
			{
				Matrix_double m;
				for (int j=0; j<datas[0].columns.size(); j++)
				{
					std::vector <Matrix_double> data_inputs(2);
					
					
					for (int aux=0; aux<datas.size(); aux++)
					{
							data_inputs[0].columns.push_back(datas[aux].columns[j]);
							data_inputs[1].columns.push_back(datas[aux].columns[i]);
					}
					m.columns.push_back(covLine(data_inputs));	
				}
				covariances.push_back(m);
			}
		}
	}
	
	return covariances;
}

double Statistics::covLineMeans (std::vector <Matrix_double> data_inputs, std::vector <double> means)
{
	
	double temp=0;
	
	#pragma omp parallel 
	{
		#pragma omp for shared (data_inputs, means) private (i) reduction(+: sum) 
		{
			for(int i=0; i<data_inputs[0].columns.size(); i++)
			{
					temp+=(data_inputs[0].columns[i]-means[0])*(data_inputs[1].columns[i]-means[1]) ;
			}
		}
	}
	
	double line=temp/(data_inputs[0].columns.size()-1);
	
	return line;
}

std::vector<Matrix_double> Statistics::covMeans (std::vector <Matrix_double> datas, std::vector <double> means)
{
	std::vector <Matrix_double> covariances;

	#pragma omp parallel 
	{
		#pragma omp for shared (datas, covariances, means) private (i, j, means_aux) reduction(+: sum) 
		{
			for (int i=0; i<datas[0].columns.size(); i++)
			{
				Matrix_double m;
				for (int j=0; j<datas[0].columns.size(); j++)
				{
					std::vector <Matrix_double> data_inputs(2);
					
					
					for (int aux=0; aux<datas.size(); aux++)
					{
							data_inputs[0].columns.push_back(datas[aux].columns[j]);
							data_inputs[1].columns.push_back(datas[aux].columns[i]);
					}
					std::vector <double> means_aux(2);
					means_aux[0]=means[j];
					means_aux[1]=means[i];
					m.columns.push_back(covLineMeans(data_inputs, means_aux));	
				}
				covariances.push_back(m);
			}
		}
	}
	
	return covariances;
}



double Statistics::mahalanobisDistance2Point(std::vector<Matrix_double> covariances_invert, std::vector <double> means, std::vector <double> points)
{
	std::vector <double> auxiliar (3,0);
	double mahalanobis=0;

	#pragma omp parallel 
	{
		#pragma omp for shared (auxiliar, mahalanobis, means, covariances_invert, points) private (i) reduction(+: sum) 
		{
			for (int i=0; i<3; i++)
				auxiliar[i]=(means[i]-points[i]);
		}
	}
	
	double a1, a2, a3;
	 #pragma omp parallel sections // starts a new team
	{
		#pragma omp section
		{
			a1=auxiliar[0]*covariances_invert[0].columns[0]+auxiliar[1]*covariances_invert[0].columns[1]+auxiliar[2]*covariances_invert[0].columns[2];
		}
		#pragma omp section
		{
			a2=auxiliar[0]*covariances_invert[1].columns[0]+auxiliar[1]*covariances_invert[1].columns[1]+auxiliar[2]*covariances_invert[1].columns[2];
		}
		#pragma omp section
		{
			a3=auxiliar[0]*covariances_invert[2].columns[0]+auxiliar[1]*covariances_invert[2].columns[1]+auxiliar[2]*covariances_invert[2].columns[2];
		}
	}
	
	mahalanobis=auxiliar[0]*a1+auxiliar[1]*a2+auxiliar[2]*a3;

	mahalanobis=std::sqrt(std::abs(mahalanobis));
	
	return mahalanobis;
}

double Statistics::mahalanobisDistance(std::vector<Matrix_double> covariances_invert1, std::vector <double> means1, std::vector<Matrix_double> covariances_invert2, std::vector <double> means2)
{
	std::vector <double> auxiliar (3,0);
	std::vector <Matrix_double> covariances_invert;
	double mahalanobis=0;
	
	#pragma omp parallel 
	{
		#pragma omp for shared (auxiliar, mahalanobis, means, covariances_invert1, covariances_invert2, covariances_invert, points) private (i) reduction(+: sum) 
		{
			for (int i=0; i<covariances_invert1.size(); i++)
			{
				Matrix_double m;
				for (int j=0; j<covariances_invert1[i].columns.size(); j++)
				{
					m.columns.push_back(covariances_invert1[i].columns[j]-covariances_invert2[i].columns[j]);
				}
				covariances_invert.push_back(m);
			}
		}
	}
	
	#pragma omp parallel 
	{
		#pragma omp for shared (auxiliar, mahalanobis, means, covariances_invert, points) private (i) reduction(+: sum) 
		{
			for (int i=0; i<3; i++)
				auxiliar[i]=(means1[i]-means2[i]);
		}
	}
	
	double a1, a2, a3;
	 #pragma omp parallel sections // starts a new team
	{
		#pragma omp section
		{
			a1=auxiliar[0]*covariances_invert[0].columns[0]+auxiliar[1]*covariances_invert[0].columns[1]+auxiliar[2]*covariances_invert[0].columns[2];
		}
		#pragma omp section
		{
			a2=auxiliar[0]*covariances_invert[1].columns[0]+auxiliar[1]*covariances_invert[1].columns[1]+auxiliar[2]*covariances_invert[1].columns[2];
		}
		#pragma omp section
		{
			a3=auxiliar[0]*covariances_invert[2].columns[0]+auxiliar[1]*covariances_invert[2].columns[1]+auxiliar[2]*covariances_invert[2].columns[2];
		}
	}
	
	mahalanobis=auxiliar[0]*a1+auxiliar[1]*a2+auxiliar[2]*a3;

	mahalanobis=std::sqrt(std::abs(mahalanobis));
	
	return mahalanobis;
}
