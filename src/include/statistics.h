/*
 * statistics.h
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

#ifndef STATISTICS_H
#define STATISTICS_H

#include <vector>
#include "matrix.h"
#include "gauss.h"

class Statistics
{
public:
Statistics();
~Statistics();

/**
	* Function to calcule mean value
	* @param datas input datas
	* @return means value
	* */
double mean (std::vector <double> datas);

/**
	* Function to calcule the standard deviation
	* @param datas input datas
	* @param mean means value
	* @return standard deviation value
	* */
double stdev (std::vector <double> datas, double mean);

/**
	* Function to calcule the covariance value of a position of the covariances matrix
	* @param data_inputs input datas
	* @return covariance value of a position of the covariances matrix
	* */
double covLine (std::vector <Matrix_double> data_inputs);

/**
	* Function to calcule covariances matrix
	* @param datas input datas
	* @return covariances matrix
	* */
std::vector<Matrix_double> cov (std::vector <Matrix_double> datas);

/**
	* Function to calcule the covariance value of a position of the covariances matrix with means as input
	* @param data_inputs input datas
	* @param mean means value
	* @return standard deviation value
	* */
double covLineMeans (std::vector<Matrix_double> data_inputs, std::vector<double> means);

/**
	* Function to calcule covariances matrix with means as input
	* @param datas input datas
	* @param mean means value
	* @return covariances matrix
	* */
std::vector<Matrix_double> covMeans (std::vector<Matrix_double> datas, std::vector<double> means);

/**
	* Function to calcule inverse matrix
	* @param datas input matrix
	* @return inverse matrix
	* */
std::vector<Matrix_double> inv (std::vector <Matrix_double> datas);

/**
	* Function to calcule Mahalanobis distance between a Gaussian and a point
	* @param covariances_invert inverse covariances matrix of Gaussian
	* @param means means values of Gaussian
	* @param points point value
	* @return Mahalanobis distance
	* */
double mahalanobisDistance2Point(std::vector<Matrix_double> covariances_invert, std::vector <double> means, std::vector <double> points);

/**
	* Function to calcule Mahalanobis distance between two Gaussians
	* @param covariances_invert1 inverse covariances matrix of Gaussian 1
	* @param means1 means values of Gaussian 1
	* @param covariances_invert2 inverse covariances matrix of Gaussian 2
	* @param means2 means values of Gaussian 2
	* @return Mahalanobis distance
	* */
double mahalanobisDistance(std::vector<Matrix_double> covariances_invert1, std::vector <double> means1, std::vector<Matrix_double> covariances_invert2, std::vector <double> means2);

};

#endif // STATISTICS_H
