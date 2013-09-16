/*
 * learning.h
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

#ifndef LEARNING_H
#define LEARNING_H

#include "gauss.h"

class Learning
{
public:
Learning();
~Learning();

/**
	* Function to performs the Gaussians learning
	* @param gaussians input Gauss vector with new Gaussians obtained on the last analysis
	* @param gaussians_learned input Gauss learned vector
	* @param gaussians_max maximum number of Gaussians which can be learned
	* @return Gauss vector of Gaussians parameters
	* */
std::vector <Gauss> learnGaussians(std::vector<Gauss> gaussians, std::vector<Gauss> gaussians_learned, int gaussians_max);

/**
	* Function to update Gaussians values
	* @param covariances_invert1 inverse covariances matrix of Gaussian 1
	* @param covariances1 covariances matrix of Gaussian 1
	* @param means1 values of means of Gaussian 1
	* @param mass1 number of points envolved by Gaussian 1
	* @param covariances_invert2 inverse covariances matrix of Gaussian 2
	* @param covariances2 covariances matrix of Gaussian 2
	* @param means2 values of means of Gaussian 2
	* @param mass2 number of points envolved by Gaussian 2
	* @return Gauss object which contains updated Gaussians parameters
	* */
Gauss updateGaussians(std::vector<Matrix_double> covariances_invert1, std::vector<Matrix_double> covariances1, std::vector<double> means1, double mass1, std::vector<Matrix_double> covariances_invert2, std::vector<Matrix_double> covariances2, std::vector<double> means2, double mass2);

/**
	* Function to perform the first learning
	* @param gaussians input Gauss vector with new Gaussians obtained on the last analysis
	* @param gaussians_max maximum number of Gaussians which can be learned
	* @return Gauss vector of Gaussians parameters learned by first time
	* */
std::vector <Gauss> initLearning(std::vector<Gauss> gaussians, int gaussians_max);

/**
	* Function to compare Gaussians parameters
	* @param gaussians input Gauss vector with new Gaussians obtained on the last analysis
	* @param gaussians_learned input Gauss learned vector
	* @return Gauss vector of Gaussians parameters
	* */
std::vector <Gauss> compareGaussians(std::vector< Gauss > gaussians, std::vector< Gauss > gaussians_learned);

/**
	* Function to sort Gaussians
	* @param gaussians input Gauss vector with new Gaussians obtained on the last analysis
	* @param gaussians_max maximum number of Gaussians which can be learned
	* @return Gauss vector of Gaussians parameters
	* */
std::vector <Gauss> sortGaussians(std::vector <Gauss> gaussians, int gaussians_max);
};

#endif // LEARNING_H
