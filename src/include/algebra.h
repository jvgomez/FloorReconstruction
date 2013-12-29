#ifndef ALGEBRA_H
#define ALGEBRA_H

#include "matrix.h"

class Algebra
{
public:
    Algebra();
    ~Algebra();

    /**
        * Function to calcule rotatory matrix on x-axis
        * @param rotatory angle
        * @return rotatory matrix
        * */
    std::vector <Matrix_double> rotx (double alpha);

    /**
        * Function to calcule rotatory matrix on y-axis
        * @param rotatory angle
        * @return rotatory matrix
        * */
    std::vector <Matrix_double> roty (double alpha);

    /**
        * Function to calcule rotatory matrix on z-axis
        * @param rotatory angle
        * @return rotatory matrix
        * */
    std::vector <Matrix_double> rotz (double alpha);

    /**
        * Function to calcule product result of a 3x3 matrix and 3x1 matrix
        * @param 3x3 matrix
        * @param 3x1 matrix
        * @return product matrix
        * */
    std::vector <double> productMatrix3x1 (std::vector <Matrix_double> matrix, std::vector <double> vector);

    /**
        * Function to calcule product result of a 3x3 matrix and 3x3 matrix
        * @param 3x3 matrix
        * @param 3x3 matrix
        * @return product matrix
        * */
    std::vector <Matrix_double> productMatrix3x3 (std::vector <Matrix_double> matrix1, std::vector <Matrix_double> matrix2);

    /**
        * Function to calcule angle between two vectors
        * @param vector 1
        * @param vector 2
        * @return angle between vector 1 and vector 2
        * */
    double angleVectors (std::vector <double> vector1, std::vector <double> vector2);

    /**
        * Function to calcule inverse matrix
        * @param datas input matrix
        * @return inverse matrix
        * */
    std::vector<Matrix_double> inv (std::vector <Matrix_double> datas);
};

#endif // ALGEBRA_H
