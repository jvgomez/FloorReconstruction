#include "../include/algebra.h"

#include <cmath>
#include <omp.h>

Algebra::Algebra() {}
Algebra::~Algebra() {}

std::vector <Matrix_double> Algebra::rotx (double alpha)
{
    std::vector <Matrix_double> matrixRotx(3);
    std::vector <double> row (3,0);

    row[0]=1;
    row[1]=0;
    row[2]=0;

    matrixRotx[0].columns=row;

    row[0]=0;
    row[1]=cos(alpha);
    row[2]=-sin(alpha);

    matrixRotx[1].columns=row;

    row[1]=sin(alpha);
    row[2]=cos(alpha);

    matrixRotx[2].columns=row;

    return matrixRotx;
}

std::vector <Matrix_double> Algebra::roty (double alpha)
{
    std::vector <Matrix_double> matrixRoty(3);
    std::vector <double> row (3,0);

    row[0]=cos(alpha);
    row[1]=0;
    row[2]=sin(alpha);

    matrixRoty[0].columns=row;

    row[0]=0;
    row[1]=1;
    row[2]=0;

    matrixRoty[1].columns=row;

    row[0]=-sin(alpha);
    row[1]=0;
    row[2]=cos(alpha);

    matrixRoty[2].columns=row;

    return matrixRoty;
}

std::vector <Matrix_double> Algebra::rotz (double alpha)
{
    std::vector <Matrix_double> matrixRotz(3);
    std::vector <double> row (3,0);

    row[0]=cos(alpha);
    row[1]=-sin(alpha);
    row[2]=0;

    matrixRotz[0].columns=row;

    row[0]=sin(alpha);
    row[1]=cos(alpha);
    row[2]=0;

    matrixRotz[1].columns=row;

    row[0]=0;
    row[1]=0;
    row[2]=1;

    matrixRotz[2].columns=row;

    return matrixRotz;
}

std::vector <double> Algebra::productMatrix3x1 (std::vector <Matrix_double> matrix, std::vector <double> vector)
{
    std::vector <double> result(3,0);

    #pragma omp parallel
    {
        #pragma omp for shared (result, matrix) private (i) reduction(+: sum)
        {
            for (int i=0; i<3; i++)
                result[i]=matrix[i].columns[0]*vector[0]+matrix[i].columns[1]*vector[1]+matrix[i].columns[2]*vector[2];
        }
    }

    return result;
}

std::vector <Matrix_double> Algebra::productMatrix3x3 (std::vector <Matrix_double> matrix1, std::vector <Matrix_double> matrix2)
{
    std::vector <Matrix_double> result(3);

    #pragma omp parallel
    {
        #pragma omp for shared (result, matrix) private (i, auxiliar) reduction(+: sum)
        {
            for (int i=0; i<3; i++)
                for (int j=0; j<3; j++)
                {
                    double auxiliar=matrix1[i].columns[0]*matrix2[0].columns[j]+matrix1[i].columns[1]*matrix2[1].columns[j]+matrix1[i].columns[2]*matrix2[2].columns[j];
                    result[i].columns.push_back(auxiliar);
                }
        }
    }

    return result;
}

double Algebra::angleVectors (std::vector <double> vector1, std::vector <double> vector2)
{
    double alpha=acos((vector1[0]*vector2[0]+vector1[1]*vector2[1]+vector1[2]*vector2[2])/(std::sqrt(vector1[0]*vector1[0]+vector1[1]*vector1[1]+vector1[2]*vector1[2])*std::sqrt(vector2[0]*vector2[0]+vector2[1]*vector2[1]+vector2[2]*vector2[2])));

    return alpha;
}

std::vector<Matrix_double> Algebra::inv (std::vector <Matrix_double> datas)
{
    std::vector<Matrix_double> inverse;

    Matrix_double cc2;
    Matrix_double cc3;
    Matrix_double cc4;

    double determinant=datas[0].columns[0]*datas[1].columns[1]*datas[2].columns[2]+
    datas[0].columns[1]*datas[1].columns[2]*datas[2].columns[0]+
    datas[0].columns[2]*datas[2].columns[1]*datas[1].columns[0]-
    datas[0].columns[2]*datas[1].columns[1]*datas[2].columns[0]-
    datas[0].columns[1]*datas[1].columns[0]*datas[2].columns[2]-
    datas[0].columns[0]*datas[1].columns[2]*datas[2].columns[1];

    cc2.columns.push_back((datas[1].columns[1]*datas[2].columns[2]-datas[1].columns[2]*datas[2].columns[1])/determinant);
    cc2.columns.push_back(-(datas[0].columns[1]*datas[2].columns[2]-datas[2].columns[1]*datas[0].columns[2])/determinant);
    cc2.columns.push_back((datas[0].columns[1]*datas[1].columns[2]-datas[1].columns[1]*datas[0].columns[2])/determinant);

    inverse.push_back(cc2);

    cc3.columns.push_back(-(datas[1].columns[0]*datas[2].columns[2]-datas[2].columns[0]*datas[1].columns[2])/determinant);
    cc3.columns.push_back((datas[0].columns[0]*datas[2].columns[2]-datas[2].columns[0]*datas[0].columns[2])/determinant);
    cc3.columns.push_back(-(datas[0].columns[0]*datas[1].columns[2]-datas[1].columns[0]*datas[0].columns[2])/determinant);

    inverse.push_back(cc3);

    cc4.columns.push_back((datas[1].columns[0]*datas[2].columns[1]-datas[2].columns[0]*datas[1].columns[1])/determinant);
    cc4.columns.push_back(-(datas[0].columns[0]*datas[2].columns[1]-datas[2].columns[0]*datas[0].columns[1])/determinant);
    cc4.columns.push_back((datas[0].columns[0]*datas[1].columns[1]-datas[0].columns[1]*datas[1].columns[0])/determinant);

    inverse.push_back(cc4);

    return inverse;
}
