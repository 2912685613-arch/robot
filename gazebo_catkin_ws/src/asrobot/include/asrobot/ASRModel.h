#ifndef ASRMODEL_H
#define ASRMODEL_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
using namespace Eigen;
typedef struct {
	Matrix2d  MassMatrix;         //mass matrix
	Matrix2d  HydroLinMatrix;     //hydrodynamic linear damping force matrix
	Matrix2d  HydroNonLinMatrix;  //hydrodynamic Nonlinear damping force matrix
	
} DyModel;
//typedef struct
//{
	
//	MatrixXd matrix_Aue;
//	VectorXd matrix_Upue;
//	VectorXd matrix_Lowue;
//}UeconstraintMatrix;
#endif
