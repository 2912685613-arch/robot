#ifndef MPC_H
#define MPC_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include<iostream>
#include"qpOASES.hpp"

using namespace std;
using namespace Eigen;
using namespace qpOASES;

class MPC {
public:
	MPC();

	void set_state_CoeffMatrix();
	void set_steplength(int m1, int p1);

	//vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
	float Solve(MatrixXf Su, MatrixXf Sx,float reference);
	float QpSolverset(MatrixXf Su, MatrixXf Sx, float reference, MatrixXf A, float c_yaw);
	//float QpSolver(MatrixXf H, MatrixXf g, MatrixXf A, MatrixXf lb, MatrixXf ub, MatrixXf lbA, MatrixXf ubA, int nWSR);
	MatrixXf ComputeSu();
	MatrixXf ComputeSx();
	MatrixXf ComputeConstrain(MatrixXf Su);
	void stateset(float currentyaw);

private:
    //离散化后的状态方程的系数矩阵
	Matrix2f Ad;
	Vector2f Bd;
	Vector2f mat;
	MatrixXf Cd;
	 int p;                         //预测步长
	 int  m;                         //控制输入步数

	float u_1;                     //输入的初始值
	Vector2f X_1;                  //状态向量的初始值，维数由Ad矩阵的列数决定
	float lastyaw;
	
	
	MatrixXf R;
	MatrixXf Q;

   //约束条件
	float u_min;
	float u_max;
	float Ymin;
	float Ymax;
};

#endif /* MPC_H */
