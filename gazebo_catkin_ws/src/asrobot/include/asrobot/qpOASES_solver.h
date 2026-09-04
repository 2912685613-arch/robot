#include<iostream>
#include<qpOASES.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

//hcy
#include <vector>

class qpOASES_solver
{
public:
	qpOASES_solver();
	qpOASES_solver(Eigen::MatrixXd Hessian, Eigen::MatrixXd Gradient, Eigen::VectorXd lb, Eigen::VectorXd ub, int max_iter);
	qpOASES_solver(Eigen::MatrixXd Hessian, Eigen::MatrixXd Gradient, Eigen::MatrixXd A_cons,
                               Eigen::VectorXd lb, Eigen::VectorXd ub, 
							   Eigen::VectorXd lbA, Eigen::VectorXd ubA, int max_iter);
	std::vector<double> solve();

private:
	int mat_iter_;
	bool inequality_constrain_mode_;
	Eigen::MatrixXd matrix_Hessian;
	Eigen::MatrixXd matrix_Gradient;
	Eigen::MatrixXd matrix_A_cons;
	Eigen::VectorXd vector_ubA_;
	Eigen::VectorXd vector_lbA_;
	Eigen::VectorXd vector_ub_;
	Eigen::VectorXd vector_lb_;

};

