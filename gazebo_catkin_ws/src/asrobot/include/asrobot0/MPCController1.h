#include<iostream>
#include"ASRModel.h"
#include"OSQP_solver.h"
#include"qpOASES_solver.h"
/*
/@ ASRmodel is a struct including the mass matrix and hydrodynamic matrix
/@ X_num is the number of state variables
/@ U_num is the number of input variables
/@ Y_num is the number of output variables
/@ Np is the predictive horizon
/@ Nc is the control horizon
/@ relax_mode is a bool variable, if it is true, which represents that the cost function contains a relax factor term
/@ referInfo need to provide the reference linear velocity on X axis, the yaw angle velocity, yaw angle, which are used for update the matrix A, and B in the state space
/@ umax is the max value of the control input
/@ umin is the min value of the control input
/@ dumax is the max value of the increment of the control input
/@ dumin is the min value of the increment of the contro input
/@ last_u is the control cmd of the last time
/@ ymax is the max value of the output
/@ ymin is the min value of the input
*/
/************************************How to use this class**************************************************************************/
/* first, define a ASRmodel,Np,Nc, X_num,Y_num,U_num, umax,umin,dumax,dumin, last_u,currentState, ymax,ymin
           DebugQMatrix
		   for (int i=1;i<100;++i)
		   {
                   Update refInfo;
		   ComputeHessianMatrix;
		   DebugQMatrix(double q1, double q2, double q3);//if need, adopt the parameter in the Q matrix
		   ComputeGradientMatrix(const Eigen::MatrixXd &matrix_Q, const Eigen::VectorXd X_piao)// pay attention on the X_piao, if the mpc is the delta mode, this X_piao should be Kesi
		   MpcSolve
		   }
*/
class MPCController
{

public:
    MPCController();
	MPCController(const DyModel &ASRmodel, const float &samplingtime, const int &Np, const int &Nc, const int &X_num, const int &U_num, const int &Y_num, bool relax_mode, const VectorXd &umax, const VectorXd &umin);
	MPCController(const DyModel &ASRmodel, const float &samplingtime, const int &Np, const int &Nc, const int &X_num, const int &U_num, const int &Y_num, bool relax_mode, const VectorXd &umax, const VectorXd &umin, const VectorXd &dumax, const VectorXd &dumin, const VectorXd &last_u);
	MPCController(const DyModel &ASRmodel, const float &samplingtime, const int &Np, const int &Nc, const int &X_num, const int &U_num, const int &Y_num, bool relax_mode, const VectorXd &umax, const VectorXd &umin, const VectorXd &dumax, const VectorXd &dumin, const VectorXd &last_u, const VectorXd &ymax, const VectorXd &ymin, const VectorXd &currentState);
	void UpdateVector_refInfo(int iter_num, MatrixXd refTrajectory);
	void UpdateVector_Last_u(VectorXd last_u);
	void UpdateVector_CurrentState(VectorXd currentState);
	//Eigen::MatrixXf ComputeQMatrix( int Np, int Y_num);
	Eigen::MatrixXd DebugQMatrix(double q1, double q2, double q3,double q4,double q5);
	Eigen::MatrixXd ComputeHessianMatrix(const Eigen::MatrixXd &matrix_Q);
	Eigen::MatrixXd ComputeHessianMatrix(const Eigen::MatrixXd &matrix_Q, double Ro); // with relax factor
	//Eigen::VectorXd ComputeGradientMatrix(const Eigen::MatrixXd &matrix_Q,const Eigen::VectorXd X_piao);
	Eigen::VectorXd ComputeGradientMatrix(const Eigen::MatrixXd &matrix_Q, const Eigen::VectorXd X_piao);
	/*************************************************************************************************matrix in the constraints*************************************************/
	//applied to generating constraints matrix in the nodelta MPC
	MatrixXd UpdateInputConstraint_Matrix_Aue();
	VectorXd UpdateInputConstraint_Matrix_Upue();
	VectorXd UpdateInputConstraint_Matrix_Lowue();
	// applied to generating constraints matrix in the delta MPC
	MatrixXd d_UpdateInputConstraint_Matrix_Aue();
	VectorXd d_UpdateInputConstraint_Matrix_Upue();
	VectorXd d_UpdateInputConstraint_Matrix_Lowue();
	//applied to generating out constraint matrix
	MatrixXd UpdateOutputConstraint_Matrix_Aue();
	VectorXd UpdateOutputConstraint_Matrix_Upue();
	VectorXd UpdateOutputConstraint_Matrix_Lowue();
	/****************************************************************************compute the control command********************************************************************************/
	VectorXd Mpc_OSQP_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix, MatrixXd A, VectorXd Upbound, VectorXd Lowbound, int max_iter, const double eps_abs);
	VectorXd Mpc_qpOASES_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix,VectorXd Lowbound, VectorXd Upbound,  int max_iter);
	VectorXd Mpc_qpOASES_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix, MatrixXd A_cons, VectorXd lbA, VectorXd ubA,
	                             VectorXd Lowbound, VectorXd Upbound, int max_iter);	
private:
	Eigen::MatrixXd UpdateStateMatrixA();
	Eigen::MatrixXd UpdateStateMatrixB();
	Eigen::MatrixXd UpdateStateMatrixC();
	Eigen::MatrixXd InitQMatrix();
	Eigen::MatrixXd ComputeRMatrix();
	Eigen::MatrixXd ComputePHIMatrix();
	Eigen::MatrixXd ComputeTHETAMatrix();
	
private:
	 DyModel ASRModel_;
	 float samplingtime_;
	 int Np_;
	 int Nc_;
	 int X_num_;
	 int Y_num_;
	 int U_num_;
	bool relax_mode_;             // if it is the mode with the relax factor
	bool delta_mode_;             // if it is true which presents the mpc is the delta mode
	VectorXd umax_;
	VectorXd umin_;
	VectorXd dumax_;
	VectorXd dumin_;
	VectorXd referInfo_;       //the dimension of referInfo_ should be consistence with the X_num
	VectorXd currentState_;
	VectorXd last_u_;
	VectorXd ymax_;
	VectorXd ymin_;
	 
};

