#include<iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <osqp/osqp.h>
#include <vector>
using namespace std;

class MpcOsqp {
public:
	/**
	* @brief Solver for discrete-time model predictive control problem.
	* @param matrix_a The system dynamic matrix
	* @param matrix_b The control matrix
	* @the state space model is x(k+1)=matrix_a*x(k)+matrix_b*u(k);y(k)=matrix_c*x(k)
	* @param matrix_q The cost matrix for state
	* @param matrix_r The cost matrix for control
	* @param matrix_lower The lower bound control constrain matrix
	* @param matrix_upper The upper bound control constrain matrix
	* @param matrix_initial_state The initial state matrix
	* @param max_iter The maximum iterations
	*/
	MpcOsqp(Eigen::MatrixXd matrix_P,  Eigen::VectorXd matrix_q,  Eigen::VectorXd matrix_u, Eigen::VectorXd matrix_l,  Eigen::MatrixXd matrix_Ac,
		int max_iter,  double eps_abs);
	
	// control vector
	std::vector<double> Solve();

private:
    
	OSQPData * CalculateData();
	OSQPSettings *Settings();
	void FreeData(OSQPData *data);
	

	template <typename T>
	T *CopyData(const std::vector<T> &vec) {
		T *data = new T[vec.size()];
		memcpy(data, vec.data(), sizeof(T) * vec.size());
		return data;
	}

private:
	Eigen::MatrixXd matrix_P_;
	Eigen::MatrixXd matrix_Ac_;
	Eigen::VectorXd matrix_q_;
	Eigen::VectorXd matrix_u_;
	Eigen::VectorXd matrix_l_;
	int max_iteration_;
	double eps_abs_;
	int Nc_;
};
