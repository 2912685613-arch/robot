#include"../../include/asrobot/qpOASES_solver.h"
qpOASES_solver::qpOASES_solver()
{
}

qpOASES_solver::qpOASES_solver(Eigen::MatrixXd Hessian, Eigen::MatrixXd Gradient, Eigen::VectorXd lb, Eigen::VectorXd ub, int max_iter)
	:matrix_Hessian(Hessian),
	matrix_Gradient(Gradient),
	vector_lb_(lb),
	vector_ub_(ub),
	mat_iter_(max_iter)
	
{
	inequality_constrain_mode_ = 0;
	matrix_A_cons = Eigen::MatrixXd::Zero(2, 2);
	vector_lbA_ = Eigen::VectorXd::Zero(2);
	vector_ubA_ = Eigen::VectorXd::Zero(2);
}
qpOASES_solver::qpOASES_solver(Eigen::MatrixXd Hessian, Eigen::MatrixXd Gradient, Eigen::MatrixXd A_cons,
                               Eigen::VectorXd lb, Eigen::VectorXd ub, 
							   Eigen::VectorXd lbA, Eigen::VectorXd ubA, int max_iter)
							  :matrix_Hessian(Hessian),
							  matrix_Gradient(Gradient),
							  matrix_A_cons(A_cons),
							  vector_lbA_(lbA),
							  vector_ubA_(ubA),
							  vector_lb_(lb),
							  vector_ub_(ub),
							  mat_iter_(max_iter)
{
	inequality_constrain_mode_ = 1;
}
std::vector<double> qpOASES_solver::solve()
{

	//the matrix is translted into the format required by qpOASES
	//the number rows of Hessian , ub,lb ,Gradient is same, whihc is the number of the optimized variables ans is same as Nc*Nu(horizon of )
	/****************************************************************************************************************************************/
	std::cout<<"------------------------coming intio the qpOASES solver----------------------------"<<std::endl;
	const auto HNumOfMatrixElements = matrix_Hessian.rows() * matrix_Hessian.cols();  //��ά������ܴ�С
	double * h_matrix = new double[HNumOfMatrixElements];  // NOLINT 
	const auto GNumOfOffsetRows = matrix_Gradient.rows();
	double *g_matrix = new double[GNumOfOffsetRows];  // 
	
	int index = 0;

	for (int r = 0; r < matrix_Hessian.rows(); ++r) {  // matrix_Hessian.rows()
		g_matrix[r] = matrix_Gradient(r, 0);
		for (int c = 0; c <matrix_Hessian.cols(); ++c) {
			h_matrix[index++] = matrix_Hessian(r, c); 
			//std::cout << "hessian"<<
			
		}
	}

	/****************************************************** the bound of the param is translated********************************************/
	const auto ubNumOfVectorElements = vector_ub_.rows();
	double *ub_vector = new double[ubNumOfVectorElements];
	double *lb_vector = new double[ubNumOfVectorElements];
	for (int r = 0; r < vector_ub_.rows(); ++r)
	{
		ub_vector[r] = vector_ub_(r, 0);
		lb_vector[r] = vector_lb_(r, 0);
	}
	/************************************************************ parameters of qpOASES setting***********************************************/
	int num_param = matrix_Hessian.rows();
	std::cout<<"num_para of qpoases is :"<<num_param<<std::endl;
	std::cout<<"num_lb of qpoase is :"<<vector_lb_.rows()<<std::endl;
	std::cout<<"num_ub of qpoase is :"<<vector_ub_.rows()<<std::endl;
	qpOASES::Options options;
	options.printLevel = qpOASES::PL_LOW;

	//save the solution of the optimization problem
	qpOASES::real_t *xOpt= new qpOASES::real_t [num_param];
     std::cout<<"inequality_constrain_mode_:"<<inequality_constrain_mode_<<std::endl;

	///******************************************************** if there is no inequality constrains**********************************************/
	if (!inequality_constrain_mode_)
	{
		qpOASES::QProblemB solveQP(num_param);
		solveQP.setOptions(options);
		qpOASES::returnValue status = solveQP.init(h_matrix, g_matrix, lb_vector, ub_vector, mat_iter_);
		//std::cout << "************���״̬*****************" << status << std::endl;
		if (status!=0)
		{
			std::cout << "--------------------------the qpOASES optimization result is invalid-----------------------------------" << std::endl;

		}
		solveQP.getPrimalSolution(xOpt);
	}

	///****************************** if there is inequatily constrains, constrains matrix is translated***********************************************/
	if (inequality_constrain_mode_)
	{
       //std::cout<<"matrix_A_cons"<<matrix_A_cons<<std::endl;
      // std::cout<<"vector_ubA_"<<vector_ubA_<<std::endl;
	   //std::cout<<"vector_lbA_"<<vector_lbA_<<std::endl;
		const auto A_consNumOfMatrixElements = matrix_A_cons.rows()*matrix_A_cons.cols();
		double *A_cons_matrix = new double[A_consNumOfMatrixElements];
		const auto ubANumOfVectorElements = vector_ubA_.rows();
		double *ubA_vector = new double[ubANumOfVectorElements];
		double *lbA_vector = new double[ubANumOfVectorElements];
		index = 0;
		for (int r = 0; r < matrix_A_cons.rows(); ++r) {
			ubA_vector[r] = vector_ubA_(r, 0);
			lbA_vector[r] = vector_lbA_(r, 0);
			for (int c = 0; c < matrix_A_cons.cols(); ++c) {
				A_cons_matrix[index++] = matrix_A_cons(r, c);          //h����Ϊ���и�ֵ
			}
		}
		int num_constrain = matrix_A_cons.rows();
		std::cout<<"num_constraints of qpoases:"<<num_constrain<<std::endl;
		std::cout<<"col of Acons is:"<<matrix_A_cons.cols()<<std::endl;
		std::cout<<"num_lbA of qpoase is :"<<vector_lbA_.rows()<<std::endl;
	    std::cout<<"num_ubA of qpoase is :"<<vector_ubA_.rows()<<std::endl;
		qpOASES::QProblem solveQP(num_param, num_constrain);
		solveQP.setOptions(options);

		qpOASES::returnValue status = solveQP.init(h_matrix, g_matrix, A_cons_matrix, lb_vector, ub_vector, lbA_vector, ubA_vector, mat_iter_);
		//std::cout << "���״̬" << status<< std::endl;
		if (status != 0)
		{
			std::cout << "--------------------------the qpOASES optimization result is invalid-----------------------------------" << std::endl;

		}
		solveQP.getPrimalSolution(xOpt);
		delete[]A_cons_matrix;
		delete[]lbA_vector;
		delete[]ubA_vector;
	}
	//save the solution of the optimizaiton
	std::vector<double> control_cmd(num_param);
	for (int i = 0; i < num_param; i++)
	{
		control_cmd[i] = xOpt[i];
		//std::cout << control_cmd[i] << std::endl;
	}

	// delete the pointer

	delete[]h_matrix;
	delete[]g_matrix;
	delete[]lb_vector;
    delete[]ub_vector;
	delete[]xOpt;

	return control_cmd;



}