#include"../../include/asrobot/OSQP_solver.h"
MpcOsqp::MpcOsqp( Eigen::MatrixXd matrix_P,  Eigen::VectorXd matrix_q,  Eigen::VectorXd matrix_u,  Eigen::VectorXd matrix_l,  Eigen::MatrixXd matrix_Ac,
				int max_iter,  double eps_abs)
	: matrix_P_(matrix_P), 
	matrix_q_(matrix_q),
	matrix_u_(matrix_u),
	matrix_l_(matrix_l),
	matrix_Ac_(matrix_Ac),
	max_iteration_(max_iter),
	eps_abs_(eps_abs)
{
	
}
//********************************compute the standard quadratic problem coefficient matrix**************************************************************//
// compute the matrix in the osqp 
OSQPData * MpcOsqp::CalculateData( )
{
	OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));
	int col_ = matrix_P_.cols(); // number of the variables
	int row_ = matrix_Ac_.rows(); // number of the constraints
	//change the matrix_P to the CSC format
	std::vector<c_float>P_data; 
	std::vector<c_int> P_indices;
	std::vector<c_int>P_indptr;
	P_indptr.push_back(0);
	int nz_num=0;
	for (int i = 0; i < col_; i++)
	{
		for (int j = 0; j < i+1; j++)
		{
			if (matrix_P_(j, i)!= 0)
			{
				P_data.push_back(matrix_P_(j, i));
				P_indices.push_back(j);
				nz_num = nz_num + 1;
			}
		}
		P_indptr.push_back(nz_num);
	}

	//change the matrix_Ac to the CSC format
	std::vector<c_float>A_data;
	std::vector<c_int> A_indices;
	std::vector<c_int>A_indptr;
	A_indptr.push_back(0);
	int nz_numA = 0;
	for (int i = 0; i < matrix_Ac_.cols(); i++)
	{
		for (int j = 0; j < matrix_Ac_.rows(); j++)
		{
			if (matrix_Ac_(j, i) != 0)
			{
				A_data.push_back(matrix_Ac_(j, i));
				A_indices.push_back(j);
				nz_numA = nz_numA + 1;
			}
		}
		A_indptr.push_back(nz_numA);
	}

	if (data) 
	{
		data->n = col_;
		data->m = row_;
		data->P = csc_matrix(data->n, data->n, P_data.size(), CopyData(P_data), CopyData(P_indices), CopyData(P_indptr));
		data->q = matrix_q_.data();
		data->A = csc_matrix(data->m, data->n, A_data.size(), CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
		data->u = matrix_u_.data();
		data->l = matrix_l_.data();

	}
	return data;
}

OSQPSettings *MpcOsqp::Settings() {
	// default setting
	OSQPSettings *settings =
		reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
	if (settings == nullptr) {
		return nullptr;
	}
	else {
		osqp_set_default_settings(settings);
		settings->polish = true;
		settings->scaled_termination = true;
		settings->verbose = false;
		settings->max_iter = max_iteration_;
		settings->eps_abs = eps_abs_;
		return settings;
	}
}


void MpcOsqp::FreeData(OSQPData *data) {
	c_free(data->A);
	c_free(data->P);
	c_free(data);
}

std::vector<double>  MpcOsqp::Solve() {

	
	OSQPData *data = CalculateData();
	int num_var = matrix_P_.cols();
	std::vector<double> control_cmd(num_var);
	//std::cout << "OSQP data done"<<endl;
	//std::cout << "OSQP data n:" << data->n<<endl;
	//std::cout << "OSQP data m:" << data->m<<endl;
	// for (int i = 0; i < data->n; ++i) {
	// 	std::cout << "OSQP data q" << i << ":" << (data->q)[i]<<endl;
	// }
	
	// for (int i = 0; i < data->m; ++i) {
	// 	std::cout << "OSQP data l" << i << ":" << (data->l)[i]<<endl;
	// }
	
	// for (int i = 0; i < data->m; ++i) {
	// 	std::cout << "OSQP data u" << i << ":" << (data->u)[i]<<endl;
	// }

	OSQPSettings *settings = Settings();
	//std::cout << "OSQP setting done" << endl;
	OSQPWorkspace *osqp_workspace = nullptr;
    osqp_setup(&osqp_workspace, data, settings);
	//osqp_workspace = osqp_setup(data, settings);
	//std::cout << "OSQP workspace ready"<<endl;
	osqp_solve(osqp_workspace);

	auto status = osqp_workspace->info->status_val;
	//std::cout << "status:" << status<<endl;
	// check status
	if (status < 0 || (status != 1 && status != 2)) {
		std::cout << "failed optimization status:\t" << osqp_workspace->info->status << endl;
		osqp_cleanup(osqp_workspace);
		FreeData(data);
		c_free(settings);
	
	}
	else if (osqp_workspace->solution == nullptr) {
	//	std::cout << "The solution from OSQP is nullptr"<<endl;
		osqp_cleanup(osqp_workspace);
		FreeData(data);
		c_free(settings);
		
	}

	for (size_t i = 0; i < num_var; ++i) {
		control_cmd[i]=osqp_workspace->solution->x[i];
		//std::cout << "control_cmd:" << "i" << ":" << control_cmd[i]<<endl;
	}

	// Cleanup
	osqp_cleanup(osqp_workspace);
	FreeData(data);
	c_free(settings);
	return control_cmd;
}
