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
/@linear_mode is a bool variable, if it is 1, which represents the linear method is the first method in the 3 Chapter.else it is the secon linear method
/@ referInfo_ need to provide the reference linear velocity on X axis, the yaw angle velocity, yaw angle, which are used for update the matrix A, and B in the state space
/@current_state_ is the current state of the robot.when the second linear method is adopt, the current_state_is used for update the matrix A and B
/@ umax is the max value of the control input
/@ umin is the min value of the control input
/@ dumax is the max value of the increment of the control input
/@ dumin is the min value of the increment of the control input
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
        MPCController(const DyModel ASRmodel, const float samplingtime, const int Np, const int Nc, const int X_num, const int U_num, const int Y_num, bool relax_mode, const VectorXd umax, const VectorXd umin);
        MPCController(const DyModel ASRmodel, const float samplingtime, const int Np, const int Nc, const int X_num, const int U_num, const int Y_num, bool relax_mode, const VectorXd umax, const VectorXd umin, const VectorXd dumax, const VectorXd dumin);
        MPCController(const DyModel ASRmodel, const float samplingtime, const int Np, const int Nc, const int X_num, const int U_num, const int Y_num, bool relax_mode, const VectorXd umax, const VectorXd umin, const VectorXd dumax, const VectorXd dumin, const VectorXd ymax, const VectorXd ymin);
        void set_linear_mode(int mode);


        //the following three update function need to adopt in every loop
        void UpdateVector_refInfo(int iter_num, MatrixXd refTrajectory);
        void UpdateVector_Last_u(VectorXd last_u);
        void UpdateVector_CurrentState(VectorXd currentState);
        /***************************************************************************************************************************************************************************/

        Eigen::MatrixXd DebugQMatrix(double q1, double q2, double q3,double q4,double q5);
        //Eigen::MatrixXd DebugRMatrix(double r1, double r2, double r3,double r4,double r5);
        Eigen::MatrixXd ComputeHessianMatrix(const Eigen::MatrixXd &matrix_Q);
        Eigen::MatrixXd ComputeHessianMatrix(const Eigen::MatrixXd &matrix_Q, double Ro); // with relax factor
        //Eigen::VectorXd ComputeGradientMatrix(const Eigen::MatrixXd &matrix_Q,const Eigen::VectorXd X_piao);
        Eigen::VectorXd ComputeGradientMatrix(const Eigen::MatrixXd &matrix_Q, const Eigen::VectorXd X_piao);
        Eigen::VectorXd ComputeGradientMatrix(const Eigen::MatrixXd &matrix_Q, const Eigen::VectorXd Yita_ref,int mark);//the Yita_ref is Np*Ny vector; represents the reference information in the Np predictive horizon                                                                                                         //the parameter "int mark" has no real meaning,which is just used to be different with the above function ComputeGradientMatrix
        /*************************************************************************************************matrix in the constraints*************************************************/
        //applied to generating constraints matrix in the nodelta MPC
        MatrixXd UpdateInputConstraint_Matrix_Aue();
        VectorXd UpdateInputConstraint_Matrix_Upue();
        VectorXd UpdateInputConstraint_Matrix_Lowue();
        // applied to generating constraints matrix in the delta MPC
        MatrixXd d_UpdateInputConstraint_Matrix_Aue();
        VectorXd d_UpdateInputConstraint_Matrix_Upue();
        VectorXd d_UpdateInputConstraint_Matrix_Lowue();
        //
        MatrixXd d_UpdateInputConstraint_Matrix_Aue_qpOASES();
        VectorXd d_UpdateInputConstraint_Matrix_Upue_qpOASES();
        VectorXd d_UpdateInputConstraint_Matrix_Lowue_qpOASES();
        //applied to generating out constraint matrix
        MatrixXd UpdateOutputConstraint_Matrix_Aue();
        VectorXd UpdateOutputConstraint_Matrix_Upue();
        VectorXd UpdateOutputConstraint_Matrix_Lowue();
        /****************************************************************************compute the control command********************************************************************************/
        VectorXd Mpc_OSQP_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix, MatrixXd A, VectorXd Upbound, VectorXd Lowbound, int max_iter, const double eps_abs);
        VectorXd Mpc_qpOASES_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix,VectorXd Lowbound, VectorXd Upbound,  int max_iter);
        VectorXd Mpc_qpOASES_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix, MatrixXd A_cons, 
                                     VectorXd Lowbound, VectorXd Upbound, 
                                     VectorXd lbA, VectorXd ubA,int max_iter);

        //--------------------------------------need to be adopt in the f_single_robot_control-----------------------------------------------------------------/
         VectorXd generate_Yita_ref(int iter, MatrixXd reftrajectory);
         void Update(int iter_num,MatrixXd refTrajectory,VectorXd last_u,VectorXd currentState);
         VectorXd Update_Control_cmd(Eigen::MatrixXd matrix_Q, double Ro,Eigen::VectorXd X_piao,int solve_mode);
         VectorXd Update2_Control_cmd(Eigen::MatrixXd matrix_Q, double Ro,int iter, MatrixXd reftrajectory,int solve_mode);


         //**************************************************************************************************************************//
         //==================================================applied to  multiple robots formation===============================================//

         MatrixXd mult_ComputeTHETAMatrix(int mult_Y_num_i);
         Eigen::MatrixXd mult_ComputePHIMatrix(int mult_Y_num_i);
         Eigen::MatrixXd mult_ComputeGAMMAMatrix(int mult_Y_num_i);






        Eigen::MatrixXd UpdateStateMatrixA();
        Eigen::MatrixXd UpdateStateMatrixB();
        Eigen::MatrixXd UpdateStateMatrixC();
        Eigen::MatrixXd d_UpdateStateMatrixA();
        Eigen::MatrixXd d_UpdateStateMatrixB();
        Eigen::MatrixXd d_UpdateStateMatrixC();
        Eigen::MatrixXd InitQMatrix();
        Eigen::MatrixXd ComputeRMatrix();
        Eigen::MatrixXd ComputePHIMatrix();
        Eigen::MatrixXd ComputeTHETAMatrix();
        Eigen::VectorXd Compute_d_piao();
        Eigen::MatrixXd ComputeGAMMAMatrix();

        VectorXd referInfo_;       //the dimension of referInfo_ should be consistence with the X_num
        VectorXd currentState_;
        VectorXd last_u_;




	
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
        int linear_mode_;            //only set as 1 or 2. default it is 1. can change it by set function.if it is 1, the linear method adopt the first. if is is 2, the linear method adopt the second. refer to the Chapter3
        VectorXd umax_;
        VectorXd umin_;
        VectorXd dumax_;
        VectorXd dumin_;

        VectorXd ymax_;
        VectorXd ymin_;

	 
};

