#ifndef MPC_FORMATION
#define MPC_FORMATION
#include"../../include/asrobot/MPCController.h"
class MPC_Formation{

    public:
     MPC_Formation();
     //对于参加编队的每个机器人来说,MPC_Formation可以选择不同的参数值
     MPC_Formation(const DyModel  ASRmodel, const float  samplingtime,
                   const int Np, const int Nc, const int X_num, const int U_num, const int   Y_num,
                   bool relax_mode, const VectorXd  umax, const VectorXd umin);
     MPC_Formation(const DyModel ASRmodel, const float samplingtime,
                   const int Np, const int Nc, const int X_num, const int U_num, const int Y_num,
                   bool relax_mode, const VectorXd umax, const VectorXd umin,
                   const VectorXd dumax, const VectorXd dumin);
     MPC_Formation(const DyModel ASRmodel, const float samplingtime,
                   const int Np, const int Nc, const int X_num, const int U_num, const int Y_num,
                   bool relax_mode, const VectorXd umax, const VectorXd umin, const VectorXd dumax,
                   const VectorXd dumin, const VectorXd ymax, const VectorXd ymin);
     void Update_referInfo(int iter_num, VectorXd referInfo1,VectorXd referInfo2, VectorXd referInfo3);
     void Update_currentState(VectorXd currentState1, VectorXd currentState2,VectorXd currentState3);
     void Update_last_U(VectorXd last_U1, VectorXd last_U2, VectorXd last_U3);

     void mult_Init_self_num(int ID);
     void set_AdjacentMatrix(MatrixXd AdjacentMatrix);
     void set_Y_num(int mult_Y_num);

     void set_Current_state();
     void set_state_piao();
     void set_PSI_Matrix();

     void set_Y_predict_Linear1();
     void set_Y_predice_Linear2();

     void set_R_Matrix();

     void set_PHIMatrix_r();
     void set_PHIMatrix_other();

     void set_THETAMatrix_r();
     void set_THETAMatrix_other();

     void set_GAMMAMatrix_r();
     void set_GAMMAMatrix_other();




     Eigen::MatrixXd mult_ComputeHessianMatrix(const Eigen::MatrixXd matrix_Q,
                                               const Eigen::MatrixXd matrix_Q_i,
                                               const Eigen::MatrixXd matrix_Q_j,
                                               double Row);


    Eigen::MatrixXd mult_ComputeGradientMatrix(const Eigen::MatrixXd matrix_Q,
                                              const Eigen::MatrixXd matrix_Q_i,
                                              const Eigen::MatrixXd matrix_Q_j);

   Eigen::MatrixXd mult_ComputeGradientMatrix(const Eigen::MatrixXd matrix_Q,VectorXd Yita_ref,
                                              const Eigen::MatrixXd matrix_Q_i,
                                              const Eigen::MatrixXd matrix_Q_j,
                                              VectorXd Y_error1,VectorXd Y_error2);

   VectorXd formation_OSQP_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix, MatrixXd A, VectorXd Upbound, VectorXd Lowbound, int max_iter, const double eps_abs);
   VectorXd formation_qpOASES_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix,VectorXd Lowbound, VectorXd Upbound,  int max_iter);
   VectorXd formation_qpOASES_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix, MatrixXd A_cons, VectorXd lbA, VectorXd ubA,
                                VectorXd Lowbound, VectorXd Upbound, int max_iter);



private:
     MPCController Robot1;
     MPCController Robot2;
     MPCController Robot3;

     VectorXd last_u1_;
     VectorXd last_u2_;
     VectorXd last_u3_;

     bool relax_mode_;
     bool delta_mode_;

     MatrixXd AdjacentMatrix_;
     int mult_Y_num_;
     int self_id_;        //id of self
     int i_id_;           // id of other robot
     int j_id_;
     int Nc_;
     int Np_;
     int U_num_;
     int X_num_;
     MatrixXd Current_state;
     MatrixXd State_piao;
     MatrixXd PSI_Matrix;
     MatrixXd Y_predict_Linear1;//针对线性化方式1的时候的其他机器人预测输出
     MatrixXd Y_predict_Linear2;

     MatrixXd R_Matrix;
     MatrixXd PHIMatrix_r;
     MatrixXd PHIMatrix_other;
     MatrixXd THETAMatrix_r;
     MatrixXd THETAMatrix_other;
     MatrixXd GAMMAMatrix_r;
     MatrixXd GAMMAMatrix_other;





};


#endif // MPC_FORMATION

