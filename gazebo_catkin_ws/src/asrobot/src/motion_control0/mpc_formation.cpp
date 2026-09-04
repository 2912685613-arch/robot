#include "../../include/asrobot/mpc_formation.h"
/*****************the desired yaw should be presented with radian*****************/
/* u, du, du y,  Robot1,2,3,  x_refer  x_current  last_u 多机的自我初始化 AdjacentMatrix
/* 
/* square
/* sine
/*********************************************************************************/
MPC_Formation::MPC_Formation()
{
    delta_mode_ = 0;
}
MPC_Formation::MPC_Formation(const DyModel ASRmodel, const float samplingtime,
                             const int Np, const int Nc, const int X_num, const int U_num, const int Y_num,
                             bool relax_mode, const VectorXd umax, const VectorXd umin) 
    : Nc_(Nc),U_num_(U_num),X_num_(X_num), Np_(Np), relax_mode_(relax_mode)
{
    delta_mode_ = 0;
    Robot1 = MPCController(ASRmodel, samplingtime, Np, Nc, X_num, U_num, Y_num, relax_mode, umax, umin);
    Robot2 = MPCController(ASRmodel, samplingtime, Np, Nc, X_num, U_num, Y_num, relax_mode, umax, umin);
    Robot3 = MPCController(ASRmodel, samplingtime, Np, Nc, X_num, U_num, Y_num, relax_mode, umax, umin);
}
MPC_Formation::MPC_Formation(const DyModel ASRmodel, const float samplingtime,
                             const int Np, const int Nc, const int X_num, const int U_num, const int Y_num,
                             bool relax_mode, const VectorXd umax, const VectorXd umin,
                             const VectorXd dumax, const VectorXd dumin)
    : Nc_(Nc), U_num_(U_num), X_num_(X_num), Np_(Np), relax_mode_(relax_mode)
{
    delta_mode_ = 1;
    Robot1 = MPCController(ASRmodel, samplingtime, Np, Nc, X_num, U_num, Y_num, relax_mode, umax, umin, dumax, dumin);
    Robot2 = MPCController(ASRmodel, samplingtime, Np, Nc, X_num, U_num, Y_num, relax_mode, umax, umin, dumax, dumin);
    Robot3 = MPCController(ASRmodel, samplingtime, Np, Nc, X_num, U_num, Y_num, relax_mode, umax, umin, dumax, dumin);
}

MPC_Formation::MPC_Formation(const DyModel ASRmodel, const float samplingtime,
                             const int Np, const int Nc, const int X_num, const int U_num, const int Y_num,
                             bool relax_mode, const VectorXd umax, const VectorXd umin, const VectorXd dumax,
                             const VectorXd dumin, const VectorXd ymax, const VectorXd ymin)
    : Nc_(Nc), U_num_(U_num), X_num_(X_num), Np_(Np), relax_mode_(relax_mode)
{
    delta_mode_ = 1;
    Robot1 = MPCController(ASRmodel, samplingtime, Np, Nc, X_num, U_num, Y_num, relax_mode, umax, umin, dumax, dumin, ymax, ymin);
    Robot2 = MPCController(ASRmodel, samplingtime, Np, Nc, X_num, U_num, Y_num, relax_mode, umax, umin, dumax, dumin, ymax, ymin);
    Robot3 = MPCController(ASRmodel, samplingtime, Np, Nc, X_num, U_num, Y_num, relax_mode, umax, umin, dumax, dumin, ymax, ymin);
}
void MPC_Formation::Update_referInfo(int iter_num, VectorXd referInfo1, VectorXd referInfo2, VectorXd referInfo3)
{
    Robot1.UpdateVector_refInfo(iter_num, referInfo1);
    Robot2.UpdateVector_refInfo(iter_num, referInfo2);
    Robot3.UpdateVector_refInfo(iter_num, referInfo3);
}
void MPC_Formation::Update_currentState(VectorXd currentState1, VectorXd currentState2, VectorXd currentState3)
{
    Robot1.UpdateVector_CurrentState(currentState1);
    Robot2.UpdateVector_CurrentState(currentState2);
    Robot3.UpdateVector_CurrentState(currentState3);
}
void MPC_Formation::Update_last_U(VectorXd last_U1, VectorXd last_U2, VectorXd last_U3)
{
    Robot1.UpdateVector_Last_u(last_U1);
    Robot2.UpdateVector_Last_u(last_U2);
    Robot3.UpdateVector_Last_u(last_U3);
}
void MPC_Formation::mult_Init_self_num(int ID)
{
    self_id_ = ID;
}

void MPC_Formation::set_AdjacentMatrix(MatrixXd AdjacentMatrix)
{
    AdjacentMatrix_ = AdjacentMatrix;
}
void MPC_Formation::set_Y_num(int mult_Y_num)
{
    mult_Y_num_ = mult_Y_num;
}

void MPC_Formation::set_Current_state()
{
    Current_state = MatrixXd::Zero(X_num_, 3);
    Current_state.block(0, 0, X_num_, 1) = Robot1.currentState_;
    Current_state.block(0, 1, X_num_, 1) = Robot2.currentState_;
    Current_state.block(0, 2, X_num_, 1) = Robot3.currentState_;
}

void MPC_Formation::set_state_piao()
{
    State_piao = MatrixXd::Zero(X_num_, 3);
    State_piao.block(0, 0, X_num_, 1) = Robot1.currentState_ - Robot1.referInfo_; //每一列对应一个机器人的piao信息
    State_piao.block(0, 1, X_num_, 1) = Robot2.currentState_ - Robot2.referInfo_;
    State_piao.block(0, 2, X_num_, 1) = Robot3.currentState_ - Robot3.referInfo_;
}

void MPC_Formation::set_PSI_Matrix()
{
    VectorXd d_piao1 = Robot1.Compute_d_piao();
    VectorXd d_piao2 = Robot2.Compute_d_piao();
    VectorXd d_piao3 = Robot3.Compute_d_piao();
    int size_d_piao = d_piao1.rows();
    VectorXd PSI1 = VectorXd::Zero(Np_ * size_d_piao);
    VectorXd PSI2 = VectorXd::Zero(Np_ * size_d_piao);
    VectorXd PSI3 = VectorXd::Zero(Np_ * size_d_piao);
    for (int i = 0; i < Np_; i++)
    {
        PSI1.segment(i * size_d_piao, Np_) = d_piao1;
        PSI2.segment(i * size_d_piao, Np_) = d_piao2;
        PSI3.segment(i * size_d_piao, Np_) = d_piao3;
    }

    PSI_Matrix.block(0, 0, size_d_piao * Np_, 1) = PSI1;
    PSI_Matrix.block(0, 1, size_d_piao * Np_, 1) = PSI2;
    PSI_Matrix.block(0, 2, size_d_piao * Np_, 1) = PSI3;
}

void MPC_Formation::set_R_Matrix()
{
    MatrixXd R_Matrix1 = Robot1.ComputeRMatrix();
    MatrixXd R_Matrix2 = Robot2.ComputeRMatrix();
    MatrixXd R_Matrix3 = Robot3.ComputeRMatrix();
    int row_ = R_Matrix1.rows();
    int col_ = R_Matrix1.cols();
    R_Matrix.block(0, 0, row_, col_) = R_Matrix1;
    R_Matrix.block(row_, 0, row_, col_) = R_Matrix2;
    R_Matrix.block(2 * row_, 0, row_, col_) = R_Matrix3;
}

void MPC_Formation::set_PHIMatrix_r()
{
    MatrixXd PHIMatirx1_r = Robot1.ComputePHIMatrix();
    MatrixXd PHIMatrix2_r = Robot2.ComputePHIMatrix();
    MatrixXd PHIMatrix3_r = Robot3.ComputePHIMatrix();
    int row_ = PHIMatirx1_r.rows();
    int col_ = PHIMatirx1_r.cols();
    PHIMatrix_r.block(0, 0, row_, col_) = PHIMatirx1_r;
    PHIMatrix_r.block(row_, 0, row_, col_) = PHIMatrix2_r;
    PHIMatrix_r.block(2 * row_, 0, row_, col_) = PHIMatrix3_r;
}
void MPC_Formation::set_PHIMatrix_other()
{
    MatrixXd PHIMatirx1_other = Robot1.mult_ComputePHIMatrix(mult_Y_num_);
    MatrixXd PHIMatrix2_other = Robot2.mult_ComputePHIMatrix(mult_Y_num_);
    MatrixXd PHIMatrix3_other = Robot3.mult_ComputePHIMatrix(mult_Y_num_);
    int row_ = PHIMatirx1_other.rows();
    int col_ = PHIMatirx1_other.cols();
    PHIMatrix_other.block(0, 0, row_, col_) = PHIMatirx1_other;
    PHIMatrix_other.block(row_, 0, row_, col_) = PHIMatrix2_other;
    PHIMatrix_other.block(2 * row_, 0, row_, col_) = PHIMatrix3_other;
}

void MPC_Formation::set_THETAMatrix_r()
{
    MatrixXd THETAMatrix1_r = Robot1.ComputeTHETAMatrix();
    MatrixXd THETAMatrix2_r = Robot2.ComputeTHETAMatrix();
    MatrixXd THETAMatrix3_r = Robot3.ComputeTHETAMatrix();
    int row_ = THETAMatrix1_r.rows();
    int col_ = THETAMatrix1_r.cols();
    THETAMatrix_r.block(0, 0, row_, col_) = THETAMatrix1_r;
    THETAMatrix_r.block(row_, 0, row_, col_) = THETAMatrix2_r;
    THETAMatrix_r.block(2 * row_, 0, row_, col_) = THETAMatrix3_r;
}
void MPC_Formation::set_THETAMatrix_other()
{
    MatrixXd THETAMatrix1_other = Robot1.mult_ComputeTHETAMatrix(mult_Y_num_);
    MatrixXd THETAMatrix2_other = Robot2.mult_ComputeTHETAMatrix(mult_Y_num_);
    MatrixXd THETAMatrix3_other = Robot3.mult_ComputeTHETAMatrix(mult_Y_num_);
    int row_ = THETAMatrix1_other.rows();
    int col_ = THETAMatrix1_other.cols();
    THETAMatrix_other.block(0, 0, row_, col_) = THETAMatrix1_other;
    THETAMatrix_other.block(row_, 0, row_, col_) = THETAMatrix2_other;
    THETAMatrix_other.block(2 * row_, 0, row_, col_) = THETAMatrix3_other;
}
void MPC_Formation::set_GAMMAMatrix_r()
{
    MatrixXd GAMMAMatrix1_r = Robot1.ComputeGAMMAMatrix();
    MatrixXd GAMMAMatrix2_r = Robot2.ComputeGAMMAMatrix();
    MatrixXd GAMMAMatrix3_r = Robot3.ComputeGAMMAMatrix();
    int row_ = GAMMAMatrix1_r.rows();
    int col_ = GAMMAMatrix1_r.cols();
    GAMMAMatrix_r.block(0, 0, row_, col_) = GAMMAMatrix1_r;
    GAMMAMatrix_r.block(row_, 0, row_, col_) = GAMMAMatrix2_r;
    GAMMAMatrix_r.block(2 * row_, 0, row_, col_) = GAMMAMatrix3_r;
}
void MPC_Formation::set_GAMMAMatrix_other()
{
    MatrixXd GAMMAMatrix1_other = Robot1.mult_ComputeGAMMAMatrix(mult_Y_num_);
    MatrixXd GAMMAMatrix2_other = Robot2.mult_ComputeGAMMAMatrix(mult_Y_num_);
    MatrixXd GAMMAMatrix3_other = Robot3.mult_ComputeGAMMAMatrix(mult_Y_num_);
    int row_ = GAMMAMatrix1_other.rows();
    int col_ = GAMMAMatrix1_other.cols();
    GAMMAMatrix_other.block(0, 0, row_, col_) = GAMMAMatrix1_other;
    GAMMAMatrix_other.block(row_, 0, row_, col_) = GAMMAMatrix2_other;
    GAMMAMatrix_other.block(2 * row_, 0, row_, col_) = GAMMAMatrix3_other;
}

void MPC_Formation::set_Y_predict_Linear1()
{
    VectorXd Last_U1 = VectorXd::Zero(Nc_ * U_num_);
    VectorXd Last_U2 = VectorXd::Zero(Nc_ * U_num_);
    VectorXd Last_U3 = VectorXd::Zero(Nc_ * U_num_);
    for (int i = 0; i < Nc_; i++)
    {
        Last_U1.segment(i * U_num_, U_num_) = last_u1_;
        Last_U2.segment(i * U_num_, U_num_) = last_u2_;
        Last_U3.segment(i * U_num_, U_num_) = last_u3_;
    }
    int PHI_row_ = PHIMatrix_other.rows() / 3;
    int PHI_col_ = PHIMatrix_other.cols();
    int THETA_row_ = THETAMatrix_other.rows() / 3;
    int THETA_col_ = THETAMatrix_other.cols();
    VectorXd X1_piao = State_piao.block(0, 0, X_num_, 1);
    VectorXd X2_piao = State_piao.block(0, 1, X_num_, 1);
    VectorXd X3_piao = State_piao.block(0, 2, X_num_, 1);

    VectorXd Y_predict_1 = PHIMatrix_other.block(0, 0, PHI_row_, PHI_col_) * X1_piao + THETAMatrix_other.block(0, 0, THETA_row_, THETA_col_) * Last_U1;
    VectorXd Y_predict_2 = PHIMatrix_other.block(PHI_row_, 0, PHI_row_, PHI_col_) * X2_piao + THETAMatrix_other.block(THETA_row_, 0, THETA_row_, THETA_col_) * Last_U2;
    VectorXd Y_predict_3 = PHIMatrix_other.block(2 * PHI_row_, 0, PHI_row_, PHI_col_) * X3_piao + THETAMatrix_other.block(2 * THETA_row_, 0, THETA_row_, THETA_col_) * Last_U3;
    int row_ = Y_predict_1.rows();
    Y_predict_Linear1.block(0, 0, row_, 1) = Y_predict_1;
    Y_predict_Linear1.block(0, 1, row_, 1) = Y_predict_2;
    Y_predict_Linear1.block(0, 2, row_, 1) = Y_predict_3;
}
void MPC_Formation::set_Y_predice_Linear2()
{
    VectorXd d_piao1 = Robot1.Compute_d_piao();
    VectorXd d_piao2 = Robot2.Compute_d_piao();
    VectorXd d_piao3 = Robot3.Compute_d_piao();
    int size_d_piao = d_piao1.rows();
    VectorXd PSI1 = VectorXd::Zero(Np_ * size_d_piao);
    VectorXd PSI2 = VectorXd::Zero(Np_ * size_d_piao);
    VectorXd PSI3 = VectorXd::Zero(Np_ * size_d_piao);
    for (int i = 0; i < Np_; i++)
    {
        PSI1.segment(i * size_d_piao, Np_) = d_piao1;
        PSI2.segment(i * size_d_piao, Np_) = d_piao2;
        PSI3.segment(i * size_d_piao, Np_) = d_piao3;
    }
    VectorXd Last_U1 = VectorXd::Zero(Nc_ * U_num_);
    VectorXd Last_U2 = VectorXd::Zero(Nc_ * U_num_);
    VectorXd Last_U3 = VectorXd::Zero(Nc_ * U_num_);
    for (int i = 0; i < Nc_; i++)
    {
        Last_U1.segment(i * U_num_, U_num_) = last_u1_;
        Last_U2.segment(i * U_num_, U_num_) = last_u2_;
        Last_U3.segment(i * U_num_, U_num_) = last_u3_;
    }
    int PHI_row_ = PHIMatrix_other.rows() / 3;
    int PHI_col_ = PHIMatrix_other.cols();
    int THETA_row_ = THETAMatrix_other.rows() / 3;
    int THETA_col_ = THETAMatrix_other.cols();
    int GAMMA_row_ = GAMMAMatrix_other.rows() / 3;
    int GAMMA_col_ = GAMMAMatrix_other.cols();
    VectorXd Y_predict_1 = PHIMatrix_other.block(0, 0, PHI_row_, PHI_col_) * Robot1.currentState_ +
                           THETAMatrix_other.block(0, 0, THETA_row_, THETA_col_) * Last_U1 +
                           GAMMAMatrix_other.block(0, 0, GAMMA_row_, GAMMA_col_) * PSI1;
    VectorXd Y_predict_2 = PHIMatrix_other.block(PHI_row_, 0, PHI_row_, PHI_col_) * Robot2.currentState_ +
                           THETAMatrix_other.block(THETA_row_, 0, THETA_row_, THETA_col_) * Last_U2 +
                           GAMMAMatrix_other.block(GAMMA_row_, 0, GAMMA_row_, GAMMA_col_) * PSI2;
    VectorXd Y_predict_3 = PHIMatrix_other.block(2 * PHI_row_, 0, PHI_row_, PHI_col_) * Robot3.currentState_ +
                           THETAMatrix_other.block(2 * THETA_row_, 0, THETA_row_, THETA_col_) * Last_U3 +
                           GAMMAMatrix_other.block(2 * GAMMA_row_, 0, GAMMA_row_, GAMMA_col_);
    int row_ = Y_predict_1.rows();
    Y_predict_Linear2.block(0, 0, row_, 1) = Y_predict_1;
    Y_predict_Linear2.block(0, 1, row_, 1) = Y_predict_2;
    Y_predict_Linear2.block(0, 2, row_, 1) = Y_predict_3;
}

Eigen::MatrixXd MPC_Formation::mult_ComputeHessianMatrix(const Eigen::MatrixXd matrix_Q,
                                                         const Eigen::MatrixXd matrix_Q_i,
                                                         const Eigen::MatrixXd matrix_Q_j,
                                                         double Row)
{
    MatrixXd HessianMatrix;
    int THETA_r_row_ = THETAMatrix_r.rows() / 3;
    int THETA_r_col_ = THETAMatrix_r.cols();
    int THETA_other_row_ = THETAMatrix_other.rows() / 3;
    int THETA_other_col_ = THETAMatrix_other.cols();
    MatrixXd THETAMatrix_ref = THETAMatrix_r.block((self_id_ - 1) * THETA_r_row_, 0, THETA_r_row_, THETA_r_col_);
    MatrixXd THETAMatrix_i = THETAMatrix_other.block((self_id_ - 1) * THETA_other_row_, 0, THETA_other_row_, THETA_other_col_);
    MatrixXd THETAMatrix_j = THETAMatrix_i;
    int R_row_ = R_Matrix.rows() / 3;
    int R_col_ = R_Matrix.cols();
    MatrixXd matrix_R = R_Matrix.block((self_id_ - 1) * R_row_, 0, R_row_, R_col_);
    int size_R = matrix_R.cols();
    MatrixXd part_r = THETAMatrix_ref.transpose() * matrix_Q * THETAMatrix_ref + matrix_R;
    MatrixXd part_i = THETAMatrix_i.transpose() * matrix_Q_i * THETAMatrix_i;
    MatrixXd part_j = THETAMatrix_j.transpose() * matrix_Q_j * THETAMatrix_j;
    MatrixXd temp = 2 * (part_r +
                         AdjacentMatrix_(self_id_, i_id_) * MatrixXd::Identity(Nc_ * U_num_, Nc_ * U_num_) * part_i +
                         AdjacentMatrix_(self_id_, j_id_) * MatrixXd::Identity(Nc_ * U_num_, Nc_ * U_num_) * part_j);

    if (!relax_mode_)
    {
        HessianMatrix = temp;
    }
    if (relax_mode_)
    {
        HessianMatrix = MatrixXd::Zero(size_R + 1, size_R + 1);
        HessianMatrix.block(0, 0, size_R, size_R) = temp;
        HessianMatrix(size_R, size_R) = Row;
    }

    return HessianMatrix;
}

Eigen::MatrixXd MPC_Formation::mult_ComputeGradientMatrix(const Eigen::MatrixXd matrix_Q,
                                                          const Eigen::MatrixXd matrix_Q_i,
                                                          const Eigen::MatrixXd matrix_Q_j)
{
    VectorXd GradientMatrix;
    int THETA_r_row_ = THETAMatrix_r.rows() / 3;
    int THETA_r_col_ = THETAMatrix_r.cols();
    int THETA_other_row_ = THETAMatrix_other.rows() / 3;
    int THETA_other_col_ = THETAMatrix_other.cols();
    int PHI_r_row_ = PHIMatrix_r.rows() / 3;
    int PHI_r_col_ = PHIMatrix_r.cols();
    int PHI_other_row_ = PHIMatrix_other.rows() / 3;
    int PHI_other_col_ = PHIMatrix_other.cols();
    int Y_predict_row_ = Y_predict_Linear1.rows();

    MatrixXd THETAMatrix_ref = THETAMatrix_r.block((self_id_ - 1) * THETA_r_row_, 0, THETA_r_row_, THETA_r_col_);
    int col_ = THETAMatrix_ref.cols();
    MatrixXd THETAMatrix_i = THETAMatrix_other.block((self_id_ - 1) * THETA_other_row_, 0, THETA_other_row_, THETA_other_col_);
    MatrixXd THETAMatrix_j = THETAMatrix_i;

    MatrixXd PHIMatrix_ref = PHIMatrix_r.block((self_id_ - 1) * PHI_r_row_, 0, PHI_r_row_, PHI_r_col_);
    MatrixXd PHIMatrix_i = PHIMatrix_other.block((self_id_)*PHI_other_row_, 0, PHI_other_row_, PHI_other_col_);
    MatrixXd PHIMatrix_j = PHIMatrix_i;
    MatrixXd part_r = 2 * THETAMatrix_ref.transpose() * matrix_Q * PHIMatrix_ref * State_piao.block(0, self_id_ - 1, X_num_, 1);
    MatrixXd part_i = 2 * THETAMatrix_i.transpose() * matrix_Q_i * (PHIMatrix_i * State_piao.block(0, self_id_ - 1, X_num_, 1) - Y_predict_Linear1.block(0, i_id_ - 1, Y_predict_row_, 1));
    MatrixXd part_j = 2 * THETAMatrix_j.transpose() * matrix_Q_j * (PHIMatrix_j * State_piao.block(0, self_id_ - 1, X_num_, 1) - Y_predict_Linear1.block(0, j_id_ - 1, Y_predict_row_, 1));
    VectorXd temp = part_r +
                    AdjacentMatrix_(self_id_, i_id_) * MatrixXd::Identity(Nc_ * U_num_, Nc_ * U_num_) * part_i +
                    AdjacentMatrix_(self_id_, j_id_) * MatrixXd::Identity(Nc_ * U_num_, Nc_ * U_num_) * part_j;
    if (!relax_mode_)
    {
        GradientMatrix = temp;
    }
    if (relax_mode_)
    {
        GradientMatrix = VectorXd::Zero(col_ + 1);
        GradientMatrix.head(col_) = temp;
    }
    return GradientMatrix;
}

Eigen::MatrixXd MPC_Formation::mult_ComputeGradientMatrix(const Eigen::MatrixXd matrix_Q, VectorXd Yita_ref,
                                                          const Eigen::MatrixXd matrix_Q_i,
                                                          const Eigen::MatrixXd matrix_Q_j,
                                                          VectorXd Y_error_i, VectorXd Y_error_j)
{
    VectorXd GradientMatrix;
    int THETA_r_row_ = THETAMatrix_r.rows() / 3;
    int THETA_r_col_ = THETAMatrix_r.cols();
    int THETA_other_row_ = THETAMatrix_other.rows() / 3;
    int THETA_other_col_ = THETAMatrix_other.cols();
    int PHI_r_row_ = PHIMatrix_r.rows() / 3;
    int PHI_r_col_ = PHIMatrix_r.cols();
    int PHI_other_row_ = PHIMatrix_other.rows() / 3;
    int PHI_other_col_ = PHIMatrix_other.cols();

    int Y_predict_row_ = Y_predict_Linear2.rows();

    int PSI_row_ = PSI_Matrix.rows();

    int GAMMA_row_ = GAMMAMatrix_r.rows() / 3;
    int GAMMA_col_ = GAMMAMatrix_r.cols();

    MatrixXd THETAMatrix_ref = THETAMatrix_r.block((self_id_ - 1) * THETA_r_row_, 0, THETA_r_row_, THETA_r_col_);
    int col_ = THETAMatrix_ref.cols();
    MatrixXd THETAMatrix_i = THETAMatrix_other.block((self_id_ - 1) * THETA_other_row_, 0, THETA_other_row_, THETA_other_col_);
    MatrixXd THETAMatrix_j = THETAMatrix_i;
    MatrixXd PHIMatrix_ref = PHIMatrix_r.block((self_id_ - 1) * PHI_r_row_, 0, PHI_r_row_, PHI_r_col_);
    MatrixXd PHIMatrix_i = PHIMatrix_other.block((self_id_ - 1) * PHI_other_row_, 0, PHI_other_row_, PHI_other_col_);
    MatrixXd PHIMatrix_j = PHIMatrix_i;

    MatrixXd part_r = 2 * THETAMatrix_ref.transpose() * matrix_Q * (PHIMatrix_ref * Current_state.block(0, self_id_ - 1, X_num_, 1) + GAMMAMatrix_r.block((self_id_ - 1) * GAMMA_row_, 0, GAMMA_row_, GAMMA_col_) * PSI_Matrix.block(0, (self_id_ - 1), PSI_row_, 1) - Yita_ref);
    //Y_error_i=Yita_ref-Yita_ref_i;Y_error_j=Yita_ref-Yita_ref_j
    MatrixXd part_i = 2 * THETAMatrix_i.transpose() * matrix_Q_i * (PHIMatrix_i * Current_state.block(0, self_id_ - 1, X_num_, 1) + GAMMAMatrix_r.block((self_id_ - 1) * GAMMA_row_, 0, GAMMA_row_, GAMMA_col_) * PSI_Matrix.block(0, (self_id_ - 1), PSI_row_, 1) -(Y_predict_Linear2.block(0, i_id_ - 1, Y_predict_row_, 1) + Y_error_i));
    MatrixXd part_j = 2 * THETAMatrix_j.transpose() * matrix_Q_j * (PHIMatrix_j * Current_state.block(0, self_id_ - 1, X_num_, 1) + GAMMAMatrix_r.block((self_id_ - 1) * GAMMA_row_, 0, GAMMA_row_, GAMMA_col_) * PSI_Matrix.block(0, (self_id_ - 1), PSI_row_, 1) - (Y_predict_Linear2.block(0, j_id_ - 1, Y_predict_row_, 1) + Y_error_j));
    VectorXd temp = part_r +
                    AdjacentMatrix_(self_id_, i_id_) * MatrixXd::Identity(Nc_ * U_num_, Nc_ * U_num_) * part_i +
                    AdjacentMatrix_(self_id_, j_id_) * MatrixXd::Identity(Nc_ * U_num_, Nc_ * U_num_) * part_j;
    if (!relax_mode_)
    {
        GradientMatrix = temp;
    }
    if (relax_mode_)
    {
        GradientMatrix = VectorXd::Zero(col_ + 1);
        GradientMatrix.head(col_) = temp;
    }
    return GradientMatrix;
}

VectorXd MPC_Formation::formation_OSQP_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix, MatrixXd A, VectorXd Upbound, VectorXd Lowbound, int max_iter, const double eps_abs)
{
    VectorXd input = VectorXd::Zero(U_num_);
    MpcOsqp Osqp_prob(HessianMatrix, GradientMatrix, Upbound, Lowbound, A, max_iter, eps_abs);

    std::vector<double> control_total = Osqp_prob.Solve();
    //std::vector<double> control_cmd(U_num_);
    if (!delta_mode_)
    {
        for (int i = 0; i < U_num_; i++)
        {
            input(i) = control_total[i];
            cout << "*****control_cmd:**************" << i << "  " << input(i) << endl;
        }
    }
    if (delta_mode_)
    {
        for (int i = 0; i < U_num_; i++)
        {
            input(i) = control_total[i] + last_u1_(i); //这个要很具机器人编号取选择是last_U1还是last_U2..
            cout << "*****control_cmd:**************" << i << "  " << input(i) << endl;
        }
    }

    return input;
}
VectorXd MPC_Formation::formation_qpOASES_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix, VectorXd Lowbound, VectorXd Upbound, int max_iter)
{
    VectorXd input = VectorXd::Zero(U_num_);
    qpOASES_solver qpOASES_prob(HessianMatrix, GradientMatrix, Lowbound, Upbound, max_iter);
    std::vector<double> control_total = qpOASES_prob.solve();
    if (!delta_mode_)
    {
        for (int i = 0; i < U_num_; i++)
        {
            input(i) = control_total[i];
            cout << "*****control_cmd**************" << i << "  " << input(i) << endl;
        }
    }

    if (delta_mode_)
    {
        for (int i = 0; i < U_num_; i++)
        {
            input(i) = control_total[i] + last_u1_(i);
            cout << "*****control_cmd**************" << i << "  " << input(i) << endl;
        }
    }

    return input;
}
VectorXd MPC_Formation::formation_qpOASES_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix, MatrixXd A_cons, VectorXd lbA, VectorXd ubA, VectorXd Lowbound, VectorXd Upbound, int max_iter)
{
    VectorXd input = VectorXd::Zero(U_num_);
    qpOASES_solver qpOASES_prob(HessianMatrix, GradientMatrix, A_cons, Lowbound, Upbound, lbA, ubA, max_iter);
    std::vector<double> control_total = qpOASES_prob.solve();
    if (!delta_mode_)
    {
        for (int i = 0; i < U_num_; i++)
        {
            input(i) = control_total[i];
            cout << "*****control_cmd:**************" << i << "  " << input(i) << endl;
        }
    }

    if (delta_mode_)
    {
        for (int i = 0; i < U_num_; i++)
        {
            input(i) = control_total[i] + last_u1_(i);
            cout << "*****control_cmd:**************" << i << "  " << input(i) << endl;
        }
    }

    return input;
}
// void MPC_Formation::Update(int iter_num, MatrixXd refTrajectory, VectorXd last_u, VectorXd currentState)
// {




// }