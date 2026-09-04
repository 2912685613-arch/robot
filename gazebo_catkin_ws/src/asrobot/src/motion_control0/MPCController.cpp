#include "../../include/asrobot/MPCController.h"
MPCController::MPCController()
{
        delta_mode_ = 0;
        linear_mode_ = 1;
        relax_mode_ = 1;
        //AdjacentMatrix_=MatrixXd::Zero(3,3);
}

MPCController::MPCController(const DyModel ASRmodel, const float samplingtime, const int Np, const int Nc, const int X_num, const int U_num, const int Y_num, bool relax_mode, const VectorXd umax, const VectorXd umin)
    : ASRModel_(ASRmodel),
      samplingtime_(samplingtime),
      Np_(Np),
      Nc_(Nc),
      X_num_(X_num),
      U_num_(U_num),
      Y_num_(Y_num),
      relax_mode_(relax_mode),
      umax_(umax),
      umin_(umin)
{
        dumax_ = VectorXd::Zero(U_num);
        dumin_ = VectorXd::Zero(U_num);
        currentState_ = VectorXd::Zero(X_num);
        last_u_ = VectorXd::Zero(U_num);
        referInfo_ = VectorXd::Zero(X_num);
        ymax_ = VectorXd::Zero(Y_num);
        ymin_ = VectorXd::Zero(Y_num);
        delta_mode_ = 0;
        linear_mode_ = 1;
        //AdjacentMatrix_=MatrixXd::Zero(3,3);
}
MPCController::MPCController(const DyModel ASRmodel, const float samplingtime, const int Np, const int Nc, const int X_num, const int U_num, const int Y_num, bool relax_mode, const VectorXd umax, const VectorXd umin, const VectorXd dumax, const VectorXd dumin)
    : ASRModel_(ASRmodel),
      samplingtime_(samplingtime),
      Np_(Np),
      Nc_(Nc),
      X_num_(X_num),
      U_num_(U_num),
      Y_num_(Y_num),
      relax_mode_(relax_mode),
      umax_(umax),
      umin_(umin),
      dumax_(dumax),
      dumin_(dumin)

{
        delta_mode_ = 1;
        linear_mode_ = 1;
        currentState_ = VectorXd::Zero(X_num);
        last_u_ = VectorXd::Zero(U_num);
        referInfo_ = VectorXd::Zero(X_num);
        ymax_ = VectorXd::Zero(Y_num);
        ymin_ = VectorXd::Zero(Y_num);
        // AdjacentMatrix_=MatrixXd::Zero(3,3);
}
MPCController::MPCController(const DyModel ASRmodel, const float samplingtime, const int Np, const int Nc, const int X_num, const int U_num, const int Y_num, bool relax_mode, const VectorXd umax, const VectorXd umin, const VectorXd dumax, const VectorXd dumin, const VectorXd ymax, const VectorXd ymin)
    : ASRModel_(ASRmodel),
      samplingtime_(samplingtime),
      Np_(Np),
      Nc_(Nc),
      X_num_(X_num),
      U_num_(U_num),
      Y_num_(Y_num),
      relax_mode_(relax_mode),
      umax_(umax),
      umin_(umin),
      dumax_(dumax),
      dumin_(dumin),

      ymax_(ymax),
      ymin_(ymin)

{
        currentState_ = VectorXd::Zero(X_num);
        last_u_ = VectorXd::Zero(U_num);
        referInfo_ = VectorXd::Zero(X_num);
        delta_mode_ = 1;
        linear_mode_ = 1;
        //  AdjacentMatrix_=MatrixXd::Zero(3,3);
        if (X_num != referInfo_.size())
        {
                std::cout << "the dimension of the reference information is not consistent with X_num" << std::endl;
        }
}
void MPCController::set_linear_mode(int mode) { linear_mode_ = mode; }
void MPCController::UpdateVector_refInfo(int iter_num, MatrixXd refTrajectory)
{

        referInfo_ = refTrajectory.block(iter_num - 1, 0, 1, X_num_).transpose();
}
void MPCController::UpdateVector_Last_u(VectorXd last_u)
{
        last_u_ = last_u;
}
void MPCController::UpdateVector_CurrentState(VectorXd currentState)
{
        currentState_ = currentState;
}

Eigen::MatrixXd MPCController::UpdateStateMatrixA()
{

        Eigen::MatrixXd A = MatrixXd::Zero(X_num_, X_num_);
        //based on the dynamic model and kinematic model
        if (linear_mode_ == 1)
        {
                if (X_num_ == 5)
                {
                        double u_d = referInfo_(3);     // at k step, desired linear velocity on the x axis
                        double r_d = referInfo_(4);     // at k step, desired yaw angle velocity
                        double theta_d = referInfo_(2); // at k steo, desired yaw angle
                        double T = samplingtime_;
                        A << 1, 0, -u_d * sin(theta_d) * T, cos(theta_d) * T, 0,
                            0, 1, u_d * cos(theta_d) * T, sin(theta_d) * T, 0,
                            0, 0, 1, 0, T,
                            0, 0, 0, 1, 0,
                            0, 0, 0, 0, 1; //A=I+T*A(t)
                        Matrix2d desiredMatrix;
                        desiredMatrix << u_d, 0,
                            0, r_d;
                        Matrix2d a22 = -1 * samplingtime_ * (ASRModel_.MassMatrix.inverse() * ASRModel_.HydroLinMatrix + 2 * ASRModel_.MassMatrix.inverse() * ASRModel_.HydroNonLinMatrix * desiredMatrix) + Eigen::Matrix2d::Identity();
                        A.block(3, 3, 2, 2) = a22;
                        //cout<<"Matrix_A:"<<"     "<<A<<endl;
                }

                //based on the kinematic model
                if (X_num_ == 3)
                {
                        double u_d = referInfo_(3);     // at k step, desired linear velocity on the x axis
                        double r_d = referInfo_(4);     // at k step, desired yaw angle velocity
                        double theta_d = referInfo_(2); // at k steo, desired yaw angle
                        double T = samplingtime_;
                        A << 1, 0, -u_d * sin(theta_d) * T,
                            0, 1, u_d * cos(theta_d) * T,
                            0, 0, 1;
                }
        }
        if (linear_mode_ == 2)
        {
                if (X_num_ == 5)
                {
                        double u_d = currentState_(3);     // at k step, desired linear velocity on the x axis
                        double r_d = currentState_(4);     // at k step, desired yaw angle velocity
                        double theta_d = currentState_(2); // at k steo, desired yaw angle
                        double T = samplingtime_;
                        A << 1, 0, -u_d * sin(theta_d) * T, cos(theta_d) * T, 0,
                            0, 1, u_d * cos(theta_d) * T, sin(theta_d) * T, 0,
                            0, 0, 1, 0, T,
                            0, 0, 0, 1, 0,
                            0, 0, 0, 0, 1; //A=I+T*A(t)
                        Matrix2d desiredMatrix;
                        desiredMatrix << u_d, 0,
                            0, r_d;
                        Matrix2d a22 = -1 * samplingtime_ * (ASRModel_.MassMatrix.inverse() * ASRModel_.HydroLinMatrix + 2 * ASRModel_.MassMatrix.inverse() * ASRModel_.HydroNonLinMatrix * desiredMatrix) + Eigen::Matrix2d::Identity();
                        A.block(3, 3, 2, 2) = a22;
                        //cout<<"Matrix_A:"<<"     "<<A<<endl;
                }

                //based on the kinematic model
                if (X_num_ == 3)
                {
                        double u_d = currentState_(3);     // at k step, desired linear velocity on the x axis
                        double r_d = currentState_(4);     // at k step, desired yaw angle velocity
                        double theta_d = currentState_(2); // at k steo, desired yaw angle
                        double T = samplingtime_;
                        A << 1, 0, -u_d * sin(theta_d) * T,
                            0, 1, u_d * cos(theta_d) * T,
                            0, 0, 1;
                }
        }

        return A;
}

Eigen::MatrixXd MPCController::UpdateStateMatrixB()
{
        Eigen::MatrixXd B = MatrixXd::Zero(X_num_, U_num_);
        if (linear_mode_ == 1)
        {
                if (X_num_ == 5)
                {
                        B.block(3, 0, 2, 2) = samplingtime_ * ASRModel_.MassMatrix.inverse();
                        //cout<<"Matrix_B:"<<"     "<<B<<endl;
                }
                if (X_num_ == 3)
                {
                        double theta_d = referInfo_(2);
                        B << cos(theta_d) * samplingtime_, 0,
                            sin(theta_d) * samplingtime_, 0,
                            0, 1 * samplingtime_;
                }
        }
        if (linear_mode_ == 2)
        {
                if (X_num_ == 5)
                {
                        B.block(3, 0, 2, 2) = samplingtime_ * ASRModel_.MassMatrix.inverse();
                        //cout<<"Matrix_B:"<<"     "<<B<<endl;
                }
                if (X_num_ == 3)
                {
                        double theta_d = currentState_(2);
                        B << cos(theta_d) * samplingtime_, 0,
                            sin(theta_d) * samplingtime_, 0,
                            0, 1 * samplingtime_;
                }
        }
        return B;
}

Eigen::MatrixXd MPCController::UpdateStateMatrixC()
{

        Eigen::MatrixXd C = MatrixXd::Identity(Y_num_, X_num_);
        //cout<<"Matrix_C:"<<"     "<<C<<endl;
        return C;
}
//---------------------------------------------------the coefficient Matrix of the state space in the delta mode-------------------------//
Eigen::MatrixXd MPCController::d_UpdateStateMatrixA()
{
        MatrixXd A = MatrixXd::Zero(U_num_ + X_num_, U_num_ + X_num_);
        A.block(0, 0, X_num_, X_num_) = UpdateStateMatrixA();
        A.block(0, X_num_, X_num_, U_num_) = UpdateStateMatrixB();
        A.block(X_num_, X_num_, U_num_, U_num_) = MatrixXd::Identity(U_num_, U_num_);
        return A;
}
Eigen::MatrixXd MPCController::d_UpdateStateMatrixB()
{
        MatrixXd B = MatrixXd::Zero(X_num_ + U_num_, U_num_);
        B.block(0, 0, X_num_, U_num_) = UpdateStateMatrixB();
        B.block(X_num_, 0, U_num_, U_num_) = MatrixXd::Identity(U_num_, U_num_);
        return B;
}
Eigen::MatrixXd MPCController::d_UpdateStateMatrixC()
{
        MatrixXd C = MatrixXd::Zero(Y_num_, X_num_ + U_num_);
        C.block(0, 0, Y_num_, X_num_) = UpdateStateMatrixC();
        return C;
}
//*****************************************************************************************************************************************
Eigen::VectorXd MPCController::Compute_d_piao()
{
        double m11 = ASRModel_.MassMatrix(0, 0);
        double m22 = ASRModel_.MassMatrix(1, 1);
        double dL_11 = ASRModel_.HydroLinMatrix(0, 0);
        double dL_22 = ASRModel_.HydroLinMatrix(1, 1);
        double dNL_11 = ASRModel_.HydroNonLinMatrix(0, 0);
        double dNL_22 = ASRModel_.HydroNonLinMatrix(1, 1);
        MatrixXd A = UpdateStateMatrixA();
        MatrixXd B = UpdateStateMatrixB();
        Eigen::VectorXd temp = VectorXd::Zero(X_num_);
        Eigen::VectorXd next_state = VectorXd::Zero(X_num_);
        Eigen::VectorXd d_piao;
        if (X_num_ == 5)
        {
                next_state(0) = currentState_(0) + samplingtime_ * currentState_(3) * cos(currentState_(2));
                next_state(1) = currentState_(1) + samplingtime_ * currentState_(3) * sin(currentState_(2));
                next_state(2) = currentState_(2) + samplingtime_ * currentState_(4);
                next_state(3) = currentState_(3) + samplingtime_ * (last_u_(0) - dL_11 * currentState_(3) - dNL_11 * currentState_(3) * currentState_(3)) / m11;
                next_state(4) = currentState_(4) + samplingtime_ * (last_u_(1) - dL_22 * currentState_(4) - dNL_22 * currentState_(4) * currentState_(4)) / m22;
                temp = next_state - A * currentState_ - B * last_u_;
        }
        if (X_num_ == 3)
        {
                next_state(0) = currentState_(0) + samplingtime_ * last_u_(0) * cos(currentState_(2));
                next_state(1) = currentState_(1) + samplingtime_ * last_u_(0) * sin(currentState_(2));
                next_state(2) = currentState_(2) + samplingtime_ * last_u_(1);
                temp = next_state - A * currentState_ - B * last_u_;
        }
        if (!delta_mode_)
        {
                d_piao = VectorXd::Zero(X_num_);
                d_piao = temp;
        }
        if (delta_mode_)
        {
                d_piao = VectorXd::Zero(X_num_ + U_num_);
                d_piao.head(X_num_) = temp;
        }
        return d_piao;
}
Eigen::MatrixXd MPCController::InitQMatrix()
{

        int total = Np_ * Y_num_;
        MatrixXd Q = MatrixXd::Identity(total, total);
        return Q;
}
Eigen::MatrixXd MPCController::DebugQMatrix(double q1, double q2, double q3, double q4, double q5)
{
        //generally only the first three parameters in the matrix need to be adjusted,
        //respect to adjusting the conveger speed of the position x,y, and yaw angle theta
        MatrixXd q_ = MatrixXd::Identity(Y_num_, Y_num_);
       std::cout<<"coming into the debugQ"<<std::endl;
        if(Y_num_==5)
        {
        q_(0, 0) = q1;
        q_(1, 1) = q2;
        q_(2, 2) = q3;
        q_(3, 3) = q4;
        q_(4, 4) = q5;
        }
        if(Y_num_==3)
        {
        q_(0, 0) = q1;
        q_(1, 1) = q2;
        q_(2, 2) = q3;

        }

        cout << "Q matrix is:"
             << "   " << q_ << endl;
        int total = Np_ * Y_num_;
        MatrixXd Q = MatrixXd::Zero(total, total);
        for (int i = 0; i < Np_; i++)
        {
                Q.block(i * Y_num_, i * Y_num_, Y_num_, Y_num_) = q_;
        }
        return Q;
}
Eigen::MatrixXd MPCController::ComputeRMatrix()
{
        int total = U_num_ * Nc_;
        MatrixXd R = 0.1 * MatrixXd::Identity(total, total);
        return R;
}
//******************************************************************************************************************************************
Eigen::MatrixXd MPCController::ComputePHIMatrix()
{
        MatrixXd A;
        MatrixXd C;
        if (!delta_mode_)
        {
                A = UpdateStateMatrixA();
                C = UpdateStateMatrixC();
        }
        if (delta_mode_)
        {
                A = d_UpdateStateMatrixA();
                //cout<<"d_matrix_A:"<<A<<endl;
                C = d_UpdateStateMatrixC();
        }
        int col_ = A.cols();
        int row_ = C.rows();
        int total = Np_ * row_;
        MatrixXd PHIMatrix = MatrixXd::Zero(total, col_);
        for (int i = 0; i < Np_; i++)
        {
                MatrixXd temp2 = MatrixXd::Identity(col_, col_);
                for (int j = 0; j < i + 1; j++)
                {
                        temp2 = temp2 * A;
                }
                PHIMatrix.block(i * row_, 0, row_, col_) = C * temp2;
        }
        return PHIMatrix;
}
Eigen::MatrixXd MPCController::ComputeTHETAMatrix()
{
        MatrixXd A;
        MatrixXd B;
        MatrixXd C;
        if (!delta_mode_)
        {
                A = UpdateStateMatrixA();
                B = UpdateStateMatrixB();
                C = UpdateStateMatrixC();
        }
        if (delta_mode_)
        {
                A = d_UpdateStateMatrixA();
                B = d_UpdateStateMatrixB();
                //cout<<"d_matrix_A:"<<B<<endl;
                C = d_UpdateStateMatrixC();
        }
        int row_ = C.rows();   // is the Ny
        int col_ = B.cols();   // is the Nu
        int size_A = A.rows(); // is the Nx
        int total_rows = Np_ * row_;
        int total_cols = Nc_ * col_;
        MatrixXd THETAMatrix = MatrixXd::Zero(total_rows, total_cols);
        MatrixXd RoMatrix = MatrixXd::Zero(total_rows, col_); //RoMatrix is the first Nu colunm of the THETAMatrix
        for (int i = 0; i < Np_; i++)
        {
                MatrixXd temp3 = MatrixXd::Identity(size_A, size_A);
                for (int j = 0; j < i; j++)
                {
                        temp3 = A * temp3;
                }

                RoMatrix.block(i * row_, 0, row_, col_) = C * temp3 * B;
        }
        THETAMatrix.block(0, 0, row_ * Np_, col_) = RoMatrix;
        for (int k = 1; k < Nc_; k++)
        {
                MatrixXd temp4 = MatrixXd::Zero(k * row_, col_);
                MatrixXd temp5 = MatrixXd::Zero(Np_ * row_, col_);
                temp5.block(0, 0, k * row_, col_) = temp4;
                temp5.block(k * row_, 0, (Np_ - k) * row_, col_) = RoMatrix.block(0, 0, (Np_ - k) * row_, col_);
                THETAMatrix.block(0, k * col_, row_ * Np_, col_) = temp5;
        }
        return THETAMatrix;
}
Eigen::MatrixXd MPCController::ComputeGAMMAMatrix()
{
        MatrixXd A;
        MatrixXd C;
        if (!delta_mode_)
        {
                A = UpdateStateMatrixA();
                C = UpdateStateMatrixC();
        }
        if (delta_mode_)
        {
                A = d_UpdateStateMatrixA();
                C = d_UpdateStateMatrixC();
        }
        int row_ = C.rows();   // is the Ny
        int size_A = A.rows(); // is the Nx
        int total_rows = Np_ * row_;
        int total_cols = Np_ * size_A;
        MatrixXd GAMMAMatrix = MatrixXd::Zero(total_rows, total_cols);
        MatrixXd RoMatrix = MatrixXd::Zero(total_rows, size_A); //RoMatrix is the first Nu colunm of the THETAMatrix
        cout << "======Gradient in this place1========" << endl;
        for (int i = 0; i < Np_; i++)
        {
                MatrixXd temp3 = MatrixXd::Identity(size_A, size_A);
                for (int j = 0; j < i; j++)
                {
                        temp3 = A * temp3;
                }

                RoMatrix.block(i * row_, 0, row_, size_A) = C * temp3;
        }
        GAMMAMatrix.block(0, 0, row_ * Np_, size_A) = RoMatrix;
        cout << "======Gradient in this place2========" << endl;
        for (int k = 1; k < Np_; k++)
        {
                MatrixXd temp4 = MatrixXd::Zero(k * row_, size_A);
                MatrixXd temp5 = MatrixXd::Zero(Np_ * row_, size_A);
                temp5.block(0, 0, k * row_, size_A) = temp4;
                temp5.block(k * row_, 0, (Np_ - k) * row_, size_A) = RoMatrix.block(0, 0, (Np_ - k) * row_, size_A);
                GAMMAMatrix.block(0, k * size_A, row_ * Np_, size_A) = temp5;
        }
        cout << "======Gradient in this place4========" << endl;
        return GAMMAMatrix;
}
//******************************************************************************************************************************************
Eigen::MatrixXd MPCController::ComputeHessianMatrix(const Eigen::MatrixXd &matrix_Q)
{
        MatrixXd THETAMatrix = ComputeTHETAMatrix();
        MatrixXd matrix_R = ComputeRMatrix();
        MatrixXd HessianMatrix = 2 * (THETAMatrix.transpose() * matrix_Q * THETAMatrix + matrix_R);
        return HessianMatrix;
}
Eigen::MatrixXd MPCController::ComputeHessianMatrix(const Eigen::MatrixXd &matrix_Q, double Ro)
{
        MatrixXd THETAMatrix = ComputeTHETAMatrix();
        MatrixXd matrix_R = ComputeRMatrix();
        int size_R = matrix_R.cols();
        MatrixXd HessianMatrix = MatrixXd::Zero(size_R + 1, size_R + 1);
        HessianMatrix.block(0, 0, size_R, size_R) = 2 * (THETAMatrix.transpose() * matrix_Q * THETAMatrix + matrix_R);
        HessianMatrix(size_R, size_R) = Ro;
        return HessianMatrix;
}
Eigen::VectorXd MPCController::ComputeGradientMatrix(const Eigen::MatrixXd &matrix_Q, const Eigen::VectorXd X_piao)
{
        //if relax_mode=1,represent the matrix contains the relax factor term
        MatrixXd THETAMatrix = ComputeTHETAMatrix();
        MatrixXd PHIMatrix = ComputePHIMatrix();
        int col_ = THETAMatrix.cols();
        VectorXd GradientMatrix;
        if (relax_mode_)
        {
                GradientMatrix = VectorXd::Zero(col_ + 1);
                GradientMatrix.head(col_) = 2 * THETAMatrix.transpose() * matrix_Q * PHIMatrix * X_piao;
        }
        else
        {
                GradientMatrix = VectorXd::Zero(col_);
                GradientMatrix = 2 * THETAMatrix.transpose() * matrix_Q * PHIMatrix * X_piao;
        }
        return GradientMatrix;
}
Eigen::VectorXd MPCController::ComputeGradientMatrix(const Eigen::MatrixXd &matrix_Q, const Eigen::VectorXd Yita_ref, int mark)
{
        //if relax_mode=1,represent the matrix contains the relax factor term
        MatrixXd THETAMatrix = ComputeTHETAMatrix();
        MatrixXd PHIMatrix = ComputePHIMatrix();
        VectorXd d_piao = Compute_d_piao();
        int size_d_piao = d_piao.rows();
        VectorXd PSI = VectorXd::Zero(Np_ * size_d_piao);
        for (int i = 0; i < Np_; i++)
        {

                PSI.segment(i * size_d_piao, size_d_piao) = d_piao;
        }

        MatrixXd GAMMAMatrix = ComputeGAMMAMatrix();
        int col_ = THETAMatrix.cols();
        VectorXd GradientMatrix;
        if (relax_mode_)
        {
                GradientMatrix = VectorXd::Zero(col_ + 1);
                Eigen::VectorXd kesi(X_num_+U_num_);
                kesi.head(X_num_)=currentState_;
                kesi.segment(X_num_, U_num_)=last_u_;
                if(delta_mode_){
                GradientMatrix.head(col_) = 2 * THETAMatrix.transpose() * matrix_Q * (PHIMatrix * kesi + GAMMAMatrix * PSI - Yita_ref);
                }
                if(!delta_mode_)
                {
                GradientMatrix.head(col_) = 2 * THETAMatrix.transpose() * matrix_Q * (PHIMatrix * currentState_ + GAMMAMatrix * PSI - Yita_ref);   
                }
                cout << "======Gradient in this place3========" << endl;
        }
        else
        {
                GradientMatrix = VectorXd::Zero(col_);
                Eigen::VectorXd kesi(X_num_+U_num_);
                kesi.head(X_num_)=currentState_;
                kesi.segment(X_num_, U_num_)=last_u_;
                if(delta_mode_){
                GradientMatrix.head(col_) = 2 * THETAMatrix.transpose() * matrix_Q * (PHIMatrix * kesi + GAMMAMatrix * PSI - Yita_ref);
                }
                if(!delta_mode_)
                {
                GradientMatrix.head(col_) = 2 * THETAMatrix.transpose() * matrix_Q * (PHIMatrix * currentState_ + GAMMAMatrix * PSI - Yita_ref);   
                }
               // GradientMatrix = 2 * THETAMatrix.transpose() * matrix_Q * (Yita_ref - PHIMatrix * currentState_ - GAMMAMatrix * PSI);
        }
        return GradientMatrix;
}
/******************************************************************************************************************************************************************************************/
MatrixXd MPCController::UpdateInputConstraint_Matrix_Aue()
{

        MatrixXd matrix_Aue;
        if (relax_mode_)
        {
                matrix_Aue = MatrixXd::Zero(Nc_ * U_num_ + 1, Nc_ * U_num_ + 1);
        }
        else
        {
                matrix_Aue = MatrixXd::Zero(Nc_ * U_num_, Nc_ * U_num_);
        }

        matrix_Aue.block(0, 0, Nc_ * U_num_, Nc_ * U_num_) = MatrixXd::Identity(Nc_ * U_num_, Nc_ * U_num_);

        return matrix_Aue;
}
VectorXd MPCController::UpdateInputConstraint_Matrix_Upue()
{
        int size_u = umax_.size();
        VectorXd matrix_Upue;
        VectorXd temp = VectorXd::Zero(Nc_ * size_u);

        for (size_t i = 0; i < Nc_; i++)
        {
                if (!delta_mode_)
                {
                        temp.segment(i * size_u, size_u) = umax_;
                }
                if (delta_mode_)
                {
                        temp.segment(i * size_u, size_u) = dumax_;
                }
        }
        if (!relax_mode_)
        {

                matrix_Upue = temp;
        }
        if (relax_mode_)
        {
                matrix_Upue = VectorXd::Zero(Nc_ * size_u + 1);
                matrix_Upue.head(Nc_ * size_u) = temp;
                matrix_Upue(Nc_ * size_u) = 100;
        }

        return matrix_Upue;
}
VectorXd MPCController::UpdateInputConstraint_Matrix_Lowue()
{
        int size_u = umin_.size();
        VectorXd matrix_Lowue;
        VectorXd temp = VectorXd::Zero(Nc_ * size_u);
        for (size_t i = 0; i < Nc_; i++)
        {
                if (!delta_mode_)
                {
                        temp.block(i * size_u, 0, size_u, 1) = umin_;
                }
                if (delta_mode_)
                {
                        temp.block(i * size_u, 0, size_u, 1) = dumin_;
                }
        }
        if (!relax_mode_)
        {
                matrix_Lowue = temp;
        }
        if (relax_mode_)
        {
                matrix_Lowue = VectorXd::Zero(Nc_ * size_u + 1);
                matrix_Lowue.head(Nc_ * size_u) = temp;
                matrix_Lowue(Nc_ * size_u) = 0;
        }

        return matrix_Lowue;
}
/***************************** following function generate matrix contains the constraint of u and delta u in the delta mode**********************************************************************************************************/
MatrixXd MPCController::d_UpdateInputConstraint_Matrix_Aue()
{

        MatrixXd matrix_Aue;
        if (relax_mode_)
        {
                matrix_Aue = MatrixXd::Zero(2 * U_num_ * Nc_, U_num_ * Nc_ + 1); //锟斤拷没锟斤拷锟缴筹拷锟斤拷锟接碉拷锟斤拷锟金，讹拷锟斤拷一锟斤拷0
        }
        else
        {
                matrix_Aue = MatrixXd::Zero(2 * U_num_ * Nc_, U_num_ * Nc_);
        }
        matrix_Aue.block(0, 0, Nc_ * U_num_, Nc_ * U_num_) = MatrixXd::Identity(Nc_ * U_num_, Nc_ * U_num_);

        // the coefficienf matrix for U
        MatrixXd con_matrix_A = MatrixXd::Zero(U_num_ * Nc_, U_num_ * Nc_);
        MatrixXd temp = MatrixXd::Zero(U_num_ * Nc_, U_num_);
        for (int i = 0; i < Nc_; i++)
        {
                temp.block(i * U_num_, 0, U_num_, U_num_) = MatrixXd::Identity(U_num_, U_num_);
        }
        con_matrix_A.block(0, 0, U_num_ * Nc_, U_num_) = temp;

        for (int i = 1; i < Nc_; i++)
        {
                con_matrix_A.block(i * U_num_, i * U_num_, (Nc_ - i) * U_num_, U_num_) = temp.block(i * U_num_, 0, (Nc_ - i) * U_num_, U_num_);
        }

        matrix_Aue.block(U_num_ * Nc_, 0, U_num_ * Nc_, U_num_ * Nc_) = con_matrix_A;

        return matrix_Aue;
}
VectorXd MPCController::d_UpdateInputConstraint_Matrix_Upue()
{

        VectorXd matrix_Upue = VectorXd::Zero(2 * U_num_ * Nc_);
        VectorXd con_matrix_d_Umax = VectorXd::Zero(Nc_ * U_num_);
        VectorXd con_matrix_Umax = VectorXd::Zero(Nc_ * U_num_);
        VectorXd con_matrix_last_U = VectorXd::Zero(Nc_ * U_num_);
        for (int i = 0; i < Nc_; i++)
        {
                con_matrix_d_Umax.block(i * U_num_, 0, U_num_, 1) = dumax_;
                con_matrix_Umax.block(i * U_num_, 0, U_num_, 1) = umax_;
                con_matrix_last_U.block(i * U_num_, 0, U_num_, 1) = last_u_;
        }
        matrix_Upue.block(0, 0, U_num_ * Nc_, 1) = con_matrix_d_Umax;
        matrix_Upue.block(U_num_ * Nc_, 0, U_num_ * Nc_, 1) = con_matrix_Umax - con_matrix_last_U;
        return matrix_Upue;
}
VectorXd MPCController::d_UpdateInputConstraint_Matrix_Lowue()
{
        VectorXd matrix_Lowue = VectorXd::Zero(2 * U_num_ * Nc_);
        VectorXd con_matrix_d_Umin = VectorXd::Zero(Nc_ * U_num_);
        VectorXd con_matrix_Umin = VectorXd::Zero(Nc_ * U_num_);
        VectorXd con_matrix_last_U = VectorXd::Zero(Nc_ * U_num_);
        for (int i = 0; i < Nc_; i++)
        {
                con_matrix_d_Umin.block(i * U_num_, 0, U_num_, 1) = dumin_;
                con_matrix_Umin.block(i * U_num_, 0, U_num_, 1) = umin_;
                con_matrix_last_U.block(i * U_num_, 0, U_num_, 1) = last_u_;
        }
        matrix_Lowue.block(0, 0, U_num_ * Nc_, 1) = con_matrix_d_Umin;
        matrix_Lowue.block(U_num_ * Nc_, 0, U_num_ * Nc_, 1) = con_matrix_Umin - con_matrix_last_U;
        return matrix_Lowue;
}

/************************ following function generate matrix contains the constraint of u in the delta mode**********************************************************************************************************/
MatrixXd MPCController::d_UpdateInputConstraint_Matrix_Aue_qpOASES()
{

        MatrixXd matrix_Aue;
        if (relax_mode_)
        {
                matrix_Aue = MatrixXd::Zero(U_num_ * Nc_, U_num_ * Nc_ + 1); //锟斤拷没锟斤拷锟缴筹拷锟斤拷锟接碉拷锟斤拷锟金，讹拷锟斤拷一锟斤拷0
        }
        else
        {
                matrix_Aue = MatrixXd::Zero(U_num_ * Nc_, U_num_ * Nc_);
        }

        // the coefficienf matrix for U
        MatrixXd con_matrix_A = MatrixXd::Zero(U_num_ * Nc_, U_num_ * Nc_);
        MatrixXd temp = MatrixXd::Zero(U_num_ * Nc_, U_num_);
        for (int i = 0; i < Nc_; i++)
        {
                temp.block(i * U_num_, 0, U_num_, U_num_) = MatrixXd::Identity(U_num_, U_num_);
        }
        con_matrix_A.block(0, 0, U_num_ * Nc_, U_num_) = temp;
        for (int i = 1; i < Nc_; i++)
        {
                con_matrix_A.block(i * U_num_, i * U_num_, (Nc_ - i) * U_num_, U_num_) = temp.block(i * U_num_, 0, (Nc_ - i) * U_num_, U_num_);
        }

        matrix_Aue.block(0, 0, U_num_ * Nc_, U_num_ * Nc_) = con_matrix_A;
        return matrix_Aue;
}
VectorXd MPCController::d_UpdateInputConstraint_Matrix_Upue_qpOASES()
{

        VectorXd matrix_Upue = VectorXd::Zero(U_num_ * Nc_);

        VectorXd con_matrix_Umax = VectorXd::Zero(Nc_ * U_num_);
        VectorXd con_matrix_last_U = VectorXd::Zero(Nc_ * U_num_);
        for (int i = 0; i < Nc_; i++)
        {

                con_matrix_Umax.block(i * U_num_, 0, U_num_, 1) = umax_;
                con_matrix_last_U.block(i * U_num_, 0, U_num_, 1) = last_u_;
        }

        matrix_Upue = con_matrix_Umax - con_matrix_last_U;
        return matrix_Upue;
}
VectorXd MPCController::d_UpdateInputConstraint_Matrix_Lowue_qpOASES()
{
        VectorXd matrix_Lowue = VectorXd::Zero(U_num_ * Nc_);
        VectorXd con_matrix_Umin = VectorXd::Zero(Nc_ * U_num_);
        VectorXd con_matrix_last_U = VectorXd::Zero(Nc_ * U_num_);
        for (int i = 0; i < Nc_; i++)
        {

                con_matrix_Umin.block(i * U_num_, 0, U_num_, 1) = umin_;
                con_matrix_last_U.block(i * U_num_, 0, U_num_, 1) = last_u_;
        }

        matrix_Lowue = con_matrix_Umin - con_matrix_last_U;
        return matrix_Lowue;
}

/******************************************************************************************************************************************************************************************/
MatrixXd MPCController::UpdateOutputConstraint_Matrix_Aue()
{
        MatrixXd THETAMatrix = ComputeTHETAMatrix();
        int col_ = THETAMatrix.cols();
        int row_ = THETAMatrix.rows();
        MatrixXd matrix_Aue;
        if (relax_mode_)
        {
                matrix_Aue = MatrixXd::Zero(row_, col_ + 1);
        }
        else
        {
                matrix_Aue = MatrixXd::Zero(row_, col_);
        }
        matrix_Aue.block(0, 0, row_, col_) = THETAMatrix;
        return matrix_Aue;
}
VectorXd MPCController::UpdateOutputConstraint_Matrix_Upue() // generate the bound vector for the unequal constraint
{

        MatrixXd PHIMatrix = ComputePHIMatrix();
        VectorXd matrix_bound_ue = VectorXd::Zero(Np_ * Y_num_);
        VectorXd con_matrix_Ymax = VectorXd::Zero(Np_ * Y_num_);
        for (int i = 0; i < Np_; i++)
        {
                con_matrix_Ymax.block(i * Y_num_, 0, Y_num_, 1) = ymax_;
        }
        matrix_bound_ue = con_matrix_Ymax - PHIMatrix * currentState_;
        return matrix_bound_ue;
}
VectorXd MPCController::UpdateOutputConstraint_Matrix_Lowue() // generate the bound vector for the unequal constraint
{
        MatrixXd PHIMatrix = ComputePHIMatrix();
        VectorXd matrix_bound_ue = VectorXd::Zero(Np_ * Y_num_);
        VectorXd con_matrix_Ymin = VectorXd::Zero(Np_ * Y_num_);
        for (int i = 0; i < Np_; i++)
        {
                con_matrix_Ymin.block(i * Y_num_, 0, Y_num_, 1) = ymin_;
        }
        matrix_bound_ue = con_matrix_Ymin - PHIMatrix * currentState_;
        return matrix_bound_ue;
}
/********************************************************************************************************************************************************************************************/
VectorXd MPCController::Mpc_OSQP_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix, MatrixXd A, VectorXd Upbound, VectorXd Lowbound, int max_iter, const double eps_abs)
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
                        cout << "*****quadratic solver:**************" << i << "  " << input(i) << endl;
                }
        }
        if (delta_mode_)
        {
                for (int i = 0; i < U_num_; i++)
                {
                        //input(i) = control_total[i]+last_u_(i);
                        input(i) = control_total[i];
                        cout << "*****quadratic solver:**************" << i << "  " << input(i) << endl;
                }
        }

        return input;
}

/***********************************************************************************************************************************************************************************************/
VectorXd MPCController::Mpc_qpOASES_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix, VectorXd Lowbound, VectorXd Upbound, int max_iter)
{
        VectorXd input = VectorXd::Zero(U_num_);
        qpOASES_solver qpOASES_prob(HessianMatrix, GradientMatrix, Lowbound, Upbound, max_iter);
        std::vector<double> control_total = qpOASES_prob.solve();
        if (!delta_mode_)
        {
                for (int i = 0; i < U_num_; i++)
                {
                        input(i) = control_total[i];
                        cout << "*****quadratic solver**************" << i << "  " << input(i) << endl;
                }
        }

        if (delta_mode_)
        {
                for (int i = 0; i < U_num_; i++)
                {
                        // input(i) = control_total[i]+last_u_(i);
                        input(i) = control_total[i];
                        cout << "*****quadratic solver**************" << i << "  " << input(i) << endl;
                }
        }

        return input;
}
VectorXd MPCController::Mpc_qpOASES_Solve(MatrixXd HessianMatrix, VectorXd GradientMatrix, MatrixXd A_cons,
                                          VectorXd Lowbound, VectorXd Upbound,
                                          VectorXd lbA, VectorXd ubA, int max_iter)
{
        VectorXd input = VectorXd::Zero(U_num_);
        qpOASES_solver qpOASES_prob(HessianMatrix, GradientMatrix, A_cons, Lowbound, Upbound, lbA, ubA, max_iter);
        std::vector<double> control_total = qpOASES_prob.solve();
        if (!delta_mode_)
        {
                for (int i = 0; i < U_num_; i++)
                {
                        input(i) = control_total[i];
                        cout << "*****quadratic solver:**************" << i << "  " << input(i) << endl;
                }
        }

        if (delta_mode_)
        {
                for (int i = 0; i < U_num_; i++)
                {
                        // input(i) = control_total[i]+last_u_(i);
                        input(i) = control_total[i];
                        cout << "*****quadratic solver:**************" << i << "  " << input(i) << endl;
                }
        }

        return input;
}

//**************************************************************************************************************************************************//
VectorXd MPCController::generate_Yita_ref(int iter, MatrixXd reftrajectory)
{
        VectorXd Yita_ref = VectorXd::Zero(Np_ * Y_num_);
        for (int i = 0; i < Np_; i++)
        {
                VectorXd temp = reftrajectory.block(iter - 1, 0, 1, Y_num_).transpose();
                Yita_ref.segment(i * Y_num_, Y_num_) = temp;
        }
        return Yita_ref;
}

void MPCController::Update(int iter_num, MatrixXd refTrajectory, VectorXd last_u, VectorXd currentState)
{
        UpdateVector_refInfo(iter_num, refTrajectory);
        UpdateVector_Last_u(last_u);
        UpdateVector_CurrentState(currentState);
}
VectorXd MPCController::Update_Control_cmd(Eigen::MatrixXd Q, double Ro, Eigen::VectorXd x_piao, int solve_mode)
{
        VectorXd real_force = VectorXd::Zero(U_num_);
        MatrixXd Hessian = ComputeHessianMatrix(Q, Ro);
        MatrixXd Gradient = ComputeGradientMatrix(Q, x_piao);
        MatrixXd A;
        MatrixXd Upbound;
        MatrixXd Lowbound;
        MatrixXd Aue;
        MatrixXd Upue;
        MatrixXd Lowue;

        if (solve_mode == 1)
        {
                if (!delta_mode_)
                {
                        A = UpdateInputConstraint_Matrix_Aue();
                        Upbound = UpdateInputConstraint_Matrix_Upue();
                        Lowbound = UpdateInputConstraint_Matrix_Lowue();
                }
                if (delta_mode_)
                {
                        A = d_UpdateInputConstraint_Matrix_Aue();
                        Upbound = d_UpdateInputConstraint_Matrix_Upue();
                        Lowbound = d_UpdateInputConstraint_Matrix_Lowue();
                }
                // cout<<"Hessian matrix:"<<Hessian<<endl;
                // cout<<"Gradient matrix:"<<Gradient<<endl;
                real_force = Mpc_OSQP_Solve(Hessian, Gradient, A, Upbound, Lowbound, 3000, 1.0e+04);
        }
        if (solve_mode == 2)
        {

                // A=UpdateInputConstraint_Matrix_Aue();
                Upbound = UpdateInputConstraint_Matrix_Upue();
                Lowbound = UpdateInputConstraint_Matrix_Lowue();
                if (!delta_mode_)
                {
                        // cout<<"coming into this place"<<endl;
                        real_force = Mpc_qpOASES_Solve(Hessian, Gradient, Lowbound, Upbound, 300);
                }
                if (delta_mode_)
                {
                        Aue = d_UpdateInputConstraint_Matrix_Aue_qpOASES();
                        Upue = d_UpdateInputConstraint_Matrix_Upue_qpOASES();
                        Lowue = d_UpdateInputConstraint_Matrix_Lowue_qpOASES();
                        cout << "Hessian matrix:" << Hessian << endl;
                        cout << "Gradient matrix:" << Gradient << endl;
                        cout << "Aue" << Aue << endl;
                        cout << "LOwue" << Lowue << endl;
                        cout << "upue" << Upue << endl;
                        cout << "lowbound" << Lowbound << endl;
                        cout << "upbound" << Upbound << endl;

                        real_force = Mpc_qpOASES_Solve(Hessian, Gradient, Aue, Lowbound, Upbound, Lowue, Upue, 300);
                }
        }
        return real_force;
}
VectorXd MPCController::Update2_Control_cmd(Eigen::MatrixXd Q, double Ro, int iter, MatrixXd reftrajectory, int solve_mode)
{
        VectorXd real_force = VectorXd::Zero(U_num_);
        VectorXd Yita_ref = generate_Yita_ref(iter, reftrajectory);

        MatrixXd Hessian = ComputeHessianMatrix(Q, Ro);
        // cout<<"===The matrixs of the OSQP problem have been obtained11==="<<endl;
        MatrixXd Gradient = ComputeGradientMatrix(Q, Yita_ref, 1);//1 : differ from ComputeGradientMatrix in Update_Control_cmd
        //cout<<"===The matrixs of the OSQP problem have been obtained22==="<<endl;
        MatrixXd A;
        MatrixXd Upbound;
        MatrixXd Lowbound;
        MatrixXd Aue;
        MatrixXd Upue;
        MatrixXd Lowue;

        if (solve_mode == 1)
        {
                if (!delta_mode_)
                {
                        A = UpdateInputConstraint_Matrix_Aue();
                        Upbound = UpdateInputConstraint_Matrix_Upue();
                        Lowbound = UpdateInputConstraint_Matrix_Lowue();
                }
                if (delta_mode_)
                {
                        A = d_UpdateInputConstraint_Matrix_Aue();
                        Upbound = d_UpdateInputConstraint_Matrix_Upue();
                        Lowbound = d_UpdateInputConstraint_Matrix_Lowue();
                }
                real_force = Mpc_OSQP_Solve(Hessian, Gradient, A, Upbound, Lowbound, 3000, 1.0e+04);
        }
        if (solve_mode == 2)
        {

                Upbound = UpdateInputConstraint_Matrix_Upue();
                Lowbound = UpdateInputConstraint_Matrix_Lowue();
                if (!delta_mode_)
                {

                        real_force = Mpc_qpOASES_Solve(Hessian, Gradient, Lowbound, Upbound, 300);
                        // cout<<"====================in this place*********"<<endl;
                }
                if (delta_mode_)
                {
                        Aue = d_UpdateInputConstraint_Matrix_Aue_qpOASES();
                        Upue = d_UpdateInputConstraint_Matrix_Upue_qpOASES();
                        Lowue = d_UpdateInputConstraint_Matrix_Lowue_qpOASES();

                        real_force = Mpc_qpOASES_Solve(Hessian, Gradient, Aue, Lowbound, Upbound,
                                                       Lowue, Upue, 3000);
                }
        }
        return real_force;
}

//========================================================================================================================//
//=========================================================================================================================//

MatrixXd MPCController::mult_ComputeTHETAMatrix(int mult_Y_num_i)
{
        MatrixXd A;
        MatrixXd B;
        MatrixXd C;
        if (!delta_mode_)
        {
                A = UpdateStateMatrixA();
                B = UpdateStateMatrixB();
                C = MatrixXd::Identity(mult_Y_num_i, X_num_);
        }
        if (delta_mode_)
        {
                A = d_UpdateStateMatrixA();
                B = d_UpdateStateMatrixB();
                C = MatrixXd::Zero(mult_Y_num_i, X_num_ + U_num_);
                C.block(0, 0, mult_Y_num_i, X_num_) = MatrixXd::Identity(mult_Y_num_i, X_num_);
        }
        int row_ = C.rows();   // is the Ny
        int col_ = B.cols();   // is the Nu
        int size_A = A.rows(); // is the Nx
        int total_rows = Np_ * row_;
        int total_cols = Nc_ * col_;
        MatrixXd THETAMatrix = MatrixXd::Zero(total_rows, total_cols);
        MatrixXd RoMatrix = MatrixXd::Zero(total_rows, col_); //RoMatrix is the first Nu colunm of the THETAMatrix
        for (int i = 0; i < Np_; i++)
        {
                MatrixXd temp3 = MatrixXd::Identity(size_A, size_A);
                for (int j = 0; j < i; j++)
                {
                        temp3 = A * temp3;
                }

                RoMatrix.block(i * row_, 0, row_, col_) = C * temp3 * B;
        }
        THETAMatrix.block(0, 0, row_ * Np_, col_) = RoMatrix;
        for (int k = 1; k < Nc_; k++)
        {
                MatrixXd temp4 = MatrixXd::Zero(k * row_, col_);
                MatrixXd temp5 = MatrixXd::Zero(Np_ * row_, col_);
                temp5.block(0, 0, k * row_, col_) = temp4;
                temp5.block(k * row_, 0, (Np_ - k) * row_, col_) = RoMatrix.block(0, 0, (Np_ - k) * row_, col_);
                THETAMatrix.block(0, k * col_, row_ * Np_, col_) = temp5;
        }
        return THETAMatrix;
}

Eigen::MatrixXd MPCController::mult_ComputePHIMatrix(int mult_Y_num_i)
{
        MatrixXd A;
        MatrixXd C;
        if (!delta_mode_)
        {
                A = UpdateStateMatrixA();
                C = MatrixXd::Identity(mult_Y_num_i, X_num_);
        }
        if (delta_mode_)
        {
                A = d_UpdateStateMatrixA();
                C = MatrixXd::Zero(mult_Y_num_i, X_num_ + U_num_);
                C.block(0, 0, mult_Y_num_i, X_num_) = MatrixXd::Identity(mult_Y_num_i, X_num_);
        }

        int col_ = A.cols();
        int row_ = C.rows();
        int total = Np_ * row_;
        MatrixXd PHIMatrix = MatrixXd::Zero(total, col_);
        for (int i = 0; i < Np_; i++)
        {
                MatrixXd temp2 = MatrixXd::Identity(col_, col_);
                for (int j = 0; j < i + 1; j++)
                {
                        temp2 = temp2 * A;
                }
                PHIMatrix.block(i * row_, 0, row_, col_) = C * temp2;
        }
        return PHIMatrix;
}

Eigen::MatrixXd MPCController::mult_ComputeGAMMAMatrix(int mult_Y_num_i)
{
        MatrixXd A;
        MatrixXd C;
        if (!delta_mode_)
        {
                A = UpdateStateMatrixA();
                C = MatrixXd::Identity(mult_Y_num_i, X_num_);
        }
        if (delta_mode_)
        {
                A = d_UpdateStateMatrixA();
                C = MatrixXd::Zero(mult_Y_num_i, X_num_ + U_num_);
                C.block(0, 0, mult_Y_num_i, X_num_) = MatrixXd::Identity(mult_Y_num_i, X_num_);
        }
        int row_ = C.rows();   // is the Ny
        int size_A = A.rows(); // is the Nx
        int total_rows = Np_ * row_;
        int total_cols = Np_ * size_A;
        MatrixXd GAMMAMatrix = MatrixXd::Zero(total_rows, total_cols);
        MatrixXd RoMatrix = MatrixXd::Zero(total_rows, size_A); //RoMatrix is the first Nu colunm of the THETAMatrix
        for (int i = 0; i < Np_; i++)
        {
                MatrixXd temp3 = MatrixXd::Identity(size_A, size_A);
                for (int j = 0; j < i; j++)
                {
                        temp3 = A * temp3;
                }

                RoMatrix.block(i * row_, 0, row_, size_A) = C * temp3;
        }
        GAMMAMatrix.block(0, 0, row_ * Np_, size_A) = RoMatrix;
        for (int k = 1; k < Np_; k++)
        {
                MatrixXd temp4 = MatrixXd::Zero(k * row_, size_A);
                MatrixXd temp5 = MatrixXd::Zero(Np_ * row_, size_A);
                temp5.block(0, 0, k * row_, size_A) = temp4;
                temp5.block(k * row_, 0, (Np_ - k) * row_, size_A) = RoMatrix.block(0, 0, (Np_ - k) * row_, size_A);
                GAMMAMatrix.block(0, k * size_A, row_ * Np_, size_A) = temp5;
        }
        return GAMMAMatrix;
}
