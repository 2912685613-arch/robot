#ifndef C_CONTROLLER_H
#define C_CONTROLLER_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include"qpOASES.hpp"

using namespace std;
using namespace Eigen;
using namespace qpOASES;

namespace ControllerGroup {

/*!
    * \brief  PID ��������΢��
    */
class PID
{
public:
    PID();
    PID(float kp, float ki, float kd);
    void initPID(float kp, float ki, float kd);
    float getKp();
    void setKpPara(float Kp);
    float getKi();
    void setKiPara(float Ki);
    float getKd();
    void setKdPara(float Kd);
    float pidCalc(float fError);
    float pidNewCalc(float fError);
    float pidNewCalc(float expect, float real);
    void pidReset();

    bool enable;
    float i_limit;

private:
    float fKp, fKi, fKd;
    float fP, fI, fD;
    float fLastP, fLastI, fLastD;
    float fPreError, fCurError, fLastError;
    float fError_i, fError_d;
};



// =======================================================================
// ===           新增：DualLoopPID (双环/级联PID控制器)             ===
// =======================================================================
/*!
 * \brief  Dual-Loop / Cascade PID Controller (双环/级联PID控制器)
 * \details Encapsulates two PID controllers for cascade control,
 *          typically for position (outer loop) and velocity (inner loop).
 *          将两个PID控制器封装在一起用于级联控制，通常用于位置（外环）和速度（内环）。
 */
class DualLoopPID
{
public:
    DualLoopPID();

    /**
     * @brief 初始化双环PID的所有参数
     * @param kp_outer 外环Kp
     * @param ki_outer 外环Ki
     * @param kd_outer 外环Kd
     * @param kp_inner 内环Kp
     * @param ki_inner 内环Ki
     * @param kd_inner 内环Kd
     */
    void init(float kp_outer, float ki_outer, float kd_outer,
              float kp_inner, float ki_inner, float kd_inner);

    /**
     * @brief 计算双环PID的输出
     * @param target_pos 目标位置/角度
     * @param current_pos 当前位置/角度
     * @param current_vel 当前速度/角速度
     * @return 最终的控制输出 (例如：力或力矩)
     */
    float calculate(float target_pos, float current_pos, float current_vel);

    /**
     * @brief 重置两个环路的状态 (清除积分项和历史误差)
     */
    void reset();

    // --- 用于在线调参的接口 ---
    void setOuterLoopParams(float kp, float ki, float kd);
    void setInnerLoopParams(float kp, float ki, float kd);

    // --- 用于调试和监控的接口 ---
    float getOuterLoopError() const;
    float getOuterLoopOutput() const; // 即期望速度
    float getInnerLoopError() const;
    float getInnerLoopOutput() const; // 即最终的力

    /**
     * @brief 【新增】只计算内环，用于调试
     * @param desired_vel 手动设定的期望速度
     * @param current_vel 当前的实际速度
     * @return 最终的控制输出 (力或力矩)
     */
    float calculateInnerLoopOnly(float desired_vel, float current_vel);

    PID outer_loop_pid_; // 外环 (位置环)
    PID inner_loop_pid_; // 内环 (速度环)
private:
    // 用于存储中间调试信息的成员变量
    float last_outer_error_;
    float last_outer_output_; // 也就是 desired_velocity
    float last_inner_error_;
    float last_inner_output_; // 也就是 final_force
};
// =======================================================================
// ===                      新增部分结束                             ===
// =======================================================================

/*!
 * \brief  ADRC ����Ӧ���ſ��� position control
 */

class C_adrc // Auto Disturbances Rejection Controller
{
    // TD ESO NLSEF state variable
public:
    // TD state variable
    double m_v_1; // the input value
    double m_v_2; // the derivative of the input value
                  // this value could describe the speed
                  // of input increasement

    // ESO state variable
    double m_e;    // the error between the real output
                   // and zero order state observation output
    double m_fe_1; // a process variable, use it to calculate
                   // the first order state observation output
    double m_fe_2; // a process variable, use it to calculate
                   // the second order state observation output

    double m_z_1; // zero order output of state observation
    double m_z_2; // first order output of state observation
    double m_z_3; // second order output of state observation

    // NLSEF variable
    double m_e_1; // error between input and output
    double m_e_2; // error derivative between input and output
    double m_u_0; // original value of the controller output

    double m_u; // the controller output while has been conpensated
    double m_h; // the iterative step length

    // parameter table
    // these parameters are abstract, more details see .cpp file
    double m_r_0;
    double m_w;
    double m_beta_01;
    double m_beta_02;
    double m_beta_03;

    // need to be tackled
    double m_b_0;
    double m_c;
    double m_r;
    double m_h_1;

    // a new group need to be tackled
    double m_beta1;
    double m_beta2;
    double m_alpha1;
    double m_alpha2;
    double m_zeta;

    // some important functions
public:
    // more details of these functions are in .cpp file
    double sign(double x);
    double fhan(double x_1, double x_2, double r, double h);
    double fal(double x, double a, double delta);

    C_adrc();
    void SetadrcPmt(double k_p, double k_d, double band, double b_0);
    void SetadrcPmt_v2(double _k_p, double _k_d, double _h, double _r, double _w);

    double adrcOutput(double v, double y);
    //    double adrcOutput_v2(double v,double y);
    void clearGlobalVariables();
};

/*!
    * \brief  ADRC ����Ӧ���ſ��� speed control
    */

class C_adrc_speed              // Auto Disturbances Rejection Controller
{
//TD ESO NLSEF state variable
public:
    //TD state variable
    double v_1;         // the input value
    double v_2;         // the derivative of the input value
                          // this value could describe the speed
                          // of input increasement
    //ESO state variable
    double e;           // the error between the real output
                          // and zero order state observation output
    double z_1;         // zero order output of state observation
    double z_2;         // first order output of state observation

    //NLSEF variable
    double e_1;         // error between input and output
    double e_2;         // error derivative between input and output
    double u_0;         // original value of the controller output

    double u;           // the controller output while has been conpensated


    // parameter table
    // these parameters are abstract, more details see .cpp file
    double h;           // the iterative step length

    // TD
    double r_0;

    // ESO
    double w;
    double beta_01;
    double beta_02;
    double b_0;

    //NLSEF
    double k_p;
    double k_i;
    double k_d;
    double i_part;
    double i_limit;


    //some important functions
public:
    //more details of these functions are in .cpp file
    double sign(double x);
    double fhan(double x_1,double x_2,double r,double h);
    double fal(double x,double a,double delta);

    C_adrc_speed();
    void SetTDandESOPmt(double _sample_time, double _r_0, double _w, double _b_0);
    void SetNLSEFPmt(double _k_p, double _k_i, double _k_d, double _i_limit);

    double adrcOutput(double _v,double _y);
    void clearGlobalVariables(void);
};




//hcy
/*!
    * \brief  ADRC 自适应抗扰控制 position debug control
    */
class C_adrc_position              // Auto Disturbances Rejection Controller
{
//TD ESO NLSEF state variable
public:
    //TD state variable
    double v_1;         // the input value
    double v_2;         // the derivative of the input value
                          // this value could describe the speed
                          // of input increasement
    //ESO state variable
    double e;           // the error between the real output
                          // and zero order state observation output
    double z_1;         // zero order output of state observation
    double z_2;         // first order output of state observation
    double z_3;

    //NLSEF variable
    double e_1;         // error between input and output
    double e_2;         // error derivative between input and output
    double u_0;         // original value of the controller output

    double u;           // the controller output while has been conpensated


    // parameter table
    // these parameters are abstract, more details see .cpp file
    double h;           // the iterative step length (sample time)

    // TD
    double r_0;

    // ESO
    double w;
    double beta_01;
    double beta_02;
    double beta_03;
    double b_0;

    //NLSEF
    double k_p;
    double k_d;


    //some important functions
public:
    //more details of these functions are in .cpp file
    double sign(double x);
    double fhan(double x_1,double x_2,double r,double h);
    double fal(double x,double a,double delta);

    C_adrc_position();
    void SetadrcPosPmt(double _k_p, double _k_d, double _h, double _r, double _w, double _b_0);

    double adrcOutput(double _v,double _y);
    void clearGlobalVariables(void);
};





//hcy
/*!
    * \brief  ADRC 自适应抗扰控制 position debug control version2
    */
class C_adrc_position_v2              // Auto Disturbances Rejection Controller
{
//TD ESO NLSEF state variable
public:
    //TD state variable
    double v_1;         // the input value
    double v_2;         // the derivative of the input value
                          // this value could describe the speed
                          // of input increasement
    //ESO state variable
    double e;           // the error between the real output
                          // and zero order state observation output
    double z_1;         // zero order output of state observation
    double z_2;         // first order output of state observation
    double z_3;

    //NLSEF variable
    double e_1;         // error between input and output
    double e_2;         // error derivative between input and output
    double u_0;         // original value of the controller output

    double u;           // the controller output while has been conpensated


    // parameter table
    // these parameters are abstract, more details see .cpp file
    double h;           // the iterative step length (sample time)

    // TD
    double r_0;

    // ESO
    double w_0;
    double beta_01;
    double beta_02;
    double beta_03;
    double b_0;

    //NLSEF
    double k_p;
    double k_d;
    double w_c;

    //some important functions
public:
    //more details of these functions are in .cpp file
    double sign(double x);
    double fhan(double x_1,double x_2,double r,double h);
    double fal(double x,double a,double delta);

    C_adrc_position_v2();
    void setadrcPosPmt(double _r, double _w_c, double _w_0, double _b_0 );
    void setadrcPosPmt(double _r, double _k_p, double _k_d, double _w_0, double _b_0 );
    double adrcOutput(double _v,double _y);
    void clearGlobalVariables(void);
};



/*!
    * \brief  S surface ԭ��s�������
    */

class C_SsurfaceController_v1     // normal S surface controller
{
public:
    // the parameters of S surface controller
    double K_p;                   // the property coefficient of error
    double K_d;                   // the derivative coefficient of error
    double disturbance;           // the disturbance item of control system
    double outside_gain;          // the original controller only has an output
                                  // range from -1 to 1, so an outside gain was needed
    double step_time;             // step time of iterations
    double last_error;            // the last error
    double cur_error;             // the current error
    double cur_error_deriv;       // the current error derivative

    // input
    double expect_input;          // the input expect value
    double object_output;         // the real output of controlled object

    // output
    double org_controller_output; // the original controller output
                                  // without outside gain
    double controller_output;     // the output of SSC controller

    // some inportant function
    // more details just see .cpp file
    C_SsurfaceController_v1();
    double Sigmoid(double _input_value);
    void setSSCPmt(double _K_p, double _K_d, double _outside_gain);
    double CalSSCoutput(double _expect_input, double _object_output);
};



/*!
    * \brief  S surface v2   �Ľ���s�������
    */

class C_SsurfaceController_v2     // a new S surface controller
{
public:
    // the parameters of S surface controller
    double K_p;                   // the property coefficient of error
    double K_d;                   // the derivative coefficient of error
    double K_i;                   // the integral coefficient of error
    double disturbance;           // the disturbance item of control system
    double outside_gain;          // the original controller only has an output
                                  // range from -1 to 1, so an outside gain was needed
    double step_time;             // step time of iterations
    double last_error;            // the last error
    double cur_error;             // the current error
    double cur_error_deriv;       // the current error derivative
    double cur_error_integ;       // the current error integral

    int integ_switch;             // the switch of integral

    // input
    double expect_input;          // the input expect value
    double object_output;         // the real output of controlled object

    // output
    double org_controller_output; // the original controller output
                                  // without outside gain
    double controller_output;     // the output of SSC controller

    // some inportant function
    // more details just see .cpp file
    C_SsurfaceController_v2();
    double Sigmoid(double _input_value);
    void setSSCv2Pmt(double _K_p, double _K_i, double _K_d, double _outside_gain);
    double CalSSCv2output(double _expect_input, double _object_output);
};

/**************************************************************************/
/*
 * applied to the control of yaw
 */
class MPC {
public:
        MPC();

        //virtual ~MPC();


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
    //��ɢ�����״̬���̵�ϵ������
        Matrix2f Ad;
        Vector2f Bd;
        Vector2f mat;
        MatrixXf Cd;
         int p;                         //Ԥ�ⲽ��
         int  m;                         //�������벽��

        float u_1;                     //����ĳ�ʼֵ
        Vector2f X_1;                  //״̬�����ĳ�ʼֵ��ά����Ad�������������
        float lastyaw;


        MatrixXf R;
        MatrixXf Q;

   //Լ������
        float u_min;
        float u_max;
        float Ymin;
        float Ymax;
};
/* This is consensus with the simulation in matlab*/
/* State of the robot is x, y, phi, u and r, x_real is a 5 dimension vector
 * the objective function is basd on the equation 4.7 in the book <<wu ren jia shi che liang mo xing yu ce kong zhi>>
 * control step and predictive step is Tsim
 *
 * */
class MPC_trajectoryTracking
{
public:

        MPC_trajectoryTracking(int Vnum, int vTsium, int vNx,int vNu);
        //�����켣

        //�����ڴ��켣ʱ�����Ŀ�������
        MatrixXf Generate_RefTao(MatrixXf ref);
        //����mpc�����µĿ�������
        Vector2f Generate_realTao(MatrixXf ref, MatrixXf tao, VectorXf xreal);
        VectorXf getstate();
        void setstate(VectorXf x_real_current);





private:

        float umin1;
        float umax1;
        float umin2;
        float umax2;

        //ˮ�¾�������˶���ѧ������,�����Լ���ģ�ͽ�������

        Matrix2f M;                //��������M
        Matrix2f D11;             //ˮ����������������
        Matrix2f D12;             //ˮ�������������������ϵ��

        //ϵͳ�������

        int Num;                 //�ο��켣������
        int iter_num;              //�����Ĵ�����ԭ���ϲ��ܳ���Num
        int Tsim;                 //����ʱ��,���Ե��ڸ���Ч����Խ�����Ч��Խ�ã�̫�������������
        int Nx;                   //ϵͳ״̬������
        int Nu;                   //��������������

        //һЩ������
        MatrixXf q; // ��һ��Nx*Nx�� ����
        MatrixXf Q; // ��һ������ά����Nx*Tsim��q��Q�Խ����ϵľ���
        MatrixXf R;
        VectorXf x_real_last;

        MatrixXf setQ(MatrixXf vq, int vTsim)
        {
                int c = vq.cols();
                int r = vq.rows();
                MatrixXf vQ = MatrixXf::Zero(r*vTsim, c*vTsim);
                for (int i = 0; i < vTsim; i++)
                {
                        vQ.block(i*r, i*c, r, c) = vq;

                }
                return vQ;
        }

        void update_iter()
        {

                        iter_num = iter_num + 1;


        }


};
/********************************************************************************/
/*
 * This code can achieve trajectory independly.
 * The code is written based on the the chapter 4 of the book
 * <<wu ren jia shi che liang mo xing yu ce kong zhi>>
 * the state of the robot is the x,y,phi,u,r, and the xreal is a 5 dimension vector
 * */
class MPC_trajectoryTracking2
{
public:

	MPC_trajectoryTracking2(int vNum, int vNp,int vNc, int vNx, int vNu);
	//?��?��?��??
	MatrixXf  Generate_Reftrajectory();
	//?��?��?????��???��??????????????
	MatrixXf Generate_RefTao(MatrixXf ref);
	//?��??mpc????????????????
	Vector2f Generate_realTao(MatrixXf ref, MatrixXf tao, VectorXf xreal);





private:


	//???????��?��?��???????��???��????,?��??��??????????????��??

	Matrix2f M;                //???????��M
	Matrix2f D11;             //??????????����?????��
	Matrix2f D12;             //??????��?????����?????��??????

	//?????����??��??

	int Num;                 //?????��????????
	int iter_num;              //?��?��??????????��??????????Num
	int Np;                    //?�?????�
	int Nc;                    //???????�
	int Nx;                   //????��?????????
	int Nu;                   //??????????????
	int Row;                  //?????����?
	//
	Vector2f U_min;
	Vector2f U_max;

	float yaw_min;
	float yaw_max;


	//???????��????
	
	MatrixXf Q; // ??????��??��????????Nx*Tsim??q??Q?????????????��
	MatrixXf R;
	Vector2f U;
	
	void update_iter()
	{
		if (iter_num<Num)
		{
			iter_num = iter_num + 1;

		}
	}


};
/*******************************************************************************************/
/*
 * This is the control of yaw. the velocity of the robot is set a constant. The other is consensus
 * with the MPC_trajectoyTracking. state of the robot is x,y,phi and r,x_real is a 4dimension vector.
 * */
class MPC_trajectoryTrackingyaw
{
public:

	MPC_trajectoryTrackingyaw(int Vnum, int vTsium, int vNx, int vNu);
	//?��?��?��??

	//?��?��?????��???��??????????????
	VectorXf Generate_RefTao(MatrixXf ref);
	//?��??mpc????????????????
	float Generate_realTao(MatrixXf ref, VectorXf tao, VectorXf xreal, float velocity);
	VectorXf getstate();
	void setstate(VectorXf x_real_current);





private:



	float umin;
	float umax;

	//???????��?��?��???????��???��????,?��??��??????????????��??

	float M;                //???????��M
	float D11;             //??????????����?????��
	float D12;             //??????��?????����?????��??????


	//?????����??��??

	int Num;                 //?????��????????
	int iter_num;              //?��?��??????????��??????????Num
	int Tsim;                 //��????��??,?????��???����??��???????��?����??��???????????��???????��????
	int Nx;                   //????��?????????
	int Nu;                   //??????????????

	//???????��????
	MatrixXf q; // ??????Nx*Nx?? ???��
	MatrixXf Q; // ??????��??��????????Nx*Tsim??q??Q?????????????��
	MatrixXf R;
	VectorXf x_real_last;

	MatrixXf setQ(MatrixXf vq, int vTsim)
	{
		int c = vq.cols();
		int r = vq.rows();
		MatrixXf vQ = MatrixXf::Zero(r*vTsim, c*vTsim);
		for (int i = 0; i < vTsim; i++)
		{
			vQ.block(i*r, i*c, r, c) = vq;

		}
		return vQ;
	}

	void update_iter()
	{
		if (iter_num<Num)
		{
			iter_num = iter_num + 1;

		}
	}


};








}

#endif // C_CONTROLLER_H
