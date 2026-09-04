#include "../../include/asrobot/c_controller_group.h"

namespace ControllerGroup {

/*!
    * \brief  PID 比例积分微分 具体实现
    */
PID::PID()
{
    fCurError = 0;
    fLastError = 0;
    fPreError = 0;
    fKp = 0; fKi = 0; fKd = 0;
    fP = 0; fI = 0; fD = 0;
    fLastP = 0;
    fLastI = 0;
    fLastD = 0;
    fError_i = 0;
    fError_d = 0;
    enable = true;
    i_limit=15;
    //qDebug("PID::PID()");
}
PID::PID(float kp, float ki, float kd)
{
    fCurError = 0;
    fLastError = 0;
    fPreError = 0;
    fKp = kp; fKi = ki; fKd = kd;
    fP = 0; fI = 0; fD = 0;
    fLastP = 0;
    fLastI = 0;
    fLastD = 0;
    fError_i = 0;
    fError_d = 0;
    enable = true;
    i_limit=15;
    //qDebug("PID::PID()2");
}

void PID::initPID(float kp, float ki, float kd)
{
    fCurError = 0;
    fLastError = 0;
    fPreError = 0;
    fKp = kp; fKi = ki; fKd = kd;
    fP = 0; fI = 0; fD = 0;
    fLastP = 0;
    fLastI = 0;
    fLastD = 0;
    fError_i = 0;
    fError_d = 0;
    enable = true;
    i_limit=15;
    //qDebug("PID::PID()3");
}

float PID::getKp()
{
    return fKp;
}

void PID::setKpPara(float Kp)
{
    fKp = Kp;
}

float PID::getKi()
{
    return fKi;
}

void PID::setKiPara(float Ki)
{
    fKi = Ki;
}

float PID::getKd()
{
    return fKd;
}

void PID::setKdPara(float Kd)
{
    fKd = Kd;
}

/*@
* intr：增量式PID
* input：误差
* outout：输出控制量
*/
float PID::pidCalc(float fError)
{
    float fOutput = 0;
    fPreError = fLastError;
    fLastError = fCurError;
    fCurError = fError;
//    qDebug("p=%f,i=%f,d=%f\n",fKp,fKi,fKd);
    fOutput = fKp * (fCurError - fLastError)
            + fKi * fCurError
            + fKd * (fCurError - 2 * fLastError + fPreError);
    //qDebug("fOutput=%f\n",fOutput);
    return fOutput;
}

/*@
 * 非标准的、带平滑功能的位置式 PID 算法
 *
 */
float PID::pidNewCalc(float fError)
{
    float speed1_out;
    if(enable)
    {
        fError_i += fError;
        if(fError_i > i_limit) fError_i = i_limit;          //积分限幅
        else if(fError_i < -i_limit) fError_i = -i_limit;

        fError_d = fError - fLastError;
        fLastError = fError;

        fLastP = fP;
        fLastI = fI;
        fLastD = fD;

        fP = fKp*fError;
        fI = fKi*fError_i;
        fD = fKd*fError_d;
        speed1_out = (fP + fI + fD + fLastP + fLastI + fLastD) / 2; // 平均值计算相当于在控制器输出上加了一个简单的低通滤波器，这有助于减少控制信号的快速波动或“抖动”，使物理响应更平滑
    }
    else
    {
        speed1_out=0;
    }

    return speed1_out;
}


float PID::pidNewCalc(float expect, float real)
{
    float speed1_out;
    fPreError = fLastError; //上上次e
    fLastError = fCurError; //上次e
    fCurError = expect-real;//本次e
    fLastP = fP;
    fLastI = fI;
    fLastD = fD;

    fP = fKp*(fCurError - fLastError);
    fI = fKi*fCurError;
    fD = fKd*(fCurError - 2 * fLastError + fPreError);
    //if( i % 20 == 0 )
//    qDebug("\n ce=%f, le=%f, pe=%f \n", fCurError, fLastError, fPreError);
    speed1_out =  (fP + fI + fD + fLastP + fLastI + fLastD) / 2 ;
    return speed1_out;
}




/*!
    * \brief  ADRC 自适应抗扰 具体实现 调节位置用这个ADRC
    */

/* @
 * Introduction: instanciation of class ADRC
 *
 * Input: none
 *
 * Output: none
 *
 * Attention: these paramter should not modify randomly!!!!!
 */
C_adrc::C_adrc()
{
   //TD ESO NLSEF state variable

   //TD state variable
    m_v_1 = 0.0;
    m_v_2 = 0.0;

    //ESO state variable
    m_e = 0.0;
    m_fe_1 = 0.0;
    m_fe_2 = 0.0;
    m_z_1 = 0.0;
    m_z_2 = 0.0;
    m_z_3 = 0.0;

    //NLSEF variable
    m_e_1 = 0.0;
    m_e_2 = 0.0;
    m_u_0 = 0.0;

    //the control output
    m_u = 0.0;
    //the iterative step
    m_h = 0.05;    // 采样时间 sample_time


    //parameter table
    //static
//      m_r_0 = 10000;
//      m_beta_01 = 0.3;
//      m_beta_02 = 0.03;
//      m_beta_03 = 0.001;

    //need to be tackled
//      m_b_0 = 0.;
//      m_c = 0.0;
//      m_r = 100;
//      m_h_1 = 0.0;

    //a new group of parameters need to be tackled
    m_r_0 = 6;                 // this variable assosiate with the  过渡过程中的加速度环节 越大过度的越快
                                   // speed of transient process
    m_w = 30;                      // 状态观测3w法则的w
    m_beta_01 = 30;                // related to band-width beta01,02,03是状态观测器的三个参数
                                   // 3 * band_width
    m_beta_02 = 300;               // related to band-width
                                   // 3 * band_width * band_width
    m_beta_03 = 1000;              // related to band-width
                                   // band_width * band_width * band*width
    m_b_0 = 0.00;                  // conpensatory factor       // b_0是状态观测器的补偿因子
    m_beta1 = 6;                   // weight of property like PID P
    m_beta2 = 0.005;               // weight of derivative like PID D

    m_alpha1 = 0.8;                // the following three variables
    m_alpha2 = 1.5;                // are the parameters of function fal()
    m_zeta = 50;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//some important mathmatical functions

/*@
 * Introduction: a sign function, if the input value larger than 0
 *               the output is 1, otherwise the output is -1
 *
 * Input: real number
 *
 * Output: 1 or (-1) as the conditions
 */
double C_adrc::sign(double x)
{
    if(x>=0.0001)
        return 1;
    else if(x<=-0.0001)
        return -1;
    else
        return 0;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
 * Introduction: A nonlinear engineering function
 *
 * Input: two state variables x_1 x_2, transient speed variable r,
 *        and iteration step h
 *
 * Output: A nonlinear output
 */
/**
 * @description:
 * @param {double} x_1  误差 x1 - v
 * @param {double} x_2  速度 x2
 * @param {double} r       控制参数,大-->x1跟踪v更快,但可能震荡,小-->跟踪慢,但更平滑
 * @param {double} h      采样时间
 * @return {*}
 * @author: LJL
 */
double C_adrc::fhan(double x_1,double x_2,double r,double h)
{
    double d;
    double a_0;
    double y;
    double a_1;
    double a_2;
    double s_y;
    double a;
    double s_a;
    double fhan;

    d = r*h*h;
    a_0 = h*x_2;
    y = x_1 + a_0;
    a_1 = sqrt(d*( d+8*fabs(y) ));
    a_2 = a_0+sign(y)*(a_1 - d)/2.0;
    s_y = ( sign(y+d) - sign(y-d))/2.0;
    a = ( a_0+y-a_2 )*s_y +a_2;
    s_a = (sign(a+d) - sign(a-d))/2.0;
    fhan = -r*(a/d)*s_a - r*sign(a)*(1-s_a);

    return fhan;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
 * Introduction: A nonlinear engineering function
 *
 * Input: A state variable x, two parameters a and delta
 *
 * Output: A nonlinear output
 */
double C_adrc::fal(double x,double a,double delta)
{
    double fal;
    if(fabs(x) <= delta)
        fal = x / pow((double)delta , (double)(1.0 - a) );
    else
        fal = sign(x)*pow((double)fabs(x) ,a );
    return fal;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
 * Introduction: set controller parameters
 *
 * Input:
 *   k_p: the gain coefficient of error, like PID P
 *   k_d: the gain coefficient of error derivative like PID D
 *   band : the band width of input signal
 *   b_0 : the conpensate factor
 *
 * Output: none
 *
 * Attention: in practically, the b_0 has not been used
 */
void C_adrc::SetadrcPmt(double k_p,double k_d,double band,double b_0)
{
    m_beta1 = k_p;
    m_beta2 = k_d;
    m_beta_01 = 3.0*band;
    m_beta_02 = 3.0*band*band;
    m_beta_03 = band*band*band;
    m_w = band;
    m_b_0 = b_0;
}

void C_adrc::SetadrcPmt_v2(double _k_p, double _k_d, double _h, double _r , double _w)
{
    m_beta1 = _k_p;
    m_beta2 = _k_d;
    m_h = _h;
    m_r_0 = _r;
    m_w = _w;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
 * Introduction: produce the output of controller
 *
 * Input:
 *   v: the expect value of input
 *   y: the real output of the controlled object
 *
 * Output: the output of ADRC
 *
 * Attention: in practically, the b_0 has not been used
 */
//double C_adrc::adrcOutput(double v, double y)
//{
//    //TD
//    m_v_1 = m_v_1 + m_v_2*m_h;
//    m_v_2 = m_v_2 + fhan( m_v_1 - v , m_v_2 , m_r_0 , m_h )*m_h;
////    printf("m_v_1 is %.3f   m_v_2 is %.3f\n",m_v_1, m_v_2);

//    //ESO
//    // m_beta_01 = 3*band_width, m_beta_02 = 3*band_width^2,m_beta_02 = band_width^3
//    m_e = m_z_1 - y;
//    m_fe_1 = fal(m_e , 0.5, m_h);
//    m_fe_2 = fal(m_e , 0.25 , m_h);
//    m_z_1 = m_z_1 + (m_z_2 - m_beta_01 * m_e)*m_h;
//    m_z_2 = m_z_2 + (m_z_3 - m_beta_02 * m_fe_1 + m_b_0*m_u)*m_h;
//    //m_z_3 = m_z_3 + ( -m_beta_03 * m_fe_2)*m_h;

//    //printf("m_z_3 is %.3f\n",m_z_3);


//    //a new NLSEF
//    m_e_1 = m_v_1 - m_z_1;
//    m_e_2 = m_v_2 - m_z_2;

////    printf("m_e_1: %.2f    m_e_2: %.2f\n", m_e_1, m_e_2);

//    if(m_e_2 > 75)
//        m_e_2 = 75;
//    else if(m_e_2 < -75)
//        m_e_2 = -75;

//    m_u_0 = m_beta1 * fal(m_e_1,m_alpha1, m_zeta)+
//        m_beta2 * fal(m_e_2,m_alpha2, m_zeta);


//    //output
//    //constrain u_0
//    if(m_u_0 > 3)
//        m_u_0 = 3;
//    else if(m_u_0 < -3)
//        m_u_0 = -3;

////    printf("the controller output is  %.3f\n",m_u_0);
//    //m_u = m_u_0 - m_z_3/m_b_0;

//    m_u = m_u_0;
//    return m_u_0;
//}

//hcy:gazebo原程序p使用的
// adrc 新策略
/**
 * @description: 采用ADRC计算该方向的控制量
 * @param {double} v  期望的x/y/z
 * @param {double} y  当前的x/y/z
 * @return {*}m_u_0     控制信号
 * @author: LJL
 */
double C_adrc::adrcOutput(double v, double y)
{
    cout << "**************************ADRC计算参数**************************" << endl;
    // 1.跟踪微分器（TD）
    // 过渡过程，积分步长取采样时间 m_h，调节过渡过程速度 m_r_0
    /* m_v_1 和 m_v_2 是 v（目标输入）的平滑逼近：
         m_v_1：逼近 v 的状态变量
         m_v_2：逼近 v 的一阶导数（速度）
    */
    m_v_1 = m_v_1 + m_v_2*m_h;
    m_v_2 = m_v_2 + fhan(m_v_1 - v, m_v_2, m_r_0, m_h) * m_h; // fhan让 m_v_1 平滑跟随 v，不会突变
    printf("m_v_1: %.3f   m_v_2: %.3f\n", m_v_1, m_v_2);

    // 2.扩张状态观测器（ESO）
    // ESO 扩张状态观测器 采用线性状态观测器,需要调的参数更少
    /*ESO 估计系统状态，并观测外部扰动：
        m_z_1 估计系统的输出（位移）
        m_z_2 估计系统速度
        m_z_3 估计系统的总扰动
        m_w 控制 ESO 观测误差的收敛速度（通常与系统带宽 band_width 相关）
    */
    // m_beta_01 = 3*band_width, m_beta_02 = 3*band_width^2,m_beta_03 = band_width^3
    m_e = m_z_1 - y;
    m_z_1 = m_z_1 + (m_z_2 - 3*m_w *m_e)*m_h;
    m_z_2 = m_z_2 + (m_z_3 - 3*m_w*m_w *m_e + m_b_0*m_u)*m_h;
    m_z_3 = m_z_3 + ( -m_w*m_w*m_w *m_e)*m_h;
    // cout << " m_z_1: " << m_z_1
    //      << " m_z_2: " << m_z_2
    //      << " m_z_3: " << m_z_3 << endl;
    printf("m_z_1: %.3f   m_z_2: %.3f   m_z_3: %.3f\n", m_z_1, m_z_2, m_z_3);

    // 3.误差反馈 + PD 控制
    // 比例微分组合 其实用PD控制器即可
    m_e_1 = m_v_1 - m_z_1;//位置误差
    m_e_2 = m_v_2 - m_z_2;//速度误差

    m_u_0 = m_beta1 * m_e_1 + m_beta2 * m_e_2; // 采用 PD 控制计算控制量

   // 避免过大的控制输入
    if(m_u_0 > 4)
        m_u_0 = 4;
    else if(m_u_0 < -4)
        m_u_0 = -4;

    // 扰动不大 先不补偿了 直接输出
    m_u = m_u_0;
    return m_u_0;
}

void C_adrc::clearGlobalVariables()
{
    m_v_1 = 0;
    m_v_2 = 0;

    m_e = 0;
    m_fe_1 = 0;
    m_fe_2 = 0;

    m_z_1 = 0;
    m_z_2 = 0;
    m_z_3 = 0;

    m_u_0 = 0;
    m_u = 0;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*!
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/







/*!
    * \brief  ADRC 自适应抗扰 具体实现 做速度调节用这个版本的ADRC
    */

/* @
 * Introduction: instanciation of class ADRC
 *
 * Input: none
 *
 * Output: none
 *
 * Attention: these paramter should not modify randomly!!!!!
 */
C_adrc_speed::C_adrc_speed()
{
   //TD ESO NLSEF state variable

   //TD state variable
    v_1 = 0.0;
    v_2 = 0.0;

    //ESO state variable
    e = 0.0;

    z_1 = 0.0;
    z_2 = 0.0;


    //NLSEF variable
    e_1 = 0.0;
    e_2 = 0.0;
    u_0 = 0.0;

    //the control output
    u = 0.0;
    //the iterative step
    h = 0.05;    // 采样时间 sample_time

    //a new group of parameters need to be tackled
    r_0 = 6;                 // this variable assosiate with the  过渡过程中的加速度环节 越大过度的越快
                                   // speed of transient process
    w = 3;                      // 状态观测3w法则的w
    beta_01 = 3*w*w;                // related to band-width beta01,02,03是状态观测器的三个参数
                                   // 3 * band_width * band_width
    beta_02 = w*w*w;               // related to band-width
                                   // band_width * band_width * band_width
    b_0 = 0.00;                  // conpensatory factor       // b_0是状态观测器的补偿因子

    k_p = 1;                   // weight of property like PID P
    k_i = 0;               // weight of derivative like PID D
    k_d = 0;
    i_part = 0;
    i_limit = 100;

}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//some important mathmatical functions

/*@
 * Introduction: a sign function, if the input value larger than 0
 *               the output is 1, otherwise the output is -1
 *
 * Input: real number
 *
 * Output: 1 or (-1) as the conditions
 */
double C_adrc_speed::sign(double x)
{
    if(x>=0.0001)
        return 1;
    else if(x<=-0.0001)
        return -1;
    else
        return 0;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
 * Introduction: A nonlinear engineering function
 *
 * Input: two state variables x_1 x_2, transient speed variable r,
 *        and iteration step h
 *
 * Output: A nonlinear output
 */
double C_adrc_speed::fhan(double x_1,double x_2,double r,double h)
{
    double d;
    double a_0;
    double y;
    double a_1;
    double a_2;
    double s_y;
    double a;
    double s_a;
    double fhan;

    d = r*h*h;
    a_0 = h*x_2;
    y = x_1 + a_0;
    a_1 = sqrt(d*( d+8*fabs(y) ));
    a_2 = a_0+sign(y)*(a_1 - d)/2.0;
    s_y = ( sign(y+d) - sign(y-d))/2.0;
    a = ( a_0+y-a_2 )*s_y +a_2;
    s_a = (sign(a+d) - sign(a-d))/2.0;
    fhan = -r*(a/d)*s_a - r*sign(a)*(1-s_a);

    return fhan;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
 * Introduction: A nonlinear engineering function
 *
 * Input: A state variable x, two parameters a and delta
 *
 * Output: A nonlinear output
 */
double C_adrc_speed::fal(double x,double a,double delta)
{
    double fal;
    if(fabs(x) <= delta)
        fal = x / pow((double)delta , (double)(1.0 - a) );
    else
        fal = sign(x)*pow((double)fabs(x) ,a );
    return fal;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
 * Introduction: set controller parameters
 *
 * Input:
 *   k_p: the gain coefficient of error, like PID P
 *   k_d: the gain coefficient of error derivative like PID D
 *   band : the band width of input signal
 *   b_0 : the conpensate factor
 *
 * Output: none
 *
 * Attention: in practically, the b_0 has not been used
 */
void C_adrc_speed::SetTDandESOPmt(double _sample_time, double _r_0, double _w, double _b_0)
{
    h = _sample_time;
    r_0 = _r_0;
    w = _w;
    beta_01 = 3*w*w;
    beta_02 = w*w*w;
    b_0 = _b_0;
}

void C_adrc_speed::SetNLSEFPmt(double _k_p, double _k_i, double _k_d, double _i_limit)
{
    k_p = _k_p;
    k_i = _k_i;
    k_d = _k_d;
    i_part = 0;
    i_limit = _i_limit;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
 * Introduction: produce the output of controller
 *
 * Input:
 *   v: the expect value of input
 *   y: the real output of the controlled object
 *
 * Output: the output of ADRC
 *
 * Attention: in practically, the b_0 has not been used
 */

// adrc 新策略
double C_adrc_speed::adrcOutput(double _v, double _y)
{
    // 过渡过程 积分步长统一取采样时间m_h 过度过程速度r_0需要被调节
    v_1 = v_1 + v_2 * h;
    v_2 = v_2 + fhan( v_1 - _v , v_2 , r_0 , h )* h;
    printf("v_1 is %.3f   v_2 is %.3f\n", v_1, v_2);

    //ESO 扩张状态观测器 现已降到了2维
    e = z_1 - _y;
    z_1 = z_1 + (z_2 - beta_01 *e + b_0*u)*h;
    z_2 = z_2 + ( - beta_02 *e)*h;
    cout << " z_1: " << z_1
         << " z_2: " << z_2 << endl;

    // 比例积分微分组合
    e_1 = v_1 - z_1;     // 比例
    i_part += e_1;       // 积分
    e_2 = v_2 - z_2;     // 微分
    if(i_part > i_limit) // 积分限制幅值
        i_part = i_limit;
    else if(i_part < -i_limit)
        i_part = -i_limit;

    u_0 = k_p * e_1 +
          k_i * i_part +
          k_d * e_2;

    // 输出限制幅值
    if(u_0 > 6)
        u_0 = 6;
    else if(u_0 < -6)
        u_0 = -6;

    // 对原始控制量进行补偿
    u = u_0 - z_2/b_0;

    cout << " u_0: " << u_0 << "  " << " z_2/b_0: " << z_2/b_0 << endl;

    // 输出限制幅值u
    if(u > 6)
        u = 6;
    else if(u < -6)
        u = -6;

    return u;
}

void C_adrc_speed::clearGlobalVariables()
{
    // TD
    v_1 = 0;
    v_2 = 0;

    // ESO
    e = 0;
    z_1 = 0;
    z_2 = 0;

    e_1 = 0;
    e_2 = 0;
    u_0 = 0;
    u = 0;
}




/* * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * *  * * * * *hcy * * * * * * *  * * * * * * * * *  * * * * * * * * * * * * * * * * * * * */

//hcy
C_adrc_position::C_adrc_position()
{
   //TD ESO NLSEF state variable

   //TD state variable
    v_1 = 0.0;
    v_2 = 0.0;

    //ESO state variable
    e = 0.0;

    z_1 = 0.0;
    z_2 = 0.0;
    z_3 = 0.0;

    //NLSEF variable
    e_1 = 0.0;
    e_2 = 0.0;
    u_0 = 0.0;

    //the control output
    u = 0.0;
    //the iterative step
    h = 0.05;    // 采样时间 sample_time

    //a new group of parameters need to be tackled
    r_0 = 6;                 // this variable assosiate with the  过渡过程中的加速度环节 越大过度的越快
                                   // speed of transient process
    w = 3;                      // 状态观测3w法则的w
    beta_01 = 3*w;                // related to band-width beta01,02,03是状态观测器的三个参数
                                   // 3 * band_width * band_width
    beta_02 = 3*w*w;               // related to band-width

    beta_03 = w*w*w;              // band_width * band_width * band_width
    b_0 = 0.00;                  // conpensatory factor       // b_0是状态观测器的补偿因子

    k_p = 1;                   // weight of property like PID P
    k_d = 0;

}

//hcy
C_adrc_position_v2::C_adrc_position_v2()
{
   //TD ESO NLSEF state variable

   //TD state variable
    v_1 = 0.0;
    v_2 = 0.0;

    //ESO state variable
    e = 0.0;

    z_1 = 0.0;
    z_2 = 0.0;
    z_3 = 0.0;

    //NLSEF variable
    e_1 = 0.0;
    e_2 = 0.0;
    u_0 = 0.0;

    //the control output
    u = 0.0;
    //the iterative step
    // h = 0.01;    // 采样时间 sample_time 50hz 33hz 40hz 60hz 50hz
    h = 0.01;    // 采样时间 sample_time 50hz 33hz 40hz 60hz 50hz

    //a new group of parameters need to be tackled
    r_0 = 6;                 // this variable assosiate with the  过渡过程中的加速度环节 越大过度的越快
                                   // speed of transient process
    w_0 = 3;                      // 状态观测3w法则的w
    beta_01 = 3*w_0;                // related to band-width beta01,02,03是状态观测器的三个参数
                                   // 3 * band_width * band_width
    beta_02 = 3*w_0*w_0;               // related to band-width

    beta_03 = w_0*w_0*w_0;              // band_width * band_width * band_width
    b_0 = 1;                  // conpensatory factor       // b_0是状态观测器的补偿因子

    w_c = 2;
    k_p = w_c*w_c;                   // weight of property like PID P
    k_d = 2*w_c;

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//hcy:some important mathmatical functions
double C_adrc_position::sign(double x)
{
    if(x>=0.0001)
        return 1;
    else if(x<=-0.0001)
        return -1;
    else
        return 0;
}

double C_adrc_position::fhan(double x_1,double x_2,double r,double h)
{
    double d;
    double a_0;
    double y;
    double a_1;
    double a_2;
    double s_y;
    double a;
    double s_a;
    double fhan;

    d = r*h*h;
    a_0 = h*x_2;
    y = x_1 + a_0;
    a_1 = sqrt(d*( d+8*fabs(y) ));
    a_2 = a_0+sign(y)*(a_1 - d)/2.0;
    s_y = ( sign(y+d) - sign(y-d))/2.0;
    a = ( a_0+y-a_2 )*s_y +a_2;
    s_a = (sign(a+d) - sign(a-d))/2.0;
    fhan = -r*(a/d)*s_a - r*sign(a)*(1-s_a);

    return fhan;
}

double C_adrc_position::fal(double x,double a,double delta)
{
    double fal;
    if(fabs(x) <= delta)
        fal = x / pow((double)delta , (double)(1.0 - a) );
    else
        fal = sign(x)*pow((double)fabs(x) ,a );
    return fal;
}

double C_adrc_position_v2::sign(double x)
{
    if(x>=0.0001)
        return 1;
    else if(x<=-0.0001)
        return -1;
    else
        return 0;
}

double C_adrc_position_v2::fhan(double x_1,double x_2,double r,double h)
{
    double d;
    double a_0;
    double y;
    double a_1;
    double a_2;
    double s_y;
    double a;
    double s_a;
    double fhan;

    d = r*h*h;
    a_0 = h*x_2;
    y = x_1 + a_0;
    a_1 = sqrt(d*( d+8*fabs(y) ));
    a_2 = a_0+sign(y)*(a_1 - d)/2.0;
    s_y = ( sign(y+d) - sign(y-d))/2.0;
    a = ( a_0+y-a_2 )*s_y +a_2;
    s_a = (sign(a+d) - sign(a-d))/2.0;
    fhan = -r*(a/d)*s_a - r*sign(a)*(1-s_a);

    return fhan;
}

double C_adrc_position_v2::fal(double x,double a,double delta)
{
    double fal;
    if(fabs(x) <= delta)
        fal = x / pow((double)delta , (double)(1.0 - a) );
    else
        fal = sign(x)*pow((double)fabs(x) ,a );
    return fal;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


//hcy:（重要）4个参数的adrc
void C_adrc_position_v2::setadrcPosPmt(double _r, double _w_c, double _w_0, double _b_0 )
{
    r_0 = _r;

    w_c = _w_c;
    k_p = w_c * w_c;
    k_d = 2 * w_c;

    w_0 = _w_0;
    beta_01 = 3*w_0;
    beta_02 = 3*w_0*w_0;
    beta_03 = w_0*w_0*w_0;

    b_0 = _b_0;
}

//hcy:（重要）5个参数的adrc
void C_adrc_position_v2::setadrcPosPmt(double _r, double _k_p, double _k_d, double _w_0, double _b_0 )
{
    r_0 = _r;

    k_p = _k_p;
    k_d = _k_d;

    w_0 = _w_0;
    beta_01 = 3*w_0;
    beta_02 = 3*w_0*w_0;
    beta_03 = w_0*w_0*w_0;

    b_0 = _b_0;
}

//hcy:（重要）6个参数的adrc
void C_adrc_position::SetadrcPosPmt(double _k_p, double _k_d,
                                    double _h, double _r,
                                    double _w, double _b_0)
{
    k_p = _k_p;
    k_d = _k_d;
    h = _h;
    r_0 = _r;

    w = _w;
    beta_01 = 3*w;
    beta_02 = 3*w*w;
    beta_03 = w*w*w;

    b_0 = _b_0;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//hcy
double C_adrc_position::adrcOutput(double _v, double _y)
{
    // 过渡过程 积分步长统一取采样时间m_h 过度过程速度r_0需要被调节
    v_1 = v_1 + v_2 * h;
    v_2 = v_2 + fhan( v_1 - _v , v_2 , r_0 , h )* h;
    printf("v_1 is %.3f   v_2 is %.3f\n", v_1, v_2);

    //ESO 扩张状态观测器 现已降到了2维
    e = z_1 - _y;
    z_1 = z_1 + (z_2 - beta_01 *e )*h;
    z_2 = z_2 + (z_3 - beta_02 *e + b_0 * u)*h;
    z_3 = z_3 + (-beta_03 *e)*h;
    cout << " z_1: " << z_1
         << " z_2: " << z_2
         << " z_3: " << z_3 << endl;

    // 比例积分微分组合
    e_1 = v_1 - z_1;     // 比例
    e_2 = v_2 - z_2;     // 微分

    /*u_0 = k_p * e_1 +
          k_d * e_2;*/

 /*u_0 = k_p * e_1 +
          k_d * e_2;*/
    u_0 = k_p * fal(e_1, 0.5, 0.4) +
          k_d * fal(e_2, 0.5, 0.4);

    // 输出限制幅值
    if(u_0 > 6)
        u_0 = 6;
    else if(u_0 < -6)
        u_0 = -6;

    // 对原始控制量进行补偿
    u = u_0 - z_3/b_0;

    cout << " u_0: " << u_0 << "  " << " z_3/b_0: " << z_3/b_0 << endl;

    // 输出限制幅值u
    if(u > 6)
        u = 6;
    else if(u < -6)
        u = -6;
    // 返回u_0;
    return u_0;
}
//hcy
void C_adrc_position::clearGlobalVariables()
{
    // TD
    v_1 = 0;
    v_2 = 0;

    // ESO
    e = 0;
    z_1 = 0;
    z_2 = 0;
    z_3 = 0;

    e_1 = 0;
    e_2 = 0;
    u_0 = 0;
    u = 0;
}


//hcy:（重要）输入：期望位置，实际位置，输出：力
double C_adrc_position_v2::adrcOutput(double _v, double _y)
{
    // 过渡过程 积分步长统一取采样时间m_h 过度过程速度r_0需要被调节
    v_1 = v_1 + v_2 * h;
    v_2 = v_2 + fhan( v_1 - _v , v_2 , r_0 , h )* h;
    printf("v_1 is %.3f   v_2 is %.3f\n", v_1, v_2);

    //ESO 扩张状态观测器
    e = z_1 - _y;
    z_1 = z_1 + (z_2 - beta_01 *e )*h;
    z_2 = z_2 + (z_3 - beta_02 *e + b_0 * u)*h;
    z_3 = z_3 + (-beta_03 *e)*h;
    cout << " z_1: " << z_1
         << " z_2: " << z_2
         << " z_3: " << z_3 << endl;

    // 比例积分微分组合
    e_1 = v_1 - z_1;     // 比例
    e_2 = v_2 - z_2;     // 微分

    /*u_0 = k_p * fal(e_1, 0.5, 0.1) +
          k_d * fal(e_2, 0.5, 0.1);*/

    u_0 = k_p * e_1 +
          k_d * e_2; 

    // u_0 = k_p * e_1;


    // 输出限制幅值
    // if(u_0 > 3)//hcy
    //    u_0 = 3;
    // else if(u_0 < -3)
    //    u_0 = -3;

    // 对原始控制量进行补偿
     u = (u_0 - z_3)/b_0;

     // u = u_0;     

    /// ADRC TEST
    // u = 0.0;
    ///
    cout << " k_p: " << k_p << "  " << " k_d " << k_d << endl;
    cout << "w_0: " << w_0 <<endl;
    cout << " u_0: " << u_0 << "  " << " z_3/b_0: " << z_3/b_0 << endl;
    cout << " h: " << h <<" " << endl;
    // 输出限制幅值u
    if(u > 2)//hcy
        u = 2;
    else if(u < -2)
        u = -2;
    // if(u > 6)//hcy
    //     u = 6;
    // else if(u < -6)
    //     u = -6;
    cout << " u: " << u << "  " << endl;
    // 返回u;
    return u;
}
//hcy
void C_adrc_position_v2::clearGlobalVariables()
{
    // TD
    v_1 = 0;
    v_2 = 0;

    // ESO
    e = 0;
    z_1 = 0;
    z_2 = 0;
    z_3 = 0;

    e_1 = 0;
    e_2 = 0;
    // NLSEF
    u_0 = 0;
    u = 0;
}

/* * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * *  * * * * ** * * * * * *  * * * * * * * * *  * * * * * * * * * * * * * * * * * * * */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*!
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/






/*!
    * \brief S surface 原版s面控制器 具体实现
    */


/*!
* the function implemention of class SsurfaceController v1
*/

/*@
* Introduction: instanciation of class SsurfaceController
*
* Input: none
*
* Output: none
*
* Attention: these paramter should not modify randomly!!!!!
*/
C_SsurfaceController_v1::C_SsurfaceController_v1()
{
    K_p = 0.0;
    K_d = 0.0;
    disturbance = 0.0;
    outside_gain = 0.0;

    step_time = 0.03;         // the program circling time is 30HZ
    last_error = 0.0;
    cur_error = 0.0;
    cur_error_deriv = 0.0;    // this variable will be calculated
                              // by last_error and cur_error
    expect_input = 0.0;
    object_output = 0.0;
    controller_output = 0.0;

    cout<< "the step time has been set to: "<< step_time <<endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
* Introduction: A Engineer function, whose shape like "S"
*
* Input: real number
*
* Output: a real number which range from -1 to 1
*
*/
double C_SsurfaceController_v1::Sigmoid(double _input_value)
{
    double _output_value;
    _output_value = 2.0 / ( 1.0 + exp( -_input_value ) ) - 1;
    return _output_value;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
* Introduction: Set key parameter of Ssurface controller
*
* Input:
*   _K_p: the property coefficient of error, like PID P
*   _K_d: the coefficient of error derivative, like PID D
*   _outside_gain: controller_output = outside_gain * org_controller_output
*
* Output: None
*
*/
void C_SsurfaceController_v1::setSSCPmt(double _K_p, double _K_d, double _outside_gain)
{
    K_p = _K_p;
    K_d = _K_d;
    outside_gain = _outside_gain;
    cout<< "the parameters of Ssurface Controller has been set"<<endl;
    cout << "K_p: "<< K_p << "K_d: "<< K_d
         <<"outside_gain: "<< outside_gain << endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
* Introduction: Calculate SSC output
*
* Input:
*   _expect_input: the expect value of input
*   _object_output: the real output of controlled object
*
* Output: the output of this controller
*
*/
double C_SsurfaceController_v1::CalSSCoutput(double _expect_input, double _object_output)
{
    expect_input = _expect_input;
    object_output = _object_output;

    cur_error = expect_input - object_output;
    cur_error_deriv = (cur_error - last_error)/step_time;

    org_controller_output = Sigmoid( K_p*cur_error + K_d*cur_error_deriv )
                            + disturbance; // actually the value of disturbance is 0
    controller_output = outside_gain * org_controller_output;// the final output
    last_error = cur_error; // used in next iteration
    cout<< "the controller output is: "<< controller_output <<endl;
    return controller_output;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*!
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/






/*!
    * \brief S surface v2 改进版s面控制器 具体实现
    */


/*!
* the function implemention of class SsurfaceController v2
*/
/*@
* Introduction: instanciation of class SsurfaceController
*
* Input: none
*
* Output: none
*
* Attention: these paramter should not modify randomly!!!!!
*/
C_SsurfaceController_v2::C_SsurfaceController_v2()
{
    K_p = 0.0;
    K_d = 0.0;
    K_i = 0.0;
    disturbance = 0.0;
    outside_gain = 0.0;

    step_time = 0.03;
    last_error = 0.0;
    cur_error = 0.0;
    cur_error_deriv = 0.0;
    cur_error_integ = 0.0;

    integ_switch = 0;

    expect_input = 0.0;
    object_output = 0.0;

    org_controller_output = 0.0;
    controller_output = 0.0;

    cout<< "the step time has been set to: "<< step_time << endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
* Introduction: A Engineer function, whose shape like "S"
*
* Input: real number
*
* Output: a real number which range from -1 to 1
*
*/
double C_SsurfaceController_v2::Sigmoid(double _input_value)
{
    double _output_value;
    _output_value = 2.0 / ( 1.0 + exp( -_input_value ) ) - 1;
    return _output_value;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
* Introduction: Set key parameter of Ssurface controller
*
* Input:
*   _K_p: the property coefficient of error, like PID P
*   _K_d: the coefficient of error derivative, like PID D
*   _K_i: the coefficient of error integral, like PID I
*   _outside_gain: controller_output = outside_gain * org_controller_output
*
* Output: None
*
*/
void C_SsurfaceController_v2::setSSCv2Pmt(double _K_p, double _K_i, double _K_d, double _outside_gain)
{
    K_p = _K_p;
    K_i = _K_i;
    K_d = _K_d;
    outside_gain = _outside_gain;
    cout<< "the K_p is: " << K_p << "the K_p is: "<< K_i
        << "the K_d is: " << K_d << "outside_gain is: " << outside_gain;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
* Introduction: Calculate SSC output
*
* Input:
*   _expect_input: the expect value of input
*   _object_output: the real output of controlled object
*
* Output: the output of this controller
*
*/
double C_SsurfaceController_v2::CalSSCv2output(double _expect_input, double _object_output)
{
    expect_input = _expect_input;
    object_output = _object_output;

    cur_error = expect_input - object_output;
    cur_error_deriv = ( cur_error - last_error )/step_time;
    cur_error_integ += cur_error;

    // according to the current error and error derivation
    // to choose integral item
    if( cur_error * cur_error_deriv < 0)
    {
        integ_switch = 1;
    }
    else
    {
        integ_switch = 0;
    }

    org_controller_output = Sigmoid(K_p * cur_error + K_d * cur_error_deriv)
                            + integ_switch * K_i * cur_error_integ;
    controller_output = outside_gain * org_controller_output;

    last_error = cur_error; // used in next iteration

    cout << "the control output is:  "<< controller_output<< endl;
    return  controller_output;
}
/********************************************************************/
//MPC 类的定义
MPC::MPC( )
{
        Ad << 1, 0.0162,
                0, 0.0541;//这些值是怎么得到的
        Bd << 0.2758, 7.7215;
        mat << 1, 0;
        Cd = mat.transpose();
        p = 15;
        m=3;
        u_1 = 0;
        X_1 = Vector2f::Zero();
        u_min = -6;
        u_max = 6;
        Ymax = 180 ;
        Ymin = -180;
        lastyaw=0;
        R = MatrixXf::Identity(p, p);
        Q = 10*MatrixXf::Identity(m, m);

}
void MPC::set_state_CoeffMatrix()
{
        Ad << 1, 0.0162,
                  0, 0.0541;
        Bd << 0.2758, 7.7215;
        mat<< 1,0;
        Cd = mat.transpose();
}
void MPC::set_steplength(int m1, int p1)
{
        m = m1;
        p = p1;
        //设置权重矩阵Q 和R
        R = MatrixXf::Identity(p1, p1);
        Q = MatrixXf::Identity(m1, m1);
}
MatrixXf MPC::ComputeSu()
{
        //计算矩阵Su
        MatrixXf Su(p,m);
        VectorXf Ro(p);
        MatrixXf temp1 = Cd*Bd;
        Ro(0) = temp1(0, 0);
        for (int i = 2; i <= p; i++)
        {
                Matrix2f temp2 = Matrix2f::Identity();
                for (int j = i - 1; j >= 1; j--)
                {
                        temp2 = Ad *temp2;

                }
                temp1=Cd*temp2*Bd+temp1;
                Ro(i-1)= temp1(0,0);
        }

        Su.col(0) = Ro;
        for (int i = 1; i < m;i++)
        {

                Su.block(0, i, i, 1) = MatrixXf::Zero(i,1);
                Su.block(i, i, p - i,1) = Ro.head(p-i);
        }
    return  Su;
}

MatrixXf MPC::ComputeSx()
{

        Matrix2f temp3 = Matrix2f::Identity();
        MatrixXf Sx(p,2);
        MatrixXf temp4 = Cd*Ad;
        Sx.row(0) = temp4;
        for (int i = 1; i < p;i++)
        {
                for (int j = i + 1; j > 0; j--)
                {
                        temp3 = Ad*temp3;
                }
                temp4 = temp4 + Cd*temp3;
                Sx.row(i) = temp4;
        }
        return Sx;
}
//没有约束的MPC
float MPC::Solve(MatrixXf Su, MatrixXf Sx, float reference)
{
        MatrixXf X = Ad*X_1 + Bd*u_1;//实际系统中用传感器测量到的值计算
        MatrixXf y = Cd*X;
        MatrixXf deltax = X - X_1;
        MatrixXf coeff;
        coeff= MatrixXf::Ones(p,1);
        MatrixXf error = reference*coeff - Sx*deltax - y(0,0)*coeff;
        MatrixXf F = (Su.transpose()*R.transpose()*R*Su + Q.transpose()*Q).inverse()*Su.transpose()*R.transpose()*R*error;
        float u = u_1 + F(0, 0);
        u_1 = u;
        X_1 = X;
        return y(0,0);

}
//带约束的MPC
float MPC::QpSolverset(MatrixXf Su, MatrixXf Sx, float reference, MatrixXf A, float c_yaw)
{
        //cout << u_1 << endl;


        stateset(c_yaw);
        MatrixXf X = Ad*X_1 + Bd*u_1;//实际系统中用传感器测量到的值计算
        MatrixXf y = Cd*X;
        MatrixXf deltax = X - X_1;
        MatrixXf coeff;
        coeff = MatrixXf::Ones(p, 1);
        MatrixXf error = reference*coeff - Sx*deltax - y(0, 0)*coeff;
        //QP问题设置
        MatrixXf H=Su.transpose()*R.transpose()*R*Su + Q.transpose()*Q;
        MatrixXf g=Su.transpose()*R.transpose()*R*(-1)*error;
        MatrixXf lb2_matrix( m, 1);
        MatrixXf ub2_matrix(m, 1);
        lb2_matrix = (u_min-u_1)*MatrixXf::Ones(m, 1);
        ub2_matrix = (u_max-u_1)*MatrixXf::Ones(m, 1);
        MatrixXf lb3_matrix(p, 1);
        MatrixXf ub3_matrix(p, 1);
        ub3_matrix = Ymax*coeff - Sx*deltax - y(0, 0)*coeff;
        lb3_matrix = Ymin*coeff -Sx*deltax - y(0, 0)*coeff;
        MatrixXf lb_matrix(m + p, 1);
        lb_matrix.block(0, 0, m, 1) = lb2_matrix;
        lb_matrix.block(m, 0, p, 1) = lb3_matrix;
        MatrixXf ub_matrix(m + p, 1);
        ub_matrix.block(0, 0, m, 1) = ub2_matrix;
        ub_matrix.block(m, 0, p, 1) = ub3_matrix;

        //矩阵转换
        const auto NumOfMatrixH = H.rows()*H.cols();
        double *h_matrix;
        h_matrix=new double[NumOfMatrixH];
        const auto NumOfMatrixg = g.cols()*g.rows();
        double*g_matrix;
        g_matrix=new double[NumOfMatrixg];
        int index = 0;
        for (int i = 0; i < H.rows(); i++)
        {
                for (int j = 0; j < H.cols();j++)
                {
                        h_matrix[index] = H(i, j);
                        index = index + 1;
                }
        }

        for (int i = 0; i < g.rows(); i++)
        {

                        g_matrix[i] = g(i,0);

                }

        const auto NumOfMatrixA = A.rows()*A.cols();
        double *A_matrix;
        A_matrix=new double [NumOfMatrixA];
        int index1 = 0;
        for (int i = 0; i < A.rows(); i++)
        {
                for (int j = 0; j < A.cols(); j++)
                {
                        A_matrix[index1] = A(i, j);
                        index1 = index1 + 1;
                }

        }
        const auto NumOfMatrixubA = ub_matrix.cols()*ub_matrix.rows();
        double *ubA;
        ubA=new double [NumOfMatrixubA];
        for (int i = 0; i < ub_matrix.rows(); i++)
        {

                ubA[i] = ub_matrix(i, 0);

        }
        const auto NumOfMatrixlbA = lb_matrix.cols()*lb_matrix.rows();
        double *lbA;
        lbA = new double[NumOfMatrixlbA];
        for (int i = 0; i < lb_matrix.rows(); i++)
        {

                lbA[i] = lb_matrix(i, 0);

        }

        QProblem solveQP(m, 1);
        Options options;
        options.printLevel=PL_NONE;
        solveQP.setOptions(options);
        real_t *lb = NULL;
        real_t *ub= NULL;
        int_t nWSR = 5;
        solveQP.init(h_matrix, g_matrix, A_matrix, lb, ub, lbA, ubA, nWSR);
        real_t *xOpt;
        xOpt = new real_t[m];
        solveQP.getPrimalSolution(xOpt);
        //更新状态
        float u = u_1 + xOpt[0];
        u_1 = u;
        lastyaw=c_yaw;
        delete [] h_matrix;
        delete [] g_matrix;
        delete [] A_matrix;
        delete [] ubA;
        delete [] lbA;
        //X_1 = X;
        return u_1;
}


MatrixXf MPC::ComputeConstrain(MatrixXf Su)
{
        MatrixXf L = MatrixXf::Zero(m, m);
        L.col(0) = VectorXf::Ones(m);
        for (int i = 1; i < m;i++)
        {
                L.block(0, i, i, 1) = MatrixXf::Zero(i, 1);
                L.block(i, i, m - i, 1) = MatrixXf::Ones(m - i, 1);
        }
        MatrixXf A2( m, m);
        A2 = L;

        MatrixXf A3(p,m);
        A3= Su;

        int total_row = p + m;
        MatrixXf A(total_row, m);
        A.block(0, 0,  m, m) = A2;
        A.block( m, 0, p, m) = A3;
        return A;
}
void MPC::stateset(float currentyaw)
{
        X_1(0)=currentyaw;
        X_1(1)=(currentyaw-lastyaw)/0.05;  //0.05 is sampleing time
}
/***************************************************************************************************************/
/**************************************************************************************************************/
MPC_trajectoryTracking::MPC_trajectoryTracking(int vNum, int vTsium, int vNx, int vNu)
{
        umin1 = -6;
        umax1 = 6;
        umin2 = -0.09;
        umax2 = 0.09;
         M << 0.00142, 0,
                 0, 0.0021;

         D11 << 0.0163, 0,
                  0, 0.1225;
         D12 << -0.000055, 0,
                   0, -0.00032702;
         Num = vNum;
         iter_num = 0;
         Tsim = vTsium;
         Nx = vNx;
         Nu = vNu;
         R = 0.1* MatrixXf::Identity(Nu*Tsim, Nu*Tsim);
         q = MatrixXf::Identity(Nx, Nx);
         Q = setQ(q, Tsim);
         x_real_last=VectorXf::Zero(5);
         //cout << Q;
}

MatrixXf MPC_trajectoryTracking::Generate_RefTao(MatrixXf ref)
{
         //注意，对每一个MatrixXf赋值之前必须给出矩阵的大小，否则会出错

        MatrixXf tao = MatrixXf::Zero(Num, 2);
        Vector2f vd = Vector2f::Zero();
        Matrix2f temp;
        MatrixXf D;
        for (int k = 0; k < Num; k++)
        {
                if (k>1)
                {
                        vd = (ref.block(k, 3, 1, 2) - ref.block(k - 1, 3, 1, 2)).transpose()/ 0.05;

                }

                temp << ref(k, 3), 0,
                            0, ref(k, 4);
                 D = D11 + D12*temp;
            tao.block(k, 0, 1, 2) = ( D*ref.block(k, 3, 1, 2).transpose()+ M*vd).transpose();

        }
        return tao;
}

Vector2f MPC_trajectoryTracking::Generate_realTao( MatrixXf ref, MatrixXf tao, VectorXf xreal)
{

    update_iter();
        if (iter_num <=Num)
        {
                cout << "now is: "<<iter_num << " Reference Point"<<endl;
                if (iter_num == 1)
                {
                        xreal(3) = 0;// if it is the first iteration, xreal  is the init of the robot, set the velocity of the robot is 0
                        xreal(4)=0;//    if it is the first iteration, xreal  is the init of the robot, set the yawvelocity of the robot is 0
                }
                VectorXf x_piao;
                x_piao = xreal-ref.block(iter_num - 1, 0, 1, 5).transpose();

                cout<<"误差信息:"<<"Error_X:"<<x_piao(0)<<";"<<"Error_Y:"<<x_piao(1)<<";"<<"Error_yaw:"<<x_piao(2)<<"***"<<"Error_u:"<<x_piao(3)<<";"<<"Error_r:"<<x_piao(4)<<endl;

                Vector2f real_tao = Vector2f::Zero();//需要返回的向量
                float t_d = ref(iter_num - 1, 2);
                float u_d = ref(iter_num - 1, 3);
                float r_d = ref(iter_num - 1, 4);
                //求解矩阵a
                Matrix2f temp1;
                Matrix3f A11;
                float Time_sampling=0.05;//设置采样时间
                A11 << 0, 0, -u_d*sin(t_d)*Time_sampling,
                        0, 0, u_d*cos(t_d)*Time_sampling,
                        0, 0, 0;
                MatrixXf A12(3, 2);
                A12 << cos(t_d)*Time_sampling, 0,
                        sin(t_d)*Time_sampling, 0,
                        0, 1 * Time_sampling;
                MatrixXf A21 = MatrixXf::Zero(2, 3);
                temp1 << u_d, 0,
                        0, r_d;
                MatrixXf A22;
                A22 = -(M.inverse()*D11 + 2 * M.inverse()*D12*temp1)*Time_sampling;
                MatrixXf time_a(Nx, Nx);
                time_a.block(0, 0, 3, 3) = A11;
                time_a.block(0, 3, 3, 2) = A12;
                time_a.block(3, 0, 2, 3) = A21;
                time_a.block(3, 3, 2, 2) = A22;
                MatrixXf a = MatrixXf::Identity(5, 5) + time_a;

                //求解矩阵b
                MatrixXf b = MatrixXf::Zero(Nx, Nu);
                b.block(3, 0, 2, 2) = M.inverse()*Time_sampling;

                //求解矩阵A
                MatrixXf A(Tsim*Nx, Nx);
                for (int k = 0; k < Tsim; k++)
                {
                        MatrixXf temp2 = MatrixXf::Identity(Nx, Nx);
                        for (int j = 0; j < k + 1; j++)
                        {
                                temp2 = temp2*a;
                        }
                        A.block(k*Nx, 0, Nx, Nx) = temp2;

                }
                //求解矩阵
                MatrixXf B = MatrixXf::Zero(Tsim*Nx, Tsim*Nu);
                MatrixXf Ro(Tsim*Nx, Nu);

                for (int i = 0; i < Tsim; i++)
                {
                        MatrixXf temp3 = MatrixXf::Identity(Nx, Nx);
                        for (int j = 0; j < i; j++)
                        {
                                temp3 = a*temp3;
                        }

                        Ro.block(i*Nx, 0, Nx, Nu) = temp3*b;
                }
                /*cout << Ro << endl;*/

                B.block(0, 0, Nx*Tsim, Nu) = Ro;
                for (int k = 1; k < Tsim; k++)

                {
                        MatrixXf temp4 = MatrixXf::Zero(k*Nx, Nu);
                        MatrixXf temp5(Tsim*Nx, Nu);
                        temp5.block(0, 0, k*Nx, Nu) = temp4;
                        temp5.block(k*Nx, 0, (Tsim - k)*Nx, Nu) = Ro.block(0, 0, (Tsim - k)*Nx, Nu);
                        B.block(0, k*Nu, Nx*Tsim, Nu) = temp5;
                }

                //二次优化中的系数矩阵
                MatrixXf H = 2 * (B.transpose()*Q*B + R);

                MatrixXf g = 2 * B.transpose()*Q*A*x_piao;


                //转换为QPoases默认的类型

                const auto NumOfMatrixH = H.rows()*H.cols();
                double *h_matrix = new double[NumOfMatrixH];
                const auto NumOfMatrixg = g.cols()*g.rows();
                double*g_matrix = new double[NumOfMatrixg];
                int index = 0;
                for (int i = 0; i < H.rows(); i++)
                {
                        for (int j = 0; j < H.cols(); j++)
                        {
                                h_matrix[index] = H(i, j);
                                index = index + 1;
                        }
                }

                for (int i = 0; i < g.rows(); i++)
                {

                        g_matrix[i] = g(i, 0);

                }
                //up and low bound constrain
                MatrixXf lb_matrix(Tsim, Nu) ;
                MatrixXf ub_matrix(Tsim, Nu);
                Matrix2f lowlimit;
                lowlimit << umin1, 0,
                        0, umin2;
                Matrix2f uplimit;
                uplimit << umax1, 0,
                        0, umax2;
                lb_matrix = MatrixXf::Ones(Tsim, Nu)*lowlimit;
                ub_matrix = MatrixXf::Ones(Tsim, Nu)*uplimit;
                const auto NumOflbMatrix = lb_matrix.rows()*lb_matrix.cols();
                double *lb= new double[NumOflbMatrix];
                const auto NumOfubMatrix =ub_matrix.cols()*ub_matrix.rows();
                double*ub = new double[NumOfubMatrix];
                int index1 = 0;
                for (int i = 0; i < lb_matrix.rows(); i++)
                {
                        for (int j = 0; j < lb_matrix.cols(); j++)
                        {
                                lb[index1] = lb_matrix(i, j);
                                index1 = index1 + 1;
                        }
                }
                int index2= 0;
                for (int i = 0; i < ub_matrix.rows(); i++)
                {
                        for (int j = 0; j < ub_matrix.cols(); j++)
                        {
                                ub[index2] = ub_matrix(i, j);
                                index2= index2 + 1;
                        }
                }
    // End up and low bound constrain

                int nWSR = 100;
                //求解最优解
                QProblemB solveQP(Tsim*Nu);
                Options options;
                options.printLevel = PL_NONE;
                solveQP.setOptions(options);
                returnValue m=solveQP.init(h_matrix, g_matrix, lb, ub, nWSR);
                cout << "初始化结果:" << m << endl;
                real_t *xOpt;
                xOpt = new real_t[Tsim*Nu];
                solveQP.getPrimalSolution(xOpt);

                //更新状态

             //   real_tao(0) = tao(iter_num - 1, 0) + xOpt[0];//需要传递给机器人系统，作为下一时刻的系统输入
               // real_tao(1) = tao(iter_num - 1, 1) + xOpt[1];
                  real_tao(0) =  xOpt[0];//需要传递给机器人系统，作为下一时刻的系统输入
                  real_tao(1) =  xOpt[1];
//                cout << "tao" << tao(iter_num - 1, 0)<<endl;
//                cout << "u1:" << real_tao(0) << endl;
//                cout << "u2:" << real_tao(1) << endl;
                delete[] h_matrix;
                delete[] g_matrix;
                delete[] xOpt;
                delete[] lb;
                delete[] ub;

                return real_tao;
        }
        else
        {
                cout << "iteration ending" << endl;
                Vector2f real_tao = Vector2f::Zero();
                return real_tao;
        }
}
   VectorXf MPC_trajectoryTracking::getstate()
   {
      return x_real_last;
   }
  void MPC_trajectoryTracking::setstate(VectorXf x_real_current)
    {
        x_real_last=x_real_current;
        }
/********************************************************************************/

MPC_trajectoryTracking2::MPC_trajectoryTracking2(int vNum, int vNp,int vNc, int vNx, int vNu)
{
	M << 283.6, 0,
		0, 29.0;

	D11 << 26.9, 0,
		0, 3.5;
	D12 << 241.3, 0,
		0, 76.9;
	Num = vNum;
	iter_num = 0;
	Np = vNp;
	Nc = vNc;
	Nx = vNx;
	Nu = vNu;
	R = 5* MatrixXf::Identity(Nu*Nc, Nu*Nc);
	Q = 5 * MatrixXf::Identity(Nx*Np, Nx*Np);
	U = Vector2f::Zero();
	U_min << -3.02, -3;
	U_max << 3.02, 3.02;
	yaw_max = 180;
	yaw_min = -180;

	//cout << Q;
}
//line trajectory
MatrixXf MPC_trajectoryTracking2::Generate_Reftrajectory()
{
	MatrixXf ref = MatrixXf::Zero(Num, 5);
	float t;
	for (int i = 0; i < Num; i++)
	{
		t = (i + 1)*0.05;// 0.05ÊÇ»úÆ÷ÈËÏµÍ³µÄ²ÉÑùÊ±Œä
		ref(i, 0) = 5 * t; //xÖáÉÏµÄÎ»ÖÃ
		ref(i, 1) = 5 * t;  //yÖáÉÏµÄÎ»ÖÃ
		ref(i, 2) = 45;     //yawœÇ
		ref(i, 3) = sqrt(50);// ÏßËÙ¶È
		ref(i, 4) = 0;       //œÇËÙ¶È
	}
	return ref;

}
MatrixXf MPC_trajectoryTracking2::Generate_RefTao(MatrixXf ref)
{
	//×¢Òâ£¬¶ÔÃ¿Ò»žöMatrixXfž³ÖµÖ®Ç°±ØÐëžø³öŸØÕóµÄŽóÐ¡£¬·ñÔò»á³öŽí

        MatrixXf tao = MatrixXf::Zero(Num, 2);
	Vector2f vd = Vector2f::Zero();
	Matrix2f temp;
	MatrixXf D;
        for (int k = 0; k < Num; k++)
	{
		if (k>0)
		{
			vd = (ref.block(k, 3, 1, 2) - ref.block(k - 1, 3, 1, 2)).transpose() / 0.05;

		}

		temp << ref(k, 3), 0,
			0, ref(k, 4);
		D = D11 + D12*temp;
		tao.block(k, 0, 1, 2) = (D*ref.block(k, 3, 1, 2).transpose() + M*vd).transpose();

	}
	return tao;
}

Vector2f MPC_trajectoryTracking2::Generate_realTao(MatrixXf ref, MatrixXf tao, VectorXf xreal)
{
	update_iter();
	if (iter_num <= Num)
	{
		cout << "µ±Ç°ÔÚµÚ£º" << iter_num << "²Î¿Œµã" << endl;
		VectorXf x_piao;
		VectorXf kesi(Nx + Nu);
		x_piao = xreal - ref.block(iter_num - 1, 0, 1, 5).transpose();
		kesi.head(5) = x_piao;
		kesi.tail(2) = U;
		Vector2f real_tao = Vector2f::Zero();//ÐèÒª·µ»ØµÄÏòÁ¿
		float t_d = ref(iter_num - 1, 2);
		float u_d = ref(iter_num - 1, 3);
		float r_d = ref(iter_num - 1, 4);
		//ÇóœâŸØÕóa
		Matrix2f temp1;
		Matrix3f A11;
		A11 << 0, 0, -u_d*sin(t_d)*0.05,
			0, 0, u_d*cos(t_d)*0.05,
			0, 0, 0;
		MatrixXf A12(3, 2);
		A12 << cos(t_d)*0.05, 0,
			sin(t_d)*0.05, 0,
			0, 1 * 0.05;
		MatrixXf A21 = MatrixXf::Zero(2, 3);
		temp1 << u_d, 0,
			0, r_d;
		MatrixXf A22;
		A22 = -(M.inverse()*D11 + 2 * M.inverse()*D12*temp1)*0.05;
		MatrixXf time_a(Nx, Nx);
		time_a.block(0, 0, 3, 3) = A11;
		time_a.block(0, 3, 3, 2) = A12;
		time_a.block(3, 0, 2, 3) = A21;
		time_a.block(3, 3, 2, 2) = A22;
		MatrixXf a = MatrixXf::Identity(5, 5) + time_a;

		//ÇóœâŸØÕób
		MatrixXf b = MatrixXf::Zero(Nx, Nu);
		b.block(3, 0, 2, 2) = M.inverse()*0.05;


		//ÇóœâŸØÕóA
		MatrixXf A=MatrixXf::Zero(Nu + Nx, Nu + Nx);
		A.block(0, 0, Nx, Nx) = a;
		A.block(0, Nx, Nx, Nu) = b;
		A.block(Nx, Nx, Nu, Nu) = MatrixXf::Identity(Nu, Nu);

		//ÇóœâŸØÕóB
		MatrixXf B = MatrixXf::Zero(Nx + Nu, Nu);
		B.block(0, 0, Nx, Nu) = b;
		//ŸØÕóC
		MatrixXf C = MatrixXf::Identity(Nx, Nx + Nu);
		//ŸØÕóCconÊÇÎªÁËŒÆËãÔŒÊøµÄÊ±ºòÓÃµÄ
		MatrixXf Ccon(1, Nx + Nu);
		Ccon << 0, 0, 1, 0, 0, 0, 0;

		//ÇóœâŸØÕóPHI
		MatrixXf PHI(Nx*Np, Nx+Nu);
		MatrixXf PHIcon(Np, Nx + Nu);// ŒÆËãÔŒÊøµÄÊ±ºòÓÃ
		MatrixXf temp = MatrixXf::Identity(Nx+Nu, Nx+Nu);
		for (int i = 0; i < Np; i++)
		{
			for (int j = 0; j < i+1; j++)
			{
				temp = A*temp;

			}
			PHI.block(i*Nx,0,Nx, Nx+Nu) = C*temp;
			PHIcon.block(i, 0, 1, Nx + Nu) = Ccon*temp;
		}


		//ÇóœâŸØÕóTHETA
		MatrixXf THETA = MatrixXf::Zero(Np*Nx, Nc*Nu);
		MatrixXf THETAcon = MatrixXf::Zero(Np, Nc*Nu);//ÓÃÓÚŒÆËãÔŒÊø
		MatrixXf Ro(Np*Nx, Nu);
		MatrixXf Rocon(Np, Nu);
		MatrixXf temp3 = MatrixXf::Identity(Nx+Nu,Nx+Nu);
		for (int k = 0; k < Np; k++)
		{
			for (int j = 0; j < k; j++)
			{
				temp3 = A*temp3;

			}
			Ro.block(k*Nx, 0, Nx, Nu) = C*temp3*B;
			Rocon.block(k, 0, 1, Nu) = Ccon*temp3*B;
		}
		THETA.block(0, 0, Np*Nx, Nu) = Ro;
		THETAcon.block(0, 0, Np, Nu) = Rocon;
		for (int i = 1; i < Nc; i++)
		{
			MatrixXf temp4 = MatrixXf::Zero(i*Nx, Nu);
			MatrixXf temp5(Np*Nx, Nu);
			temp5.block(0, 0, i*Nx, Nu) = temp4;
			temp5.block(i*Nx, 0, (Np - i)*Nx, Nu) = Ro.block(0, 0, (Np - i)*Nx, Nu);
			THETA.block(0, i*Nu, Np*Nx, Nu) = temp5;

			MatrixXf temp4con = MatrixXf::Zero(i, Nu);
			MatrixXf temp5con(Np, Nu);
			temp5con.block(0, 0, i, Nu) = temp4con;
			temp5con.block(i, 0, (Np - i), Nu) = Rocon.block(0, 0, (Np - i), Nu);
			THETAcon.block(0, i*Nu, Np, Nu) = temp5con;
		}

		//¶þŽÎÓÅ»¯ÖÐµÄŸØÕó£š²»º¬ËÉ³ÚÒò×Ó£©
		MatrixXf H = THETA.transpose()*Q*THETA + R;
		MatrixXf error = PHI*kesi;
		MatrixXf g = 2 * error.transpose()*Q*THETA;

		//ÔŒÊøÉú³É
        //²»µÈÊœÔŒÊøÌõŒþ
		MatrixXf Aueq1(Nc*Nu, Nc*Nu);
		MatrixXf Aueq2 = THETAcon;
		MatrixXf Ro2(Nc*Nu, Nu);
		for (int i = 0; i < Nc; i++)
		{
			Ro2.block(i*Nu, 0, Nu, Nu) = MatrixXf::Identity(Nu,Nu);

		}
		
		Aueq1.block(0, 0, Nc*Nu, Nu) = Ro2;
		for (int i = 1; i < Nc; i++)
		{
			Aueq1.block(0, i*Nu, i*Nu, Nu) = MatrixXf::Zero(i*Nu, Nu);
			Aueq1.block(i*Nu, i*Nu, (Nc-i)*Nu, Nu) = Ro2.block(0,0,(Nc-i)*Nu,Nu);
		}
		MatrixXf Aueq(Nc*Nu + Np, Nc*Nu);
		Aueq.block(0,0,Nc*Nu, Nc*Nu) = Aueq1;
		Aueq.block(Nc*Nu,0,Np,Nc*Nu) = Aueq2;

		
		MatrixXf Umin = Ro2*U_min;
		MatrixXf Umax = Ro2*U_max;
		MatrixXf Unow = Ro2*U;
		MatrixXf Lbueq1 = Umin - Unow;
		MatrixXf Ubueq1 = Umax - Unow;
		MatrixXf coeff= MatrixXf::Ones(Np, 1);
		MatrixXf Lbueq2 = yaw_min*coeff - PHIcon*kesi;
		MatrixXf Ubueq2 = yaw_max*coeff - PHIcon*kesi;
		MatrixXf Lbueq(Nc*Nu + Np, 1);
		Lbueq.block(0, 0, Nc*Nu, 1) = Lbueq1;
		Lbueq.block(Nc*Nu, 0, Np, 1) = Lbueq2;
		MatrixXf Ubueq(Nc*Nu + Np, 1);
		Ubueq.block(0, 0, Nc*Nu, 1) = Ubueq1;
		Ubueq.block(Nc*Nu, 0, Np, 1) = Ubueq2;

	//	//×ª»»ÎªQPoasesÄ¬ÈÏµÄÀàÐÍ
		const auto NumOfMatrixAueq1 = Aueq1.rows()*Aueq1.cols();
		double *A_matrix;
		A_matrix = new double[NumOfMatrixAueq1];
		int index1 = 0;
		for (int i = 0; i < Aueq1.rows(); i++)
		{
			for (int j = 0; j < Aueq1.cols(); j++)
			{
				A_matrix[index1] = Aueq1(i, j);
				index1 = index1 + 1;
			}

		}
		const auto NumOfMatrixUbueq1 = Ubueq1.cols()*Ubueq1.rows();
		double *ubA;
		ubA = new double[NumOfMatrixUbueq1];
		for (int i = 0; i < Ubueq1.rows(); i++)
		{

			ubA[i] = Ubueq1(i, 0);

		}
		const auto NumOfMatrixLbueq1 = Lbueq1.cols()*Lbueq1.rows();
		double *lbA;
		lbA = new double[NumOfMatrixLbueq1];
		for (int i = 0; i < Lbueq1.rows(); i++)
		{

			lbA[i] = Lbueq1(i, 0);

		}



		const auto NumOfMatrixAueq = Aueq.rows()*Aueq.cols();
		double *Aueq_matrix;
		Aueq_matrix = new double[NumOfMatrixAueq];
		int index2= 0;
		for (int i = 0; i < Aueq.rows(); i++)
		{
			for (int j = 0; j < Aueq.cols(); j++)
			{
				Aueq_matrix[index2] = Aueq(i, j);
				index2= index2 + 1;
			}

		}

		const auto NumOfMatrixUbueq= Ubueq.cols()*Ubueq.rows();
		double *Ubueq_matrix;
		Ubueq_matrix = new double[NumOfMatrixUbueq];
		for (int i = 0; i < Ubueq.rows(); i++)
		{

			Ubueq_matrix[i] = Ubueq(i, 0);

		}
		const auto NumOfMatrixLbueq = Lbueq.cols()*Lbueq.rows();
		double *Lbueq_matrix;
		Lbueq_matrix = new double[NumOfMatrixLbueq];
		for (int i = 0; i < Lbueq.rows(); i++)
		{

			Lbueq_matrix[i] = Lbueq(i, 0);

		}






		const auto NumOfMatrixH = H.rows()*H.cols();
		double *h_matrix = new double[NumOfMatrixH];
		const auto NumOfMatrixg = g.cols()*g.rows();
		double*g_matrix = new double[NumOfMatrixg];
		int index = 0;
		for (int i = 0; i < H.rows(); i++)
		{
			for (int j = 0; j < H.cols(); j++)
			{
				h_matrix[index] = H(i, j);
				index = index + 1;
			}
		}

		for (int i = 0; i < g.rows(); i++)
		{

			g_matrix[i] = g(i, 0);

		}

		real_t *lb= NULL; //¿ØÖÆÊäÈëµÄÏÂÏÞ
		real_t *ub= NULL;//¿ØÖÆÊäÈëµÄÉÏÏÞ
		int nWSR = 5;
	//Çóœâ×îÓÅœâ
	   QProblem solveQP(Nc*Nu,1);
		Options options;
		options.printLevel = PL_NONE;
		solveQP.setOptions(options);
		solveQP.init(h_matrix, g_matrix, A_matrix, lb, ub, lbA, ubA, nWSR);//Ö»¿ŒÂÇ¿ØÖÆÊäÈëµÄÏÞÖÆ
		//solveQP.init(h_matrix, g_matrix, Aueq_matrix, lb, ub, Lbueq_matrix, Ubueq_matrix, nWSR);//¿ŒÂÇ¿ØÖÆÊäÈëµÄÏÞÖÆºÍÊä³öµÄÏÞÖÆ£¬ÆäÖÐÊä³öÖ»¿ŒÂÇÆ«ºœœÇµÄÏÞÖÆ
		real_t *xOpt;
		xOpt = new real_t[Nc*Nu];
		solveQP.getPrimalSolution(xOpt);

	   //žüÐÂ×ŽÌ¬
		U(0) = U(0) + xOpt[0];//ÓÃÓÚÏÂÒ»ŽÎÑ­»·
		U(1) = U(1) + xOpt[1];

	    real_tao(0) = tao(iter_num - 1, 0) + U(0);//ÐèÒªŽ«µÝžø»úÆ÷ÈËÏµÍ³£¬×÷ÎªÏÂÒ»Ê±¿ÌµÄÏµÍ³ÊäÈë
	    real_tao(1) = tao(iter_num - 1, 1) + U(1);


		delete[] h_matrix;
		delete[] g_matrix;
		delete[] xOpt;
		delete[] A_matrix;
		delete[] ubA;
		delete[] lbA;


		return real_tao;
	}
	else
        {
            cout << "iteration ending" << endl;
            Vector2f real_tao = Vector2f::Zero();
            return real_tao;
	}
}
/*******************************************************************************************/

MPC_trajectoryTrackingyaw::MPC_trajectoryTrackingyaw(int vNum, int vTsium, int vNx, int vNu)
{
	umin = -3;//控制输入的下限 0.25 *0.1/0.028325=0.88
	umax = 3;
	M = 0.0021;
	D11 = 0.1225;
	D12 = -0.00032702;
	Num = vNum;
	iter_num = 0;
	Tsim = vTsium;
	Nx = vNx;
	Nu = vNu;
	R = 0.1* MatrixXf::Identity(Nu*Tsim, Nu*Tsim);
	q = MatrixXf::Identity(Nx, Nx);
	q(1, 1) = 0.5;
	Q = setQ(q, Tsim);
	x_real_last = VectorXf::Zero(4); // robot init state
	//cout << Q;
}

VectorXf MPC_trajectoryTrackingyaw::Generate_RefTao(MatrixXf ref)
{
	//注意，对每一个MatrixXf赋值之前必须给出矩阵的大小，否则会出错
	VectorXf tao = VectorXf::Zero(Num);
	float vd = 0;
	float D;
	for (int k = 0; k < Num; k++)
	{
		if (k>1)
		{
			vd = (ref(k, 3) - ref(k - 1, 3)) / 0.05;

		}


		D = D11 + D12*ref(k, 3);
		tao(k) = (D*ref(k, 3) + M*vd);

	}
	return tao;
}
float MPC_trajectoryTrackingyaw::Generate_realTao(MatrixXf ref, VectorXf tao, VectorXf xreal,float velocity)
{
    float real_tao=0;
    return real_tao;
//	update_iter();
//	if (iter_num <= Num)
//	{
//		if (iter_num == 1)
//		{
//			xreal(3) = 0;// if it is the first iteration, xreal  is the init of the robot, set the init yawvelocity of the robot is 0
//		}
//                cout << "当前在第"<<iter_num << "参考点:"<<endl;
//		VectorXf x_piao;
//		x_piao = xreal - ref.block(iter_num - 1, 0, 1, 4).transpose();
		
//		printf("Refpositionx:%.4f  Refpositiony:%.4f  Refyaw:%.4f   Refyawvelocity:%.4f \n", ref(iter_num - 1, 0), ref(iter_num - 1, 1), ref(iter_num - 1, 2), ref(iter_num - 1, 3));
//		float real_tao;//需要返回的向量
//                cout<<"终端"<<endl;

//		//////////////////////////////////////////////////////////////////////////////////////
//                if (x_piao(2)>-1&&x_piao(2)<1)
//		{

//                    real_tao=0;
//		}
//		///////////////////////////////////////////////////////////////////////////////////////
		
//		else
//		{
//                    cout<<"差值:"<<endl;
//		float t_d = ref(iter_num - 1, 2);
//		float u_d = velocity;                  // robot velocity in the image plane, adjust according to the experiment/////////////HXH
//		float r_d = ref(iter_num - 1, 3);
//		//求解矩阵a
//		//Matrix2f temp1;
//                cout<<"终端"<<endl;
//		Matrix3f A11;
//		A11 << 0, 0, -u_d*sin(t_d)*0.05,
//			0, 0, u_d*cos(t_d)*0.05,
//			0, 0, 0;
//		MatrixXf A12(3, 1);
//		A12 << 0,
//			0,
//			1 * 0.05;
//		MatrixXf A21 = MatrixXf::Zero(1, 3);

//		float A22;
//		A22 = -(D11 / M + 2 * D12*r_d / M)*0.05;
//		MatrixXf time_a(Nx, Nx);
//		time_a.block(0, 0, 3, 3) = A11;
//		time_a.block(0, 3, 3, 1) = A12;
//		time_a.block(3, 0, 1, 3) = A21;
//		time_a(3, 3) = A22;
		
//		MatrixXf a = MatrixXf::Identity(4, 4) + time_a;
//		//求解矩阵b
//		MatrixXf b = MatrixXf::Zero(Nx, Nu);
//		b(3, 0) = 0.05 / M;
//		//求解矩阵A
//		MatrixXf A(Tsim*Nx, Nx);
		
//		for (int k = 0; k < Tsim; k++)
//		{
//			MatrixXf temp2 = MatrixXf::Identity(Nx, Nx);///error is here
//			for (int j = 0; j < k + 1; j++)
//			{
//				temp2 = temp2*a;
//			}
//			A.block(k*Nx, 0, Nx, Nx) = temp2;

//		}
		
//		//求解矩阵
//		MatrixXf B = MatrixXf::Zero(Tsim*Nx, Tsim*Nu);
//		MatrixXf Ro(Tsim*Nx, Nu);
		
//		for (int i = 0; i < Tsim; i++)
//		{
//			MatrixXf temp3 = MatrixXf::Identity(Nx, Nx);//error is here
//			for (int j = 0; j < i; j++)
//			{
//				temp3 = a*temp3;
//			}

//			Ro.block(i*Nx, 0, Nx, Nu) = temp3*b;
//		}
		

//		B.block(0, 0, Nx*Tsim, Nu) = Ro;
//		for (int k = 1; k < Tsim; k++)

//		{
//			MatrixXf temp4 = MatrixXf::Zero(k*Nx, Nu);
//			MatrixXf temp5(Tsim*Nx, Nu);
//			temp5.block(0, 0, k*Nx, Nu) = temp4;
//			temp5.block(k*Nx, 0, (Tsim - k)*Nx, Nu) = Ro.block(0, 0, (Tsim - k)*Nx, Nu);
//			B.block(0, k*Nu, Nx*Tsim, Nu) = temp5;
//		}
//                cout<<"终端"<<endl;
//		//二次优化中的系数矩阵
//		MatrixXf H = 2 * (B.transpose()*Q*B + R);
//		MatrixXf g = 2 * B.transpose()*Q*A*x_piao;
//		//转换为QPoases默认的类型
//		const auto NumOfMatrixH = H.rows()*H.cols();
//		double *h_matrix = new double[NumOfMatrixH];
//		const auto NumOfMatrixg = g.cols()*g.rows();
//		double*g_matrix = new double[NumOfMatrixg];
//		int index = 0;
//		for (int i = 0; i < H.rows(); i++)
//		{
//			for (int j = 0; j < H.cols(); j++)
//			{
				
//				h_matrix[index] = H(i, j);
//				index = index + 1;
				
				
//			}
//		}

//		for (int i = 0; i < g.rows(); i++)
//		{
			
//			g_matrix[i] = g(i, 0);
//			cout << g_matrix[i] << endl;

//		}
//		MatrixXf lb_matrix = umin*MatrixXf::Ones(Tsim, Nu);
//		MatrixXf ub_matrix = umax*MatrixXf::Ones(Tsim, Nu);
//		const auto NumOflbmatrix = lb_matrix.rows()*lb_matrix.cols();
//		double *lb = new double[NumOflbmatrix];
//		int index1 = 0;
//		for (int  i = 0; i < lb_matrix.rows(); i++)
//			{
//			for (int j = 0; j < lb_matrix.cols(); j++)
//			{
//				lb[index1] = lb_matrix(i, j);
//				cout << lb[index1] << endl;
//				index1 = index1 + 1;
//			}
//			}
//		const auto NumOfubmatrix = ub_matrix.rows()*ub_matrix.cols();
//		double *ub = new double[NumOfubmatrix];
//		int index2 = 0;
//		for (int i = 0; i < ub_matrix.rows(); i++)
//		{
//			for (int j = 0; j < ub_matrix.cols(); j++)
//			{
//				ub[index2] = ub_matrix(i, j);
//				cout << ub[index2] << endl;
//				index2 = index2 + 1;
//			}
//		}
//		int nWSR = 50;
//		//求解最优解
//		QProblemB solveQP(Tsim*Nu);
//		Options options;
//		options.printLevel = PL_NONE;
//		solveQP.setOptions(options);
		
//		returnValue m=solveQP.init(h_matrix, g_matrix, lb, ub, nWSR);
//		cout << "初始化状态:" << m << endl;
		
//		real_t *xOpt;
//		xOpt = new real_t[Tsim*Nu];
//		solveQP.getPrimalSolution(xOpt);

//		//更新状态

//		real_tao = tao(iter_num - 1) + xOpt[0];//需要传递给机器人系统，作为下一时刻的系统输入


//		delete[] h_matrix;
//		delete[] g_matrix;
//		delete[] xOpt;
//        delete[] lb;
//		delete[] ub;

//		return real_tao;
//		}
//	}
//	else
//	{
//		cout << "iterate ending" << endl;
//		float real_tao = 0;
//		return real_tao;

//	}
}
VectorXf MPC_trajectoryTrackingyaw::getstate()
{
	return x_real_last;
}
void MPC_trajectoryTrackingyaw::setstate(VectorXf x_real_current)
{
	x_real_last = x_real_current;
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*!
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/




}
