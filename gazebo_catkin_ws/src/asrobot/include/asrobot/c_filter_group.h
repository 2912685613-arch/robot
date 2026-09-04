#ifndef C_FILTER_GROUP_H
#define C_FILTER_GROUP_H

#include "iostream"
#include "math.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>

using namespace std;
using namespace Eigen;

namespace FilterGroup {

/*!
    * \brief  limiting filter 限幅滤波 对抗脉冲扰动
    */

class C_LimitFilter
{
public:
    //parameters of limiting filter
    float last_result;                  // last result of input
    float cur_result;                   // the current input
    float deriv_threshold;              // a threshold of the derivation
                                        // between last result and current result

public:
    //the more details of these function are in .cpp file
    C_LimitFilter();
    C_LimitFilter(float _deriv_threshold, float _last_result);
    void SetFilterOrgLastResult(float _last_result);
    void SetFilterThreshold(float _deriv_threshold);
    float CalFilterOutput(float _cur_result);
};


/*!
    * \brief  first order filter 一阶滤波 信号变化不太快时可采用
    */

class C_FirstOrderFilter
{
public:
    //parameters of first order filter
    float last_result;                  // last result after filtering
    float cur_result;                   // the current input
    float opt_result;                   // the optimal result after calculation

    float weight_range;                 // the weight range, for example,if you want to represent
                                        // the weight use percentage, just let this variable be 100
    float input_weight;                 // if the weight_range is 100, the weight value is 0.37
                                        // just let this varieble be 37

public:
    //the more details of these function are in .cpp file
    C_FirstOrderFilter();
    C_FirstOrderFilter(float _input_weight, float _weight_range, float _last_result);
    void SetFilterOrgLastResult(float _last_result);
    void setFilterWeightValue(float _input_weight, float _weight_range);
    float CalFilterOutput(float _last_result, float _cur_result);
};


/*!
    * \brief  Kalman filter 原版卡尔曼，效果好，但输入的两个量需有积分关系
    *
    * 只能输入两个量，如陀螺仪加速度计测的角度 和 角速度计测的角速度
    */

class C_SecondOrderKalmanFilter
{
public:
    //the variables which kalman filter need
    float last_optimal_angle;           // the last optimal angle (k-1|k-1)
    float last_optimal_angle_rate_bias; // the last optimal bias of angle rate (k-1|k-1)

    float cur_estimate_angle;           // the estimation angle (k|k-1)
    float cur_estimate_angle_rate_bias; // the estimation bias of angle rate (k|k-1)

    float cur_measure_angle;            // the measurement value of angle   (k)
    float cur_measure_angle_rate;       // the measurement value of angle rate  (k)

    float cur_optimal_angle;            // the current optimal angle (k|k)
    float cur_optimal_angle_rate_bias;  // the current optimal bias of angle rate(k|k)

    float last_optimal_cov_matrix[2][2];// the last optimal covariance matrix (k-1|k-1)
    float cur_estimate_cov_matrix[2][2];// the estimation covariance matrix (k|k-1)
    float cur_optimal_cov_matrix[2][2]; // the optimal covariance matrix (k|k-1)
    float cov_noise_matrix[2][2];       // the noise covariance matrix of estimation (k)
    float measurement_noise;            // the noise of measurement (k)

    float angle_Kg;                     // the Kalman gain of angle
    float angle_rate_bias_Kg;           // the Kalman gain of bias

    float sample_time;                  // the sample time of filter
public:
    //the more details of these function are in .cpp file
    C_SecondOrderKalmanFilter();
    float CalFilterOutput(float _cur_measure_angle_rate,
                          float _cur_measure_angle);

};// the brace of class definition

/*!
    * \brief  Kalman filter v2
    *
    * 根据卡尔曼思想 设计的改进版滤波 输入的量不需要有积分关系
    *
    * 允许多个位置式传感器进行滤波
    */

class C_KalmanFilter_v2
{
public:
    int sample_times;                // the number of sample data
    int sensor_num;                  // the number of sensor

    MatrixXf sensor_data_matrix;     // a sensor_num rows , sample_times cols matrix

    VectorXf cur_sample_data;
    VectorXf sample_mean;
    VectorXf sample_var;
    float sensor_var_inv_sum;

    VectorXf sensor_weight;

    VectorXf sensor_initial_value;

    float sensor_optimal_value;

    // function declaration
    C_KalmanFilter_v2();
    void setSampleTimesAndSensorNum( int _sample_times, int _sensor_num);
    void setSensorInitValue ( VectorXf _init_value );
    float KalmanFilter_v2_Output(VectorXf _cur_sample_data);

};// the brace of class C_KalmanFilter_v2


/*!
    * \brief  Kalman filter white noise
    *
    * 利用卡尔曼思想 对信号进行白噪声滤波
    *
    * 只能输入一个位置式传感器的值
    */

class C_KalmanFilter_WhiteNoise
{
public:
    float cur_sample_input;   // input value from sensors ( y(k) )
    float last_opt_value;     // x(k-1 | k-1)
    float cur_est_value;      // x(k | k-1)
    float cur_opt_value;      // x(k | k)

    float mat_P;              // cov matrix P  covariance of process
    float mat_Q;              // cov matrix Q  covariance of control
    float mat_R;              // cov matrix R  covariance of sensors

    float K_g;                 // kalman gain


    float noise_ratio;   // noise between gauss white noise and process estimator
    float filter_output; // output the result

    C_KalmanFilter_WhiteNoise();
    void setNoiseRatio(int _noise_ratio);
    float generate_gauss_noise(float _mu, float _sigma);
    float CalFilterOutput(float _sample_input);

};






}



#endif // C_FILTER_GROUP_H
