#include "../../include/asrobot/c_filter_group.h"

namespace FilterGroup {

/*!
    * \brief  limiting filter 限幅滤波具体实现
    */


/*@
 * Introduction: instanciation of class LimitFilter
 *
 * Input: none
 *
 * Output: none
 */
C_LimitFilter::C_LimitFilter()
{
   deriv_threshold = 2;//默认的门限值为2
   cout<< "the default threshold was set, the value is: "
       << deriv_threshold << endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*@
 * Introduction: instanciation of class LimitFilter
 *
 * Input:
 *   _deriv_threshold: the derivation between last value and current value
 *   _last_result    : the origin last result
 *
 * Output: none
 *
 * Attention : You must set the intial value into last result !!!
 */
C_LimitFilter::C_LimitFilter(float _deriv_threshold, float _last_result)
{
    deriv_threshold = _deriv_threshold;
    last_result = _last_result;

    cout<< "the threshold value was set to: "
        << deriv_threshold << endl;

    cout<< "the original last result value was set to: "
        << last_result << endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*@
 * Introduction: Set Original value to this filter
 *
 * Input:
 *   _last_result    : the origin last result
 *
 * Output: none
 *
 * Attention : In using, you must set the intial value into last result !!!
 */
void C_LimitFilter::SetFilterOrgLastResult(float _last_result)
{
    last_result = _last_result;

    cout<< "the original last result value was set to: "
        << last_result << endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*@
 * Introduction: Set derivation threshold between last value and current value
 *
 * Input:
 *   _deriv_threshold    : derivation threshold between last value and current value
 *
 * Output: none
 *
 */
void C_LimitFilter::SetFilterThreshold(float _deriv_threshold)
{
    deriv_threshold = _deriv_threshold;

    cout<< "the threshold value was set to: "
        << deriv_threshold << endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*@
 * Introduction: calculate output of this filter
 *
 * Input:
 *   _cur_result    : the current sample value
 *
 * Output:
 *   the output value of this filter
 */
float C_LimitFilter::CalFilterOutput(float _cur_result)
{
    cur_result = _cur_result;//取回当前的结果

    //与老结果做比较
    if(abs(cur_result - last_result) >= deriv_threshold)
    {
        cout<< "this sample value(old value) is:"<< last_result<<endl;
        return last_result;
    }
    else
    {
        cout<< "this sample value(new value) is:"<< cur_result<<endl;
        last_result = cur_result;//新值给传给老值
        return cur_result;
    }
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*!
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/


/*!
    * \brief  first order filter 一阶滤波
    */

/*@
 * Introduction: the instanciation of class First Order Filter
 *
 * Input: none
 *
 * Output: none
 */
C_FirstOrderFilter::C_FirstOrderFilter()
{
    input_weight = 50;//默认输入的权重值为100
    cout<< "the default weight value is: "
        << input_weight << endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*@
 * Introduction: the instanciation of class First Order Filter
 *
 * Input:
 *   _input_weight: it represents the weight of last value,
 *                  if this value is smaller, it means current value
 *                  is more important
 *   _weight_range: it represent the range of weight value
 *   _last_result : it represent the intial sample value
 *
 * Output: none
 *
 * Attention: recommand to set a nonzero value to _last_result
 */
C_FirstOrderFilter::C_FirstOrderFilter(float _input_weight, float _weight_range, float _last_result)
{
    input_weight = _input_weight;
    weight_range = _weight_range;
    last_result = _last_result;

    cout<< "the original last result was set to: "
        << last_result << endl;

    if(weight_range < 0 )
    {
        weight_range = 100;
        cout<< "the weight range must be larger than 0"<<endl;
        cout<< "the weight range was automatically set to 100"<< endl;
    }
    else
    {
        cout<< "the weight range was set to: "<< weight_range<< endl;
    }


    if(input_weight > weight_range)
    {
        input_weight = weight_range;
        cout<< "the input weight is larger than weight range"<<endl;
        cout<< "it has been automatically set to the weight range "<<endl;
        cout<< "the input weight was set to: "
            << input_weight << endl;
    }
    else if(input_weight < 0.0001)
    {
        input_weight = 0.01;
        cout<< "the input weight must be larger than 0"<<endl;
        cout<< "it has been automatically set to 0.01"<<endl;
        cout<< "the input weight was set to: "
            << input_weight << endl;
    }
    else
    {
        cout<< "the input weight was set to: "<< input_weight<< endl;
    }

}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*@
 * Introduction: set the initial value to this filter
 *
 * Input:
 *   _last_result : it represent the intial sample value
 *
 * Output: none
 *
 * Attention: recommand to set a nonzero value to _last_result
 */
void C_FirstOrderFilter::SetFilterOrgLastResult(float _last_result)
{
    last_result = _last_result;
    cout<< "the original last result was set to: "
        << last_result << endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*@
 * Introduction: set the input weight and weight range
 *
 * Input:
 *   _input_weight : it represent the weight of last value
 *                   if this value is smaller, it means current value
 *                   is more important
 *   _weight_range : it represent the range of weight value
 *
 * Output: none
 *
 * Attention: recommand to set percentage weight, for eample, _input_weight = 36
 *            _weight_range = 100, it means the weight of last value is 0.36
 *            the weight of current value is 0.64
 */
void C_FirstOrderFilter::setFilterWeightValue(float _input_weight, float _weight_range)
{
    input_weight = _input_weight;
    weight_range = _weight_range;

    if(weight_range < 0 )
    {
        weight_range = 100;
        cout<< "the weight range must be larger than 0"<<endl;
        cout<< "the weight range was automatically set to 100"<< endl;
    }
    else
    {
        cout<< "the weight range was set to: "<< weight_range<< endl;
    }


    if(input_weight > weight_range)
    {
        input_weight = weight_range;
        cout<< "the input weight is larger than weight range"<<endl;
        cout<< "it has been automatically set to the weight range "<<endl;
        cout<< "the input weight was set to: "
            << input_weight << endl;
    }
    else if(input_weight < 0.0001)
    {
        input_weight = 0.01;
        cout<< "the input weight must be larger than 0"<<endl;
        cout<< "it has been automatically set to 0.01"<<endl;
        cout<< "the input weight was set to: "
            << input_weight << endl;
    }
    else
    {
        cout<< "the input weight was set to: "<< input_weight<< endl;
    }
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
 * Introduction: calculate the filter output value
 *
 * Input:
 *   _last_result : the last sample value
 *   _cur_result  : the current sample value
 *
 * Output:
 *   the output value of this filter
 *
 */
float C_FirstOrderFilter::CalFilterOutput(float _last_result, float _cur_result)
{
    last_result = _last_result;
    cur_result = _cur_result;

    opt_result = last_result * ( input_weight / weight_range )
            + cur_result * ((weight_range - input_weight)/weight_range);

    cout << "the opt result is: "<< opt_result<<endl;

    return opt_result;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*!
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 */



/*!
    * \brief  kalman filter 二输入卡尔曼滤波具体实现
    */

/*@
 * Introduction: the instanciation of class SecondOrderKalmanFilter
 *
 * Input: None
 *
 * Output: None
 *
 * Attention: Do not modify these variables randomly
 */
C_SecondOrderKalmanFilter::C_SecondOrderKalmanFilter()
{
    // most of them need to be set to 0
    // the introduction of these variables are in .h file
    last_optimal_angle = 0.0;
    last_optimal_angle_rate_bias = 0.0;

    cur_estimate_angle = 0.0;
    cur_estimate_angle_rate_bias = 0.0;

    cur_measure_angle = 0.0;
    cur_measure_angle_rate = 0.0;

    cur_optimal_angle = 0.0;
    cur_optimal_angle_rate_bias = 0.0;

    last_optimal_cov_matrix[0][0] = 1.0;
    last_optimal_cov_matrix[0][1] = 0.0;
    last_optimal_cov_matrix[1][0] = 0.0;
    last_optimal_cov_matrix[1][1] = 1.0;

    cur_estimate_cov_matrix[0][0] = 0.0;
    cur_estimate_cov_matrix[0][1] = 0.0;
    cur_estimate_cov_matrix[1][0] = 0.0;
    cur_estimate_cov_matrix[1][1] = 0.0;

    cur_optimal_cov_matrix[0][0] = 0.0;
    cur_optimal_cov_matrix[0][1] = 0.0;
    cur_optimal_cov_matrix[1][0] = 0.0;
    cur_optimal_cov_matrix[1][1] = 0.0;

    cov_noise_matrix[0][0] = 0.01;
    cov_noise_matrix[0][1] = 0.0;
    cov_noise_matrix[1][0] = 0.0;
    cov_noise_matrix[1][1] = 0.03;

    measurement_noise = 0.5;

    angle_Kg = 0.0;
    angle_rate_bias_Kg = 0.0;

    sample_time = 0.05;
    cout<< "the whole relative parameters was set"<<endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*@
 * Introduction: Calculate the output of Kalman Filter
 *
 * Input:
 *   _cur_measure_angle_rate: the current angle rate measured by sensors
 *   _cur_measure_angle     : the current angle measured by sensors
 *
 * Output:
 *   the output value after filtering
 *
 * Attention: Do not modify these variables randomly
 */
float C_SecondOrderKalmanFilter::CalFilterOutput(float _cur_measure_angle_rate,
                                               float _cur_measure_angle)
{
    cur_measure_angle = _cur_measure_angle;
    cur_measure_angle_rate = _cur_measure_angle_rate;

    //calculate the estimation value of angle (k|k-1)
    cur_estimate_angle = last_optimal_angle
            + (cur_measure_angle_rate - last_optimal_angle_rate_bias)*sample_time;
    //calculate the estimation value of  the .bias. of angle rate (k|k-1)
    cur_estimate_angle_rate_bias = last_optimal_angle_rate_bias;


    //calculate the estimation value of covariance matrix (k|k-1)
    cur_estimate_cov_matrix[0][0] =  last_optimal_cov_matrix[0][0]+
                                   ((-last_optimal_cov_matrix[1][0]
                                     -last_optimal_cov_matrix[0][1]
                                     +cov_noise_matrix[0][0])*sample_time);
    cur_estimate_cov_matrix[0][1] =  last_optimal_cov_matrix[0][1]+
                                   ((-last_optimal_cov_matrix[1][1]
                                     +cov_noise_matrix[0][1])*sample_time);
    cur_estimate_cov_matrix[1][0] =  last_optimal_cov_matrix[1][0]+
                                   ((-last_optimal_cov_matrix[1][1]
                                     +cov_noise_matrix[1][0])*sample_time);
    cur_estimate_cov_matrix[1][1] =  last_optimal_cov_matrix[1][1]+
                                    (cov_noise_matrix[1][1]*sample_time);


    //calculate the Kalman gain, include the Kg of angle and bias
    angle_Kg = cur_estimate_cov_matrix[0][0]/
            (cur_estimate_cov_matrix[0][0] + measurement_noise);
    angle_rate_bias_Kg = cur_estimate_cov_matrix[1][0]/
            (cur_estimate_cov_matrix[0][0] + measurement_noise);


    // calculate the optimal value
    // the optimal angle
    cur_optimal_angle += ((cur_measure_angle - cur_estimate_angle)* angle_Kg);
    // the optimal bias of angle rate
    // besides, we could obtain the optimal angle rate use the following equation
    //(the optimal angle rate) = (measurement value of angle rate) -  (the optimal bias)
    cur_optimal_angle_rate_bias += ((cur_measure_angle - cur_estimate_angle)
                                    * angle_rate_bias_Kg);


    // then calculate the optimal covariance matrix (k|k)
    cur_optimal_cov_matrix[0][0] -= (angle_Kg*cur_estimate_cov_matrix[0][0]);
    cur_optimal_cov_matrix[0][1] -= (angle_Kg*cur_estimate_cov_matrix[0][1]);
    cur_optimal_cov_matrix[1][0] -= (angle_rate_bias_Kg*cur_estimate_cov_matrix[0][0]);
    cur_optimal_cov_matrix[1][1] -= (angle_rate_bias_Kg*cur_estimate_cov_matrix[0][1]);


    // put current optimal value into last optimal value: (k|k)-> (k-1|k-1)
    // In the next iteration, this operation is very necessary
    // the first is angle and bias
    last_optimal_angle = cur_optimal_angle;
    last_optimal_angle_rate_bias = cur_optimal_angle_rate_bias;

    //the second is covariance matrix
    last_optimal_cov_matrix[0][0] = cur_optimal_cov_matrix[0][0];
    last_optimal_cov_matrix[0][1] = cur_optimal_cov_matrix[0][1];
    last_optimal_cov_matrix[1][0] = cur_optimal_cov_matrix[1][0];
    last_optimal_cov_matrix[1][1] = cur_optimal_cov_matrix[1][1];

    cout<< "the optimal angle is: "<< cur_optimal_angle <<endl;
    return cur_optimal_angle;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*!
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/


/*!
    * \brief  kalman filter v2 改进版多输入，位置式滤波实现
    */

/*@
 * Introduction: the instanciation of class KalmanFilter version2
 *
 * Discription: this is a self made filter, the main idea is: calculate the cov
 *              in real time, if the cov is larger, the weight of it would be smaller
 *
 * Input: None
 *
 * Output: None
 *
 * Attention: Do not modify these variables randomly
 */

C_KalmanFilter_v2::C_KalmanFilter_v2()
{
    cout<< "Do not forget to set the [sample_times] and the [sensor num]" <<endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
 * Introduction: Calculate the output of Kalman Filter version 2
 *
 * Input:
 *   _sample_times: the number of sample value to save for each sensor
 *   _sensor_num  : the number of sensors
 *
 * Output: None
 *
 * Attention: Do not modify these variables randomly
 */

void C_KalmanFilter_v2::setSampleTimesAndSensorNum(int _sample_times, int _sensor_num)
{
    sample_times = _sample_times;
    sensor_num = _sensor_num;

    sensor_var_inv_sum = 0;

    sensor_data_matrix = MatrixXf(sensor_num, sample_times);
    sample_mean = VectorXf(sensor_num);
    sample_var = VectorXf(sensor_num);
    sensor_weight = VectorXf(sensor_num);

    sample_mean.setZero(sensor_num);
    sample_var.setZero(sensor_num);  // initial as 0

    cout<< "[sample_times] and [sensor_num] has been set!!!" << endl;
    cout<< "[ sample_times ]: " << sample_times << endl
        << "[ sensor_num ]: "  << sensor_num << endl;
    cout<< "now please set the 'sensor initial value' " << endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*@
 * Introduction: Calculate the output of Kalman Filter version 2
 *
 * Input:
 *   _init_value: a column vector which contains current sample value
 *                the first sensor value is put in index0 , the second put in index1.
 *                and so on
 *
 * Output: None
 *
 * Attention: Do not modify these variables randomly
 */
void C_KalmanFilter_v2::setSensorInitValue(VectorXf _init_value)
{
    sensor_initial_value = _init_value;
    for(int i=0; i< sensor_data_matrix.cols(); i++)
    {
        sensor_data_matrix.block(0, i, sensor_num, 1) = sensor_initial_value;
    }

    cout << "the 'sensor initial value has been set!!! ' "<<endl
         << "you could use the function:  [ KalmanFilter_v2_Output ]  "<<endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*@
 * Introduction: Calculate the output of Kalman Filter version 2
 *
 * Input:
 *   _cur_sample_data: a column vector which contains current sample value
 *                     the first sensor value is put in index0 , the second put in index1.
 *                     and so on
 *
 * Output: the optimal sensor data
 *
 * Attention: Do not modify these variables randomly
 */
float C_KalmanFilter_v2::KalmanFilter_v2_Output(VectorXf _cur_sample_data)
{
    cur_sample_data = _cur_sample_data;
    // the first step: use the block operation to save the current sample value
    sensor_data_matrix.block(0,0, sensor_num, sample_times-1) =
      sensor_data_matrix.block(0,1, sensor_num, sample_times);

    sensor_data_matrix.block(0, sample_times-2,sensor_num,1 ) = cur_sample_data;

    // the second step: calculate the mean and var
    for(int i=0 ; i <sensor_data_matrix.rows();i++ )
    {
        for(int j=0;j<sensor_data_matrix.cols();j++)
        {
            sample_mean(i) += sensor_data_matrix(i,j);
        }
    }
    sample_mean /= sample_times;              // calculate the average

    for(int i=0 ; i <sensor_data_matrix.rows(); i++ )
    {
        for(int j=0;j<sensor_data_matrix.cols();j++)
        {
            sample_var(i) += pow(cur_sample_data(i) - sample_mean(i),2);
        }
    }
    sample_var /= (sample_times - 1);         //calculate the average

    // third step: calculate the weight value
    if(sensor_num == 2)
    {
        sensor_weight(0) = ( sample_var(1) / ( sample_var(0)+sample_var(1) ) );
        sensor_weight(1) = 1 - sensor_weight(0);
    }else
    {
        for(int i=0 ; i< sensor_num ;i++)
        {
           sensor_var_inv_sum += ( 1 / sample_var(i) );
        }

        for(int i=0; i< sensor_num ;i++)
        {
            sensor_weight(i) = ( 1 / sample_var(i) ) / sensor_var_inv_sum;
        }
    }

    // final step: calculate the optimal value of these sensors


    sensor_optimal_value = sensor_weight.transpose() * cur_sample_data;


    // return the final output
    cout << "the sensor [optimal value] is: " << sensor_optimal_value <<endl;
    return sensor_optimal_value;
}
/*!
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/



/*!
    * \brief  kalman filter v2 改进版多输入，位置式滤波实现
    */

/*@
 * Introduction: the instanciation of class KalmanFilter version2
 *
 * Discription: a first order gauss white noise filter
 *
 * Input: None
 *
 * Output: None
 *
 * Attention: Do not modify these variables randomly
 */

C_KalmanFilter_WhiteNoise::C_KalmanFilter_WhiteNoise()
{
    last_opt_value = 0;
    cur_est_value = 0;
    cur_opt_value = 0;
    noise_ratio = 50;              // it means the noise ratio between state and measurement
    mat_P = 1.0/noise_ratio * 1;    // "1" stand for the covariance of normal distribution
    mat_Q = 1.0/noise_ratio * 1;
    mat_R = 1;
    cout << "the [sample_input] [noise ratio] was set a default value"<<endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*@
 * Introduction: the instanciation of class KalmanFilter version2
 *
 * Discription: a first order gauss white noise filter
 *
 * Input:
 *        _noise_ratio: it means the noise ratio between state and measurement
 *
 * Output: None
 *
 * Attention: Do not modify these variables randomly
 */

void C_KalmanFilter_WhiteNoise::setNoiseRatio(int _noise_ratio)
{
    noise_ratio = _noise_ratio;
    cout << "you have set noise ratio to: "<<noise_ratio <<endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
 * Introduction: the instanciation of class KalmanFilter version2
 *
 * Discription: generate the normal distribution random number
 *
 * Input:
 *        _mu     : the mean of normal distribution
 *        __sigma : the standard variance of normal distribution
 *
 * Output: the random number
 *
 * Attention: Do not modify these variables randomly
 */
float C_KalmanFilter_WhiteNoise::generate_gauss_noise(float _mu, float _sigma)
{
    const double epsilon = numeric_limits<float>::min();
    const double two_pi = 2.0 * 3.14159265358979;

    static double z0, z1;

    double u1, u2;
    do
    {
        u1 = rand()*(1.0/RAND_MAX);
        u2 = rand()*(1.0/RAND_MAX);
    }
    while (u1 < epsilon);

    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);

    return (float) (z1 * _sigma + _mu);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*@
 * Introduction: the instanciation of class KalmanFilter version2
 *
 * Discription: calculate the optimal sensor value
 *
 * Input:
 *    _sample_input: the current sample value of a single sensor
 *
 * Output: the optimal value sansor output
 *
 * Attention: Do not modify these variables randomly
 */
float C_KalmanFilter_WhiteNoise::CalFilterOutput(float _sample_input)
{
    cur_sample_input = _sample_input;

    cur_est_value = last_opt_value ;
    mat_P = mat_P + mat_Q;            // 在卡尔曼滤波器原始表达式中，这是一个矩阵
                                      // 但只估计单状态，故该协方差矩阵其实是 1x1 的
    K_g = mat_P/(mat_P + mat_R);
    cur_opt_value = cur_est_value + K_g * (cur_sample_input - cur_est_value);
    mat_P = (1.0 - K_g)*mat_P;

    last_opt_value = cur_opt_value; // update the optimal value

    filter_output = cur_opt_value;  // for next iteration
    // cout <<"the [sensor optimal value] is: "<<filter_output <<endl;//hcy
    return filter_output;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*!
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/














}
