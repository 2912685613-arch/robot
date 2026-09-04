#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "ASRModel.h"
#include <cmath>
using namespace std;
using namespace Eigen;
class GenerateTrajectory
{
public:
        GenerateTrajectory(double samplingtime, double desired_velocity,int num_points);
        //这个产生的路劲是一个4维的向量,主要用于单航向的控制
        MatrixXd GenerateLine_4();
        MatrixXd GenerateSquare_4();
        //这个产生的路径是一个5维的向量
        MatrixXd GenerateLine_5();
        MatrixXd GenerateSquare_5();
        MatrixXd GenerateSine_5();
        MatrixXd GenereteTau(MatrixXd Reftrajectory,DyModel ASRmodel );
    private:
        double T;
        double velocity;
        int Nnum;


};
