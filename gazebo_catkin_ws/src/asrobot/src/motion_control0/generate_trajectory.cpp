#include "../../include/asrobot/generate_trajectory.h"
GenerateTrajectory::GenerateTrajectory(double samplingtime, double desired_velocity, int num_points) : 
T(samplingtime),velocity(desired_velocity),Nnum(num_points)
{}
/*****************the desired yaw should be presented with radian*****************/
/* y=3
/* square
/* sine
/*********************************************************************************/
MatrixXd GenerateTrajectory::GenerateLine_5()
{
        //Vnum 
        //这里边的采样时间尽量与实际系统的采样时间保持一致
        //ref is Nnum rows, 5 column.
        MatrixXd ref = MatrixXd::Zero(Nnum, 5);
        float t;
        for (int i = 0; i < Nnum; i++)
        {
                t = (i + 1) * T;                        
                ref(i, 0) = -3 + velocity * t;          //x
                ref(i, 1) = 3;                          //y
                ref(i, 2) = 0;                          //yaw angle need set according to practical sensor value (rad)
                ref(i, 3) = velocity;                   //velocity
                ref(i, 4) = 0;                          //yaw_velocity
        // cout<<"reference velocity is :"<<velocity<<endl;
        }
        return ref;
}

MatrixXd GenerateTrajectory::GenerateSquare_5() 
{
        MatrixXd ref = MatrixXd::Zero(Nnum, 5);
        int num = Nnum / 4;
        float t;
        for (int i = 0; i < Nnum; i++)
        {
                t = (i + 1) * T;                                        //T =0.05 
                if (i <= num)
                {
                        ref(i, 0) = -3 + velocity * t ;                  //+30
                        ref(i, 1) = 3;     //
                        ref(i, 2) = 0;                              
                        ref(i, 3) = velocity;
                        ref(i, 4) = 0;                            
                }
                if (i > num && i <= 2 * num)
                {
                        ref(i, 0) = - 3 + velocity * num * T ;        
                        ref(i, 1) = 3 - velocity * (t - num * T); 
                        ref(i, 2) = -90*3.1415/180;                            
                        ref(i, 3) = velocity;        
                        ref(i, 4) = 0;                         
                }
                if (i > 2 * num && i <= 3 * num)
                {
                        ref(i, 0) = - 3 + velocity * num * T - velocity * (t - 2 * num * T); 
                        ref(i, 1) = 3 - velocity * (2 * num * T - num * T);               
                        ref(i, 2) = -180*3.1415/180;                                                   
                        ref(i, 3) = velocity;     
                        ref(i, 4) = 0;                                                    
                }
                if (i > 3 * num)
                {
                        ref(i, 0) = - 3 + velocity * num * T - velocity * (3 * num * T - 2 * num * T);        
                        ref(i, 1) = 3 - velocity * (2 * num * T - num * T) + velocity * (t - 3 * num * T); 
                        ref(i, 2) = 90*3.1415/180;                                                                      
                        ref(i, 3) = velocity;  
                        ref(i, 4) = 0;                                                                        
                }
        }
        return ref;
}
MatrixXd GenerateTrajectory::GenerateSine_5()
{
        MatrixXd ref = MatrixXd::Zero(Nnum, 5);
        float t;
        float coeff=0.6;
        for (int i = 0; i < Nnum; i++)
        {
                t = (i + 1) * T;                        
                ref(i, 0) = -5+velocity * t;          
                ref(i, 1) = sin(coeff*velocity * t);                          
                ref(i, 2) = atan2(coeff*velocity*cos(coeff*velocity*t),velocity);    //              
                ref(i, 3) = sqrt( velocity*velocity +  coeff*velocity*cos(coeff*velocity * t) * coeff*velocity*cos(coeff*velocity * t));                   
                ref(i, 4) = ( - coeff*velocity*sin(coeff*velocity*t) ) / ( 1+cos( coeff*velocity*t )*cos( coeff*velocity*t) );                                                                               
        }
        return ref;
}
// generate the reference control input according to the reference state information
MatrixXd GenerateTrajectory::GenereteTau(MatrixXd Reftrajectory, DyModel ASRmodel)
{
        // int num_row=Reftrajectory.rows();
        MatrixXd tao = MatrixXd::Zero(Nnum, 2);
        Vector2d vd = Vector2d::Zero();
        Matrix2d temp;
        Matrix2d D;
        for (int k = 0; k < Nnum; k++)
        {
                if (k > 1)
                {
                        vd = (Reftrajectory.block(k, 3, 1, 2) - Reftrajectory.block(k - 1, 3, 1, 2)).transpose() / T;
                }
                temp << Reftrajectory(k, 3), 0,
                    0, Reftrajectory(k, 4);
                D = ASRmodel.HydroLinMatrix + ASRmodel.HydroNonLinMatrix * temp;
                tao.block(k, 0, 1, 2) = (D * Reftrajectory.block(k, 3, 1, 2).transpose() + ASRmodel.MassMatrix * vd).transpose();
        }
        return tao;
}









// MatrixXd GenerateTrajectory::GenerateLine_4() 
// {
//         //Vnum 
//         MatrixXd ref = MatrixXd::Zero(Nnum, 4);
//         float t;
//         for (int i = 0; i < Nnum; i++)
//         {
//                 t = (i + 1) * T;          // 0.05
//                 ref(i, 0) = velocity * t; //x,50 represent init position of the trajectory
//                 ref(i, 1) = 0;            //y
//                 ref(i, 2) = 0;            //yaw angle need set according to practical sensor value
//                 ref(i, 3) = 0;            // velocity
//         }
//         return ref;
// }

// MatrixXd GenerateTrajectory::GenerateSquare_4() //square
// {
//         //Vnum 
//         MatrixXd ref = MatrixXd::Zero(Nnum, 4);
//         int num = Nnum / 4;
//         float t;
//         for (int i = 0; i < Nnum; i++)
//         {
//                 t = (i + 1) * T; // 0.05�ǻ�����ϵͳ�Ĳ���ʱ��
//                 if (i <= num)
//                 {
//                         ref(i, 0) = velocity * t + 50;              //x+50
//                         ref(i, 1) = 420 - velocity * (t - num * T); //y���ϵ�λ��
//                         ref(i, 2) = 0;                              //yaw angle need  set according to practical sensor value
//                         ref(i, 3) = 0;                              //���ٶ�
//                 }
//                 if (i > num && i <= 2 * num)
//                 {
//                         ref(i, 0) = velocity * num * T + 50;        //x���ϵ�λ��
//                         ref(i, 1) = 420 - velocity * (t - num * T); //y���ϵ�λ��
//                         ref(i, 2) = -90;                            //yaw angle need  set according to practical sensor value
//                         ref(i, 3) = 0;                              //���ٶ�
//                 }
//                 if (i > 2 * num && i <= 3 * num)
//                 {
//                         ref(i, 0) = velocity * num * T + 50 - velocity * (t - 2 * num * T); //x���ϵ�λ��
//                         ref(i, 1) = 420 - velocity * (2 * num * T - num * T);               //y���ϵ�λ��
//                         ref(i, 2) = -180;                                                   //yaw angle need  set according to practical sensor value
//                         ref(i, 3) = 0;                                                      //���ٶ�
//                 }
//                 if (i > 3 * num)
//                 {
//                         ref(i, 0) = velocity * num * T + 50 - velocity * (3 * num * T - 2 * num * T);        //x���ϵ�λ��
//                         ref(i, 1) = 420 - velocity * (2 * num * T - num * T) + velocity * (t - 3 * num * T); //y���ϵ�λ��
//                         ref(i, 2) = 90;                                                                      //yaw angle need  set according to practical sensor value
//                         ref(i, 3) = 0;                                                                       //���ٶ�
//                 }
//         }
//         return ref;
// }
