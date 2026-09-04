#ifndef C_MULTI_AGENT_ALGORITHM_H
#define C_MULTI_AGENT_ALGORITHM_H

#include <iostream>
#include <vector>
#include <math.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>


// #include <cv.h>//hcy
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// 虚拟结构宏定义
#define FORMATION_TYPE_DEFAULT          0
#define TRIANGULAR_FORMATION            0   // "triangle formation"
#define LATERAL_LINE_FORMATION          1   // " 一 "
#define LONGITUDINAL_LINE_FORMATION     2   // " 1 "
#define M_PI                            3.14159265358979323846

// 虚拟连杆宏定义
#define     LEFT_VL         0
#define     RIGHT_VL        1
#define     LINK_NUM_1_1    0
#define     LINK_NUM_1_2    1
#define     LINK_NUM_2_1    2
#define     LINK_NUM_2_2    3



using namespace std;
using namespace Eigen;
using namespace cv;

namespace MultiAgentAlgorithm {

/*!
     * \brief The APF class 人工势场
     */
class APF
{
public:
    APF();
    ~APF();
public:
    float k_rep = 0.1;    //阻力系数
    float r = 0.5;    //机器人半径
    float r0 = 2; //保护距离

    //double n_otheragent=1;//除去此机器人的个数
    int dimension = 3;    //二维或三维坐标
    MatrixXf pv_agent;  //apf计算得到的此机器人偏移向量（一个列向量）

public:
    //修改APF的参数
    void set_krep(float);
    void set_r(float);
    void set_r0(float);

    //返回APF的参数
    float get_krep();
    float get_r();
    float get_r0();

    //计算机器人编队内部阻力，防止编队机器人之间碰撞（第一个输入此机器人的坐标，第二个输入其他机器人的坐标，输出此机器人的偏移）
    VectorXf apf_InterRepulsion(VectorXf,MatrixXf);
    //计算此机器人和障碍物之间的阻力，实现避障（第一个输入此机器人的坐标，第二个输入障碍物的坐标，第三个输入障碍物的半径，输出此机器人的偏移）
    VectorXf apf_ExterRepulsion(VectorXf,MatrixXf,MatrixXf);

};


/*!
     * \brief The VirtualStructure class 虚拟结构
     */
class VirtualStructure
{
public:
    VirtualStructure();
    // n 个机器人， 线长， 三角形边长， 编队方向
    VirtualStructure(int n, float line, float TriSide, float yaw);
    ~VirtualStructure();

public:
    int robotNum;
    Eigen::MatrixXf robotPosMatInVS;
    vector<cv::Point3f> robotPosListInVS;
    vector<cv::Point3f> robotPosListInWorld;
    float lineLength;
    float TriangularSideLength;
    float vs_yaw;
    cv::Point3f vs_pointInWorld;
    cv::Point3f world_point;


public:
    void setFormationParams(int n, float line, float TriSide, float yaw);

    vector<cv::Point3f> calcRobotsPosInTriangularForm();    // equilateral triangle
    vector<cv::Point3f> calcRobotsPosInLatLineForm();
    vector<cv::Point3f> calcRobotsPosInLonLineForm();
    Eigen::MatrixXf  transform2WorldFrame();                // 2D: virtual structure coordinate to world coordinate
};

class referenceTrajectory
{
public:
    referenceTrajectory();
    // n 个机器人， 线长， 三角形边长， 编队方向
    referenceTrajectory(int n, float line, float TriSide,MatrixXf ref_Info);
    ~referenceTrajectory();

private:
    int robotNum;
    int ref_Point_Num;
    int ref_state_Num;
    float lineLength;
    float TriangularSideLength;
public:
    MatrixXf vs_ref_Info_;
    
    MatrixXf virtureRobot_ref_Info_;
    MatrixXf robot1_ref_Info_;
    MatrixXf robot2_ref_Info_;
    MatrixXf robot3_ref_Info_;


public:
    void setFormationParams(int n, float line, float TriSide);
    void update_ref_Info(MatrixXf ref_Info);
    void calcRobotsPosInTriangularForm();    // equilateral triangle
    void calcRobotsPosInLatLineForm();
    void calcRobotsPosInLonLineForm();
    void updata_robot_ref_Info(int formationshape);
};


/*!
    * \brief The VirtualStructure class 虚拟连杆
    */

class VirtualLinkageStructure
{
public:
    VirtualLinkageStructure(Eigen::Vector3f vls_point, float fTheta,
                              float fLeftBeta, float fRightBeta);
    VirtualLinkageStructure(int link_type, Eigen::Vector3f vls_point, float fAlpha,
                    float fLeftBeta, float fRightBeta, float leftLinkage, float rightLinkage);

    VirtualLinkageStructure(int link_type);
    ~VirtualLinkageStructure();

public:
    Eigen::Vector3f vls_pointInWorld;   // position of VLS coordinate in world.
    int nVL_num;                        // virtual linkage number, nVL_num = leftVL_num + rightVL_num.
    int nLinkType;                      // 0:1_1; 1:1_2; 2:2_1; 3:2_2
    float fTheta, fAlpha, fLeftBeta, fRightBeta, fLeftLinkageLength, fRightLinkageLength;
    int leftVL_num, rightVL_num;

    // All Linkage
    vector<float> fAlphaVector;
    vector<float> fBetaVector;
    vector<float> fLinkageLength;
    vector<float> fGamaVector;
    vector<Eigen::Vector3f> vlPosInVLS;

    // 坐标系
    float roll, yaw, pitch;             // Pose relative to the World Frame(WF).
//    Eigen::Matrix3f rotation_matrix;    // rotation matrix relative to the WF
    //    Eigen::MatrixXd
    Eigen::AngleAxisf rotation_vector;
    vector<Eigen::Vector3f> vlPosInWorld;
    Eigen::MatrixXf vlMatPointWorld;
//    vector<Eigen::Vector3f> robotPosInWorld;

public:
    void setVLNum(int nVL_num, int lnum, int rnum);
    void setLinkType(int link_type);
    void InitializeParameters();
    void setVLSAngle(float fAlpha_, float fLeftBeta_, float fRightBeta_);
    void setLinkageLength(float leftLinkage_, float rightLinkage);
    void setVLPointInworld(Eigen::Vector3f vls_pointInWorld_);
    void setVLS_Angle_RYP(float roll, float yaw, float pitch);
    void vectorClear();

    // 规划机器人位置
    void calcVL_1_1_Pos();
    void calcVL_1_2_Pos();
    void calcVL_2_1_Pos();
    void calcVL_2_2_Pos();  //
    Eigen::MatrixXf calcVL_Pos();
    void transform2WorldFrame();    // transform all robots position in world

};





/*!
    * \brief The Centralization Concensus class 集中式一致性
    */

// 集中式一致性
class C_CentrInfoConcencus
{
public:
    // 变量表
    int agent_num;             // 智能体数量
    int dim_num;               // 位置维度 一般取1，2，3

    MatrixXf cur_pos;          //所有点的当前位置 先写x1,x2,...
    //再写y1,y2,...以此类推
    MatrixXf cur_pos_d;        //所有点的影子位置，先写x1_d,x2_d,...
    //再写y1_d, y2_d,...以此类推
    MatrixXf cur_pos_speed;    //所有点的影子，在各个方向上的速度
    //参数的顺序同上
    MatrixXf minus_laplace;    //取相反数的图矩阵

    MatrixXf Identity_dn;      //做克罗内克积需要的单位矩阵，维度为dim x dim

    float gama;                //收敛因子
    int conv_speed;            //收敛速度
    int iterate_times;         //计算一致性循环迭代次数
    MatrixXf dest_pos;         //输出的目标点的位置
    //参数顺序与前几个向量同上
    MatrixXf dest_pos_d;       //输出目标点位置时，当前影子的位置
    //参数顺序与前几个向量同上
    MatrixXf kron_result;      //克罗内克积乘积的结果

public:
    //成员函数 具体功能在实现部分有介绍
    C_CentrInfoConcencus();
    void setAgentNumAndDimNum(int _agent_num, int _dim_num);
    void setConvSpeed(int _conv_speed);

    template<typename Tensor_Input>
    void setMinusLaplace(const Tensor_Input& _minus_laplace );

    template<typename Tensor_Input1, typename Tensor_Input2, typename Tensor_Output>
    void kroneckerProduct_full(const Tensor_Input1& _A, const Tensor_Input2& _B, Tensor_Output & _AB);

    template<typename Tensor_Input1,typename Tensor_Input2,typename Tensor_Input3,
             typename Tensor_Output1, typename Tensor_Output2>
    void CalDestPos(const Tensor_Input1& _cur_pos, const Tensor_Input2& _cur_pos_d,const Tensor_Input3& _cur_pos_speed,
                    Tensor_Output1& _dest_pos,  Tensor_Output2& _dest_pos_d);

    template<typename Tensor_Input1,typename Tensor_Input2,typename Tensor_Input3,
             typename Tensor_Output1, typename Tensor_Output2>
    void GetRobotExpectPosition(int _robot_id, const Tensor_Input1& _cur_pos, const Tensor_Input2& _cur_pos_d, const Tensor_Input3& _cur_pos_speed,
                                Tensor_Output1& _dest_pos,  Tensor_Output2& _dest_pos_d);

};


//集中式一致性函数实现

/*@
     * 函数功能： 设置信号流图拉普拉斯矩阵
     *    输入： 拉普拉斯矩阵 MatrixXf MatrixXd均可
     *    输出： 无
     */
/************************************************************************/
template<typename Tensor_Input>
void C_CentrInfoConcencus::setMinusLaplace(const Tensor_Input& _minus_laplace )
{
    minus_laplace = _minus_laplace;
    cout << "the minus laplace has been set : "<<endl;
    cout << "the minus laplace is : "<<endl;
    cout << minus_laplace <<endl;
}
/************************************************************************/
/************************************************************************/



/*@
     * 函数功能： 采用eigen库块操作计算克罗内克积
     *    输入： 两个矩阵 可以是 Matrix所有型号，也可以是Vector的所有型号
     *    输出： 维数更大的克罗内克矩阵
     */
/************************************************************************/
template<typename Tensor_Input1, typename Tensor_Input2, typename Tensor_Output>
void C_CentrInfoConcencus::kroneckerProduct_full(const Tensor_Input1& _A,const Tensor_Input2& _B, Tensor_Output & _AB)
{
    const unsigned int Ar = _A.rows(),
            Ac = _A.cols(),
            Br = _B.rows(),
            Bc = _B.cols();

    for (unsigned int i=0; i<Ar; ++i)
        for (unsigned int j=0; j<Ac; ++j)
            _AB.block(i*Br,j*Bc,Br,Bc) = _A(i,j)*_B; //利用eigen块操作实现K积
}
/************************************************************************/
/************************************************************************/



/*@
     * 函数功能： 利用一致性算法计算期望位置和影子位置
     *    输入： 1.当前位置向量 2.当前影子向量 3.当前影子速度 （可以是Matrix 也可以是Vector）
     *    输出： 1.期望位置向量 2.期望位置速度  （可以是Matrix 也可以是Vector）
     */
/************************************************************************/
template<typename Tensor_Input1,typename Tensor_Input2,typename Tensor_Input3,
         typename Tensor_Output1, typename Tensor_Output2>
void C_CentrInfoConcencus::CalDestPos(const Tensor_Input1& _cur_pos,const Tensor_Input2& _cur_pos_d,const Tensor_Input3& _cur_pos_speed,
                                      Tensor_Output1& _dest_pos,  Tensor_Output2& _dest_pos_d)
{
    cur_pos = _cur_pos.transpose();
    cur_pos_d = _cur_pos_d.transpose();
    cur_pos_speed = _cur_pos_speed.transpose();


    kroneckerProduct_full(minus_laplace,Identity_dn,kron_result);
    cur_pos = cur_pos + cur_pos_speed + gama * kron_result * (cur_pos - cur_pos_d);
    cur_pos_d = cur_pos_d + cur_pos_speed;


    _dest_pos = cur_pos.transpose();
    _dest_pos_d = cur_pos_d.transpose();

}
/************************************************************************/
/************************************************************************/


/*@ 这个函数暂时先不使用
     * 函数功能： 利用一致性算法 结合机器人的序号计算当前机器人的期望位置和影子位置
     *    输入： 1.机器人序号 2.当前位置向量 3.当前影子向量 4.当前影子速度 （可以是Matrix 也可以是Vector）
     *    输出： 1.期望位置向量 2.期望位置速度  （可以是Matrix 也可以是Vector）
     */
/************************************************************************/
template<typename Tensor_Input1,typename Tensor_Input2,typename Tensor_Input3,
         typename Tensor_Output1, typename Tensor_Output2>
void C_CentrInfoConcencus::GetRobotExpectPosition(int _robot_id, const Tensor_Input1& _cur_pos,const Tensor_Input2& _cur_pos_d,const Tensor_Input3& _cur_pos_speed,
                                                  Tensor_Output1& _dest_pos,  Tensor_Output2& _dest_pos_d)
{
    cur_pos = _cur_pos.transpose();
    cur_pos_d = _cur_pos_d.transpose();
    cur_pos_speed = _cur_pos_speed.transpose();

    kroneckerProduct_full(minus_laplace,Identity_dn,kron_result);

    cur_pos = cur_pos + cur_pos_speed + gama * kron_result * (cur_pos - cur_pos_d);
    cur_pos_d = cur_pos_d + cur_pos_speed;

    //先取转至 再提取对应的列向量
    _dest_pos = cur_pos.transpose().col(_robot_id - 1);
    _dest_pos_d = cur_pos_d.transpose().col(_robot_id - 1);

}
/************************************************************************/
/************************************************************************/





/*!
    * \brief The Distribute Concensus class 分布式一致性
    */

// 分布式一致性
class C_DistrInfoConcencus
{
public:
    // 变量表
    int neighbour_agent_num;            // 伙伴智能体数量
    int dim_num;                        // 位置维度 一般取1，2，3

    int self_id;                        // 机器人自己的id号

    int self_index;                     // 机器人在小组当中的排名

    MatrixXf squad_cur_pos;             // 小组成员的当前位置
    MatrixXf squad_cur_pos_d;           // 小组成员的各自的影子

    MatrixXf squad_last_pos_d;          // 小组成员上一次的影子

    MatrixXf squad_cur_pos_T;             // 小组成员的当前位置 转至
    MatrixXf squad_cur_pos_d_T;           // 小组成员的各自的影子 转至

    MatrixXf squad_last_pos_d_T;          // 小组成员上一次的影子 转至


    MatrixXf self_cur_pos_T;              // 自己的位置 转至
    MatrixXf self_cur_pos_d_T;            // 自己的影子 转至
    MatrixXf self_cur_pos_d_speed_T;      // 自己的影子速度 转至

    MatrixXf minus_laplace;             //负图矩阵
    MatrixXf minus_laplace_vec;         //负图矩阵行向量

    MatrixXf Identity_dn;               //做克罗内克积需要的单位矩阵

    float gama;                         //收敛因子，内部变量
    int squad_conv_speed;               //收敛速度，可在外部改变
    int iterate_times;                  //计算一致性循环迭代次数，内部变量

    MatrixXf self_dest_pos;             //自己的期望位置
    MatrixXf self_dest_pos_d;            //自己影子的期望位置
    MatrixXf self_dest_pos_T;             //自己的期望位置 转至
    MatrixXf self_dest_pos_d_T;           //自己影子的期望位置 转至

    MatrixXf kron_result;               //克罗内克积乘积的结果

    vector<int> needed_id;              //需要的id号 用向量存储

public:
    //成员函数 具体功能在实现部分有介绍
    C_DistrInfoConcencus();
    void setSelfId(int _self_id);
    void setConvSpeed(int _squad_conv_speed);

    template<typename Tensor_Input1>
    void setMinusLaplace(const Tensor_Input1& _minus_laplace );

    vector<int> getNeededId(void);


    template<typename Tensor_Input1, typename Tensor_Output1>
    void useIdAquirePosition(const Tensor_Input1& _position_template, Tensor_Output1& _squad_current_position);


    template<typename Tensor_Input2>
    void setIntialPosition(const Tensor_Input2& _position_template);


    template<typename Tensor_Input1, typename Tensor_Input2, typename Tensor_Output1>
    void kroneckerProduct_full(const Tensor_Input1& _A, const Tensor_Input2& _B, Tensor_Output1 & _AB);


    template<typename Tensor_Input1,typename Tensor_Input2,
             typename Tensor_Output1>
    void CalSelfDestPos(const Tensor_Input1& _squad_cur_pos, const Tensor_Input2& _squad_cur_pos_d,
                        Tensor_Output1& _self_dest_pos);


    template<typename Tensor_Input1, typename Tensor_Input2>
    void InitDistrConcensusPmts(int _self_id, int _conv_speed,
                                const Tensor_Input1& _minus_laplace,
                                const Tensor_Input2& _current_position_template);

    template<typename Tensor_Input1,typename Tensor_Input2,
             typename Tensor_Output1>
    void CalDistrConcensusOutput(const Tensor_Input1 &_current_position_template, const Tensor_Input2 &_expect_position_template,
                                 Tensor_Output1& _self_dest_position);
};


//分布式一致性的实现
/*@
     * 函数功能： 设置信号流图拉普拉斯矩阵 以及 提取局部拉普拉斯行向量等
     *    输入： 拉普拉斯矩阵 MatrixXf MatrixXd均可
     *    输出： 无
     */
/************************************************************************/
template<typename Tensor_Input1>
void C_DistrInfoConcencus::setMinusLaplace(const Tensor_Input1& _minus_laplace )
{
    int _count = 0;
    MatrixXf _middle_matrix(1,1);
    _middle_matrix << 0;            // 被用来比较的中间变量
    MatrixXf _minus_laplace_vec;    // 提取拉普拉斯向量的中间变量

    ArrayXXf _middle_array;         // 进行大小比较的中间变量
    ArrayXXf _minus_laplace_array;


    // 先捕获拉普拉斯矩阵，它内部存有很多信息
    minus_laplace = _minus_laplace;
    cout << "the minus laplace is : "<<endl;
    cout << minus_laplace <<endl;


    // 计算有几个友邻
    _minus_laplace_vec = _minus_laplace.row(self_id - 1);
    for(int i=0; i<_minus_laplace_vec.cols();i++)
    {
        if(_minus_laplace_vec.col(i) != _middle_matrix )
        {
            _count += 1;
            cout<< "_count = "<< _count<<endl;
        }
    }

    // 在统计的过程中把自己也统计进去了，减去1 就是友邻的数量
    _count = _count - 1;
    neighbour_agent_num = _count;
    cout << "neighbour agent num is : "<<endl;
    cout << neighbour_agent_num <<endl;


    _count = 0; //别忘记给 _count 清0


    // 利用友邻的数量对降维的拉普拉斯向量初始化
    minus_laplace_vec = MatrixXf(1, neighbour_agent_num+1);
    // 对降维拉普拉斯向量进行赋值
    for(int i=0; i<_minus_laplace_vec.cols();i++)
    {
        if(_minus_laplace_vec.col(i) != _middle_matrix )
        {
            minus_laplace_vec.col(_count) = _minus_laplace_vec.col(i);
            _count += 1;
            cout<< "_count = "<< _count<<endl;
        }
    }
    cout << "minus_laplace_vec is : "<<endl;
    cout << minus_laplace_vec <<endl;



    // 设置一些需要初始化的矩阵
    // 设置一下克罗内克积的单位矩阵
    Identity_dn = MatrixXf::Identity(1, 1);
    // 再设置一下克罗内克积结果的矩阵
    kron_result = MatrixXf(1 * 1, (neighbour_agent_num+1) * 1);

    _count = 0; //计数变量清0

    //进行数据转换，然后作比较
    _middle_array = _middle_matrix.array();
    _minus_laplace_array = minus_laplace_vec.array();


    // 由于排0操作后，向量的维度降维，要重新算出自己的下标跑到了哪个位置
    for(int i=0;i<minus_laplace_vec.cols();i++)
    {
        if(_minus_laplace_array(0, i) < _middle_array(0, 0)) // 只要比0小
        {
            self_index = _count;
            break;
        }
        else
        {
            _count += 1;
        }                           //否则 继续+1 向上统计
    }

    cout << "the self index is: "<< self_index <<endl;

}
/************************************************************************/
/************************************************************************/






template<typename Tensor_Input1, typename Tensor_Output1>
void C_DistrInfoConcencus::useIdAquirePosition(const Tensor_Input1& _position_template, Tensor_Output1& _squad_position)
{
    vector<Tensor_Input1> _position_vector_container;

    // 将对应的列提取出来 即每个机器人的坐标列向量
    for(int i=0; i<needed_id.size();i++)
    {
        _position_vector_container.push_back(_position_template.col(needed_id[i]-1));//机器人号从1开始，但矩阵下标从0开始
    }

    // 利用块操作将提取出来的坐标列向量拼接在一起
    for(int i=0; i< _position_vector_container.size(); i++)
    {
        _squad_position.block(0, i, dim_num, 1) = _position_vector_container[i];
    }
    cout << "local squad position is: "<< endl;
    cout << _squad_position << endl;

}


template<typename Tensor_Input2>
void C_DistrInfoConcencus::setIntialPosition(const Tensor_Input2& _position_template)
{
    vector<Tensor_Input2> _position_vector_container;

    dim_num = _position_template.rows();       //提取控制量维度
    squad_cur_pos = MatrixXf(dim_num, neighbour_agent_num+1);
    squad_cur_pos_d = MatrixXf(dim_num, neighbour_agent_num+1);
    squad_last_pos_d = MatrixXf(dim_num, neighbour_agent_num+1);

    squad_cur_pos_T = MatrixXf(neighbour_agent_num+1,dim_num);
    squad_cur_pos_d_T = MatrixXf(neighbour_agent_num+1,dim_num);
    squad_last_pos_d_T = MatrixXf( neighbour_agent_num+1, dim_num);


    // 将对应的列提取出来 即每个机器人的坐标列向量
    for(int i=0; i<needed_id.size();i++)
    {
        _position_vector_container.push_back(_position_template.col(needed_id[i] - 1));//机器人号从1开始，但矩阵下标从0开始
    }

    // 利用块操作将提取出来的坐标列向量拼接在一起
    for(int i=0; i< _position_vector_container.size(); i++)
    {
        squad_cur_pos.block(0, i, dim_num, 1) = _position_vector_container[i];
    }
    squad_last_pos_d = squad_cur_pos;
    squad_cur_pos_T = squad_cur_pos.transpose();
    squad_last_pos_d_T = squad_cur_pos_T;

    cout << "local squad current has been set: "<< endl;
    cout << squad_cur_pos << endl;
}


/*@
     * 函数功能： 采用eigen库块操作计算克罗内克积
     *    输入： 两个矩阵 可以是 Matrix所有型号，也可以是Vector的所有型号
     *    输出： 维数更大的克罗内克矩阵
     */
/************************************************************************/
template<typename Tensor_Input1, typename Tensor_Input2, typename Tensor_Output1>
void C_DistrInfoConcencus::kroneckerProduct_full(const Tensor_Input1& _A,const Tensor_Input2& _B, Tensor_Output1 & _AB)
{
    const unsigned int Ar = _A.rows(),
            Ac = _A.cols(),
            Br = _B.rows(),
            Bc = _B.cols();

    for (unsigned int i=0; i<Ar; ++i)
        for (unsigned int j=0; j<Ac; ++j)
            _AB.block(i*Br,j*Bc,Br,Bc) = _A(i,j)*_B; //利用eigen块操作实现K积
}
/************************************************************************/
/************************************************************************/


/*@
     * 函数功能： 利用一致性算法计算期望位置和影子位置
     *    输入： 1.当前位置向量 2.当前影子向量 3.当前影子速度 （可以是Matrix 也可以是Vector）
     *    输出： 1.期望位置向量 2.期望位置速度  （可以是Matrix 也可以是Vector）
     */
/************************************************************************/
template<typename Tensor_Input1,typename Tensor_Input2,
         typename Tensor_Output1>
void C_DistrInfoConcencus::CalSelfDestPos( const Tensor_Input1& _squad_cur_pos,const Tensor_Input2& _squad_cur_pos_d,
                                           Tensor_Output1& _self_dest_pos )
{

    squad_cur_pos_T = _squad_cur_pos.transpose();
    squad_cur_pos_d_T = _squad_cur_pos_d.transpose();

    //cout<< 11111<<endl;
    self_cur_pos_d_speed_T = (squad_cur_pos_d_T - squad_cur_pos_T).row(self_index);

    self_cur_pos_T = squad_cur_pos_T.row(self_index);
    self_cur_pos_d_T = squad_cur_pos_d_T.row(self_index);
    //cout<< 22222<<endl;

    // cur -> last for next iteration
    squad_last_pos_d_T = squad_cur_pos_d_T;

    kroneckerProduct_full(minus_laplace_vec,Identity_dn,kron_result);

    self_dest_pos_T = self_cur_pos_T + self_cur_pos_d_speed_T + gama * kron_result * (squad_cur_pos_T - squad_cur_pos_d_T);
    //self_dest_pos_d = self_cur_pos_d + self_cur_pos_d_speed;  //这个量先不输出

    _self_dest_pos = self_dest_pos_T.transpose();  // 输出时转至成列向量，以常规格式输出
    //_self_dest_pos_d = self_dest_pos_d.transpose();

}
/************************************************************************/
/************************************************************************/

template<typename Tensor_Input1, typename Tensor_Input2>
void C_DistrInfoConcencus::InitDistrConcensusPmts(int _self_id, int _conv_speed,
                                                  const Tensor_Input1& _minus_laplace,
                                                  const Tensor_Input2& _current_position_template)
{
    setSelfId(_self_id);
    setConvSpeed(_conv_speed);
    setMinusLaplace(_minus_laplace);
    getNeededId();
    setIntialPosition(_current_position_template);
}




template<typename Tensor_Input1,typename Tensor_Input2,
         typename Tensor_Output1>
void C_DistrInfoConcencus::CalDistrConcensusOutput(const Tensor_Input1 &_current_position_template,
                                                   const Tensor_Input2 &_expect_position_template,

                                                   Tensor_Output1& _self_dest_position)
{
    useIdAquirePosition(_current_position_template, squad_cur_pos);
    useIdAquirePosition(_expect_position_template, squad_cur_pos_d);

    CalSelfDestPos(squad_cur_pos, squad_cur_pos_d, self_dest_pos); // 前两个输入，后一个为输出

    cout<< "my destinate location is: " <<endl;
    cout<< self_dest_pos <<endl;

    _self_dest_position = self_dest_pos;

}


// 一阶一致性  输出期望速度版本
class C_SingleIntegConcensusFormation
{
public:
    int self_id;                // 机器人自身编号
    int robot_num;              // 机器人的个数
    int state_num;              // 状态变量的个数

    int neighbour_agent_num;    // 友邻机器人的个数

    MatrixXf laplace_n;         // 维数是n的拉普拉斯矩阵
    MatrixXf laplace_n_1;       // 维数是n+1的拉普拉斯矩阵
    MatrixXf adjacent_n;        // n维邻接矩阵
    MatrixXf adjacent_n_1;      // n+1维邻接矩阵

    VectorXf ref_pos;           // 参考点的位置 列向量 维数最大时分别为x，y，z，yaw
    VectorXf ref_speed;         // 参考点的速度 列向量
    MatrixXf ref_pos_T;         // 转置成行向量
    MatrixXf ref_speed_T;

    MatrixXf devia_pos;         // 所有机器人偏离参考点的位置 每一列代表一个机器人偏离值 从上到下分别是 x y z yaw
    MatrixXf devia_speed;       // 所有机器人偏离参考点的速度
    MatrixXf devia_pos_T;       // 转置
    MatrixXf devia_speed_T;     // 转置

    MatrixXf cur_pos;           // 所有机器人的当前位置   每一列代表一个机器人偏离值 从上到下分别是 x y z yaw
    MatrixXf cur_speed;         // 所有机器人的当前速度
    MatrixXf cur_pos_T;         // 转置
    MatrixXf cur_speed_T;       // 转置

    float gamma;                // 需要调节的参数gamma 增大gamma可增加收敛速度
    MatrixXf target_pos;        // 机器人根据目标点和自己的偏离值，计算出属于自己的目标点
    MatrixXf target_speed;
    MatrixXf target_pos_T;      // 转置
    MatrixXf target_speed_T;

    VectorXf output;            // 各状态量对应的输出
    MatrixXf output_T;          // 转置


public:

    // 初始化
    C_SingleIntegConcensusFormation();
    C_SingleIntegConcensusFormation(int _self_id, int _robot_num, int _state_num);
    ~C_SingleIntegConcensusFormation();

    void setLaplaceMatrix(MatrixXf _laplace_n, MatrixXf _laplace_n_1);
    void setGamma(float _gamma);

    // 编队保持时用这个控制量输出
    VectorXf calFormationMaintainOutput(VectorXf _ref_pos, MatrixXf _devia_pos, MatrixXf _cur_pos);

    // 集中式用这个
    VectorXf calFormationTrackingOutput(VectorXf _ref_pos, VectorXf _ref_speed,
                                        MatrixXf _devia_pos, MatrixXf _devia_speed,
                                        MatrixXf _cur_pos);

    // 分布式用这个
    VectorXf calFormationTrackingOutput_v2(VectorXf _ref_pos, VectorXf _ref_speed,
                                           MatrixXf _devia_pos, MatrixXf _devia_speed,
                                           MatrixXf _cur_pos, MatrixXf _cur_speed);
};


}


#endif // C_MULTI_AGENT_ALGORITHM_H
