#include "../../include/asrobot/c_multi_agent_algorithm.h"

namespace MultiAgentAlgorithm {

/*!
    * \brief  人工势场实现
    */
APF::APF(){

}

APF::~APF(){

}

void APF::set_krep(float a){
    k_rep = a;
    cout << " repulsion coefficient [k_rep]: " << k_rep << endl;
}

void APF::set_r(float a){
    r = a;
    cout << " the agent radius [r]: "<< r <<endl;
}

void APF::set_r0(float a){
    r0 = a;
    cout << " the extern radius [r0]: "<< r0 << endl;
}

float APF::get_krep(){
    return k_rep;
}

float APF::get_r(){
    return r;
}

float APF::get_r0(){
    return r0;
}

VectorXf APF::apf_InterRepulsion(VectorXf p_agent,MatrixXf p_otheragent){
    float n_otheragent = 1;
    float R = r + r0;
    MatrixXf v_agent;//每个机器人此机器人之间的向量差
    MatrixXf d_agent;
    MatrixXf Scale_rep;

    dimension = p_agent.rows();//二维或三维坐标
    n_otheragent = p_otheragent.cols();//除去此机器人的个数
    Scale_rep.setZero(1, n_otheragent);

    v_agent = p_otheragent.colwise() - p_agent;
    d_agent = v_agent.colwise().squaredNorm();

    for(int i = 0; i < n_otheragent; i++){
        if(d_agent(i) > r && d_agent(i) < R){
            Scale_rep(i) = k_rep * (1 / (pow((d_agent(i) - r), 3)));
        }
    }

    v_agent = Scale_rep.replicate(dimension,1).cwiseProduct(v_agent);

    pv_agent=v_agent.rowwise().sum();
    return pv_agent;

}

/*@
 * Introduction: obstacle avaidence
 *
 * Input:   p_agent       robot position
 *          p_obstacle    obstacle position
 *          r_obstacle    obstacle radius
 * return:  pv_agent      robot offset
*/
VectorXf APF::apf_ExterRepulsion(VectorXf p_agent, MatrixXf p_obstacle, MatrixXf r_obstacle){
    float n_obstacle=1;
    MatrixXf v_agent;	//  每个障碍物与此机器人之间的向量差
    MatrixXf d_agent;   //
    MatrixXf Scale_rep;

    dimension = p_agent.rows();     //  二维或三维坐标
    n_obstacle = p_obstacle.cols(); //  障碍物的个数
    Scale_rep.setZero(1,n_obstacle);
    MatrixXf R;
    R.setOnes(1,n_obstacle);
    R = r0 * R + r_obstacle;

    v_agent = p_obstacle.colwise() - p_agent;
    d_agent = v_agent.colwise().squaredNorm();

    for(int i = 0; i < n_obstacle; i++){
        if(d_agent(i) > r_obstacle(i) && d_agent(i) < R(i)){
            Scale_rep(i) = k_rep * (1 / (pow((d_agent(i) - r_obstacle(i)), 3)));
        }
    }

    v_agent = Scale_rep.replicate(dimension,1).cwiseProduct(v_agent);

    pv_agent = v_agent.rowwise().sum();
    return pv_agent;
}


/*!
    * \brief  虚拟结构实现
    */

VirtualStructure::VirtualStructure(){

}

VirtualStructure::VirtualStructure(int n, float line, float TriSide, float yaw)
{
    this->robotNum = n;
    this->lineLength = line;
    this->TriangularSideLength = TriSide;
    this->vs_yaw = yaw;

    vs_pointInWorld.x = 10;
    vs_pointInWorld.y = 10;
    vs_pointInWorld.z = 0;

    robotPosMatInVS.setZero(3, robotNum);

}

VirtualStructure::~VirtualStructure()
{

}

void VirtualStructure::setFormationParams(int n, float line, float TriSide, float yaw){
    this->robotNum = n;
    this->lineLength = line;
    this->TriangularSideLength = TriSide;
    this->vs_yaw = yaw;
    robotPosMatInVS.setZero(3, robotNum);
}

vector<Point3f> VirtualStructure::calcRobotsPosInTriangularForm()
{
    // virtual Leader Position(0,0,0)
    robotPosListInVS.clear();
    int temp = (robotNum - 1) / 2;
    cv::Point3f robotPosTemp;
    float temp_x = TriangularSideLength * cos(M_PI * 30/180) / temp;
    float temp_y = TriangularSideLength * sin(M_PI * 30/180) / temp;

    for (int i = 0; i <= (robotNum - 1) / 2; i++){
        robotPosTemp.x = -sqrt(3) * TriangularSideLength/6 + temp_x * i;
        robotPosTemp.y = -TriangularSideLength/2 + temp_y * i;
        robotPosTemp.z = 0;
        robotPosListInVS.push_back(robotPosTemp);
    }
    for (int i = (robotNum + 1) / 2; i < robotNum; i++){
        robotPosTemp.x = -sqrt(3) * TriangularSideLength/6 + temp_x * (robotNum - 1 - i);
        robotPosTemp.y = -TriangularSideLength/2 + temp_y * i;
        robotPosTemp.z = 0;
        robotPosListInVS.push_back(robotPosTemp);
    }

    for (int i = 0; i < robotNum; i++){
        cout << "robot_" << i << ": " << robotPosListInVS[i].x << ", "
             << robotPosListInVS[i].y << ", "<< robotPosListInVS[i].z << endl;
    }
    return robotPosListInVS;
}

vector<Point3f> VirtualStructure::calcRobotsPosInLatLineForm()
{
    robotPosListInVS.clear();
    float temp_x = 0;
    float temp_y = lineLength/(robotNum - 1);
    cv::Point3f robotPosTemp;

    for(int i = 0; i <= robotNum - 1; i++){
        robotPosTemp.x = 0;
        robotPosTemp.y = -temp_y * ((robotNum-1)/2-i);
        robotPosTemp.z = 0;
        robotPosListInVS.push_back(robotPosTemp);
    }
    for (int i = 0; i < robotNum; i++){
        cout << "robot_" << i << ": " << robotPosListInVS[i].x << ", "
             << robotPosListInVS[i].y << ", "<< robotPosListInVS[i].z << endl;
    }
    return robotPosListInVS;
}

vector<Point3f> VirtualStructure::calcRobotsPosInLonLineForm()
{
    robotPosListInVS.clear();
    float temp_x = lineLength/(robotNum - 1);
    float temp_y = 0;
    cv::Point3f robotPosTemp;

    for(int i = 0; i <= robotNum - 1; i++){
        robotPosTemp.x = temp_x * ((robotNum-1)/2-i);
        robotPosTemp.y = 0;
        robotPosTemp.z = 0;
        robotPosListInVS.push_back(robotPosTemp);
    }
    for (int i = 0; i < robotNum; i++){
        cout << "robot_" << i << ": " << robotPosListInVS[i].x << ", "
             << robotPosListInVS[i].y << ", "<< robotPosListInVS[i].z << endl;
    }
    return robotPosListInVS;
}

Eigen::MatrixXf VirtualStructure::transform2WorldFrame(){

    Eigen::Vector3f posVecInWorld;
    for(int i = 0; i < robotPosListInVS.size(); i++){ // 弧度制

        posVecInWorld(0) = robotPosListInVS[i].x * cos(vs_yaw) + robotPosListInVS[i].y * sin(vs_yaw)
                + vs_pointInWorld.x;
        posVecInWorld(1) = robotPosListInVS[i].y * cos(vs_yaw) - robotPosListInVS[i].x * sin(vs_yaw)
                + vs_pointInWorld.y;
        posVecInWorld(2) = 0;
        robotPosMatInVS.block(0,i,3,1) = posVecInWorld;
    }
    return robotPosMatInVS;
}
/*!
    * \brief  类referenceTrajectory实现
    * 
    * 计算多机编队中各个机器人的参考位置信息,默认是三台机器人,需要把全局规划好的轨迹的位置信息传进来
    * 三角形队形默认是等边三角形
    */

referenceTrajectory::referenceTrajectory(){

}

referenceTrajectory::referenceTrajectory(int n, float line, float TriSide,MatrixXf ref_Info)
{
    robotNum = n;
    lineLength = line;
    TriangularSideLength = TriSide;
    vs_ref_Info_=ref_Info;
    ref_Point_Num=ref_Info.rows();
    ref_state_Num=ref_Info.cols();
    virtureRobot_ref_Info_=ref_Info;
    robot1_ref_Info_=ref_Info;
    robot2_ref_Info_=ref_Info;
    robot3_ref_Info_=ref_Info;

}

referenceTrajectory::~referenceTrajectory()
{

}

void referenceTrajectory::setFormationParams(int n, float line, float TriSide){
    robotNum = n;
    lineLength = line;
    TriangularSideLength = TriSide;
    
}
void referenceTrajectory::update_ref_Info(MatrixXf ref_Info)
{
    vs_ref_Info_=ref_Info;

}
void referenceTrajectory::calcRobotsPosInLatLineForm()
{
   
      //float dx=0;
      float dy=lineLength/(robotNum-1);
      MatrixXf DY=MatrixXf::Zero(ref_Point_Num,ref_state_Num);
      DY.block(0,1,ref_Point_Num,1)=DY.col(1)+dy*VectorXf::Ones(ref_Point_Num);
      robot1_ref_Info_=vs_ref_Info_+DY;  //y+dy
      robot2_ref_Info_=vs_ref_Info_;
      robot3_ref_Info_=vs_ref_Info_-DY;  //y-dy
}
void referenceTrajectory::calcRobotsPosInLonLineForm()
{
    float dx=lineLength/(robotNum-1);
    MatrixXf DX=MatrixXf::Zero(ref_Point_Num,ref_state_Num);
    DX.col(0)=DX.col(0)+dx*VectorXf::Ones(ref_Point_Num);
    robot1_ref_Info_=vs_ref_Info_+DX;
    robot2_ref_Info_=vs_ref_Info_;
    robot3_ref_Info_=vs_ref_Info_-DX;
}
void referenceTrajectory::calcRobotsPosInTriangularForm()
{

    MatrixXf robotPosListInVS(2, robotNum);
    int temp = (robotNum - 1) / 2;
    cv::Point2f robotPosTemp;
    float temp_y = TriangularSideLength * cos(M_PI * 30 / 180) / temp;
    float temp_x = TriangularSideLength * sin(M_PI * 30 / 180) / temp;

    // for (int i = 0; i <= (robotNum - 1) / 2; i++)
    // {
    //     robotPosTemp.x = -sqrt(3) * TriangularSideLength / 6 + temp_x * i;
    //     robotPosTemp.y = -TriangularSideLength / 2 + temp_y * i;
    //     robotPosListInVS(0, i) = robotPosTemp.x;
    //     robotPosListInVS(1, i) = robotPosTemp.y;
    // }
    // for (int i = (robotNum + 1) / 2; i < robotNum; i++)
    // {
    //     robotPosTemp.x = -sqrt(3) * TriangularSideLength / 6 + temp_x * (robotNum - 1 - i);
    //     robotPosTemp.y = -TriangularSideLength / 2 + temp_y * i;
    //     robotPosListInVS(0, i) = robotPosTemp.x;
    //     robotPosListInVS(1, i) = robotPosTemp.y;
    // }
    robotPosListInVS(0, 0)=0;
    robotPosListInVS(1, 0)=temp_y*2/3;
    robotPosListInVS(0, 1)=-temp_x;
    robotPosListInVS(1, 1)=-temp_y/3;
    robotPosListInVS(0, 2)=temp_x;
    robotPosListInVS(1, 2)=-temp_y/3;

    for (int j = 0; j < vs_ref_Info_.rows(); j++)
    {
        float vs_yaw = vs_ref_Info_(j, 2);
        robot1_ref_Info_(j, 0) = robotPosListInVS(0, 0) * cos(vs_yaw) + robotPosListInVS(1, 0) * sin(vs_yaw) + vs_ref_Info_(j, 0);
        robot1_ref_Info_(j, 1) = robotPosListInVS(1, 0) * cos(vs_yaw) - robotPosListInVS(0, 0) * sin(vs_yaw) + vs_ref_Info_(j, 1);
        robot2_ref_Info_(j, 0) = robotPosListInVS(0, 1) * cos(vs_yaw) + robotPosListInVS(1, 1) * sin(vs_yaw) + vs_ref_Info_(j, 0);
        robot2_ref_Info_(j, 1) = robotPosListInVS(1, 1) * cos(vs_yaw) - robotPosListInVS(0, 1) * sin(vs_yaw) + vs_ref_Info_(j, 1);
        robot3_ref_Info_(j, 0) = robotPosListInVS(0, 2) * cos(vs_yaw) + robotPosListInVS(1, 2) * sin(vs_yaw) + vs_ref_Info_(j, 0);
        robot3_ref_Info_(j, 1) = robotPosListInVS(1, 2) * cos(vs_yaw) - robotPosListInVS(0, 2) * sin(vs_yaw) + vs_ref_Info_(j, 1);
    }
}
void referenceTrajectory::updata_robot_ref_Info(int formationshape)
{
    if(formationshape==1)
    calcRobotsPosInLatLineForm();
    if(formationshape==2)
    calcRobotsPosInLonLineForm();
    if(formationshape==0)
    calcRobotsPosInTriangularForm();

}
/*!
    * \brief  虚拟连杆实现
    */

VirtualLinkageStructure::VirtualLinkageStructure(Eigen::Vector3f vls_point, float fTheta,
                                                 float fLeftBeta, float fRightBeta)
    :vls_pointInWorld(vls_point),fTheta(fTheta),fLeftBeta(fLeftBeta),fRightBeta(fRightBeta)
{
    // 设置连杆模式
//    setLinkType(link_type);
}

VirtualLinkageStructure::VirtualLinkageStructure(int link_type_, Eigen::Vector3f vls_point_, float fAlpha_,
                        float fLeftBeta_, float fRightBeta_, float leftLinkage_, float rightLinkage_)
{
    // 设置连杆模式
    setLinkType(link_type_);
    // 初始化参数
    this->fAlpha = fAlpha_;
    this->vls_pointInWorld = vls_point_;
    this->fLeftBeta = fLeftBeta_;
    this->fRightBeta = fRightBeta_;
    this->fLeftLinkageLength = leftLinkage_;
    this->fRightLinkageLength = rightLinkage_;

    vlMatPointWorld.setZero(3, nVL_num+1);
    vls_pointInWorld.setZero(3,1);    // 虚拟连杆机构参考点在世界坐标系下位置
}

VirtualLinkageStructure::VirtualLinkageStructure(int link_type)
{
    // 设置连杆模式
    setLinkType(link_type);

    vlMatPointWorld.setZero(3, nVL_num+1);
    vls_pointInWorld.setZero(3,1);    // 虚拟连杆机构参考点在世界坐标系下位置

}

VirtualLinkageStructure::~VirtualLinkageStructure()
{

}

void VirtualLinkageStructure::setVLNum(int num, int lnum, int rnum){
    this->nVL_num = num;
    this->leftVL_num = lnum;
    this->rightVL_num = rnum;
}

void VirtualLinkageStructure::setLinkType(int link_type){
    this->nLinkType = link_type;
    switch (link_type){
        case LINK_NUM_1_1:
            nVL_num = 2;
            leftVL_num = 1;
            rightVL_num = 1;
            break;
        case LINK_NUM_1_2:
            nVL_num = 3;
            leftVL_num = 1;
            rightVL_num = 2;
            break;
        case LINK_NUM_2_1:
            nVL_num = 3;
            leftVL_num = 2;
            rightVL_num = 1;
            break;
        case LINK_NUM_2_2:
            nVL_num = 4;
            leftVL_num = 2;
            rightVL_num = 2;
            break;
    }
}

void VirtualLinkageStructure::InitializeParameters() {
    Eigen::Vector3f temp(0,0,0);

    if(LINK_NUM_1_1 == nLinkType){
        vectorClear();  // clear vector

        for(int i = 0; i < nVL_num + 1; i++){
            fAlphaVector.push_back(0);
            fBetaVector.push_back(0);
            fGamaVector.push_back(0);
            fLinkageLength.push_back(0);
            vlPosInVLS.push_back(temp);
        }

        // set alpha, beta, linkagelength.
        fAlphaVector[0] = M_PI * fAlpha/180;
        fAlphaVector[1] = 0;
        fAlphaVector[2] = 0;
        fBetaVector[0] = M_PI * fLeftBeta/180;
        fBetaVector[1] = M_PI * fRightBeta/180;
        fBetaVector[2] = 0;
        fLinkageLength[0] = 100;
        fLinkageLength[1] = 100;
        fLinkageLength[2] = 100;

        // calculate gama
        fGamaVector[0] = M_PI/2 - fAlphaVector[0]/2;
        fGamaVector[1] = fGamaVector[0];

    }else if (LINK_NUM_2_2 == nLinkType) {
        // clear vector
        vectorClear();

        for(int i = 0; i < nVL_num + 1; i++){
            fAlphaVector.push_back(0);
            fBetaVector.push_back(0);
            fGamaVector.push_back(0);
            fLinkageLength.push_back(0);
            vlPosInVLS.push_back(temp);
        }
        // set alpha, beta, linkagelength.
        fAlphaVector[0] = 0;
        fAlphaVector[1] = M_PI * fAlpha/180;
        fAlphaVector[2] = 0;
        fAlphaVector[3] = 0;
        fAlphaVector[4] = 0;
        fBetaVector[0] = M_PI * fLeftBeta/180;
        fBetaVector[1] = M_PI * fLeftBeta/180;
        fBetaVector[2] = M_PI * fRightBeta/180;
        fBetaVector[3] = M_PI * fRightBeta/180;
        fBetaVector[4] = M_PI * 120/180;
        fLinkageLength[0] = 100;
        fLinkageLength[1] = 100;
        fLinkageLength[2] = 100;
        fLinkageLength[3] = 100;
        fLinkageLength[4] = 100;

        // calculate gama
        fGamaVector[1] = M_PI/2 - fAlphaVector[1]/2;
        fGamaVector[0] = fGamaVector[1] + fAlphaVector[0];
        fGamaVector[2] = fGamaVector[1];
        fGamaVector[3] = fGamaVector[2] + fAlphaVector[2];

    }else if (LINK_NUM_2_1 == nLinkType){
        // clear vector
        vectorClear();

        for(int i = 0; i < nVL_num + 1; i++){
            fAlphaVector.push_back(0);
            fBetaVector.push_back(0);
            fGamaVector.push_back(0);
            fLinkageLength.push_back(0);
            vlPosInVLS.push_back(temp);
        }
        // set alpha, beta, linkagelength.
        fAlphaVector[0] = 0;
        fAlphaVector[1] = M_PI * fAlpha/180;
        fAlphaVector[2] = 0;
        fAlphaVector[3] = 0;
        fBetaVector[0] = M_PI * fLeftBeta/180;
        fBetaVector[1] = M_PI * fLeftBeta/180;
        fBetaVector[2] = M_PI * fRightBeta/180;
        fBetaVector[3] = M_PI * 0/180;
        fLinkageLength[0] = 100;
        fLinkageLength[1] = 100;
        fLinkageLength[2] = 100;
        fLinkageLength[3] = 100;

        // calculate gama
        fGamaVector[1] = M_PI/2 - fAlphaVector[1]/2;
        fGamaVector[0] = fGamaVector[1] + fAlphaVector[0];
        fGamaVector[2] = fGamaVector[1];

    }else if (LINK_NUM_1_2 == nLinkType){
        // clear vector
        vectorClear();

        for(int i = 0; i < nVL_num + 1; i++){
            fAlphaVector.push_back(0);
            fBetaVector.push_back(0);
            fGamaVector.push_back(0);
            fLinkageLength.push_back(0);
            vlPosInVLS.push_back(temp);
        }

        // set alpha, beta, linkagelength.
        fAlphaVector[0] = M_PI * fAlpha/180;
        fAlphaVector[1] = 0;
        fAlphaVector[2] = 0;
        fAlphaVector[3] = 0;
        fBetaVector[0] = M_PI * fLeftBeta/180;
        fBetaVector[1] = M_PI * fRightBeta/180;
        fBetaVector[2] = M_PI * fRightBeta/180;
        fBetaVector[3] = M_PI * 0/180;
        fLinkageLength[0] = 100;
        fLinkageLength[1] = 100;
        fLinkageLength[2] = 100;
        fLinkageLength[3] = 100;

        // calculate gama
        fGamaVector[0] = M_PI/2 - fAlphaVector[0]/2;
        fGamaVector[1] = fGamaVector[0];
        fGamaVector[2] = fGamaVector[1] + fAlphaVector[1];
    }
}

void VirtualLinkageStructure::setVLSAngle(float fAlpha_, float fLeftBeta_, float fRightBeta_)
{
    this->fAlpha = fAlpha_;
    this->fLeftBeta = fLeftBeta_;
    this->fRightBeta = fRightBeta_;
}

void VirtualLinkageStructure::setLinkageLength(float leftLinkage_, float rightLinkage_){
    this->fLeftLinkageLength = leftLinkage_;
    this->fRightLinkageLength = rightLinkage_;
}

void VirtualLinkageStructure::setVLPointInworld(Vector3f vls_pointInWorld_){
    this->vls_pointInWorld = vls_pointInWorld_;    // 虚拟连杆机构参考点在世界坐标系下位置
}

void VirtualLinkageStructure::setVLS_Angle_RYP(float roll, float yaw, float pitch){
    this->roll = roll;
    this->yaw = yaw;
    this->pitch = pitch;
}

void VirtualLinkageStructure::vectorClear(){
    fAlphaVector.clear();
    fBetaVector.clear();
    fGamaVector.clear();
    fLinkageLength.clear();
    vlPosInVLS.clear();
    vlPosInWorld.clear();
}

/*                2
 *          1            3
*/
void VirtualLinkageStructure::calcVL_1_1_Pos(){
    // calcualte left linkage
    Eigen::Vector3f temp;
    Eigen::Vector3f reference(0,0,0); // 虚拟连杆参考点
    // 赋值关节2
    vlPosInVLS[1] = reference;

    // 计算关节1
    Eigen::Vector3f temp1(fLinkageLength[0] * cos(fBetaVector[0]) * cos(fGamaVector[0]),
            fLinkageLength[0] * cos(fBetaVector[0]) * sin(fGamaVector[0]),
            fLinkageLength[0] * sin(fBetaVector[0]));
    temp = reference - temp1;
    vlPosInVLS[0] = temp;

    // calcualte right linkage
    // 计算关节3
    Eigen::Vector3f temp2(fLinkageLength[1] * cos(fBetaVector[1]) * cos(fGamaVector[1]),
            -fLinkageLength[1] * cos(fBetaVector[1]) * sin(fGamaVector[1]),
            fLinkageLength[1] * sin(fBetaVector[1]));
    temp = reference - temp2;
    vlPosInVLS[2] = temp;
}

/*                        2
 *                  1           3
 *                                    4
*/
void VirtualLinkageStructure::calcVL_1_2_Pos() {
    // calcualte left linkage
    Eigen::Vector3f temp;
    Eigen::Vector3f reference(0,0,0); // 虚拟连杆参考点
    // 赋值关节3
    vlPosInVLS[1] = reference;

    // 计算关节1
    Eigen::Vector3f temp1(fLinkageLength[0] * cos(fBetaVector[0]) * cos(fGamaVector[0]),
            fLinkageLength[0] * cos(fBetaVector[0]) * sin(fGamaVector[0]),
            fLinkageLength[0] * sin(fBetaVector[0]));
    temp = reference - temp1;
    vlPosInVLS[0] = temp;

    // calcualte right linkage
    // 计算关节3
    Eigen::Vector3f temp2(fLinkageLength[1] * cos(fBetaVector[1]) * cos(fGamaVector[1]),
            -fLinkageLength[1] * cos(fBetaVector[1]) * sin(fGamaVector[1]),
            fLinkageLength[1] * sin(fBetaVector[1]));
    temp = reference - temp2;
    vlPosInVLS[2] = temp;

    // 计算关节4
    Eigen::Vector3f temp3(fLinkageLength[2] * cos(fBetaVector[2]) * cos(fGamaVector[2]),
            -fLinkageLength[2] * cos(fBetaVector[2]) * sin(fGamaVector[2]),
            fLinkageLength[2] * sin(fBetaVector[2]));
    temp = vlPosInVLS[2] - temp3;
    vlPosInVLS[3] = temp;
}

/*                          3
 *                    2            4
 *             1
 *
*/
void VirtualLinkageStructure::calcVL_2_1_Pos(){
    // calcualte left linkage
    Eigen::Vector3f temp;
    Eigen::Vector3f reference(0,0,0); // 虚拟连杆参考点
    // 赋值关节3
    vlPosInVLS[2] = reference;

    // 计算关节2
    Eigen::Vector3f temp1(fLinkageLength[1] * cos(fBetaVector[1]) * cos(fGamaVector[1]),
            fLinkageLength[1] * cos(fBetaVector[1]) * sin(fGamaVector[1]),
            fLinkageLength[1] * sin(fBetaVector[1]));
    temp = reference - temp1;
    vlPosInVLS[1] = temp;

    // 计算关节1
    Eigen::Vector3f temp2(fLinkageLength[0] * cos(fBetaVector[0]) * cos(fGamaVector[0]),
            fLinkageLength[0]*cos(fBetaVector[0])*sin(fGamaVector[0]),
            fLinkageLength[0]*sin(fBetaVector[0]));
    temp = vlPosInVLS[1] - temp2;
    vlPosInVLS[0] = temp;

    // 计算关节4
    Eigen::Vector3f temp3(fLinkageLength[2] * cos(fBetaVector[2]) * cos(fGamaVector[2]),
            -fLinkageLength[2] * cos(fBetaVector[2]) * sin(fGamaVector[2]),
            fLinkageLength[2] * sin(fBetaVector[2]));
    temp = reference - temp3;
    vlPosInVLS[3] = temp;
}

/*                       3
 *                 2            4
 *           1                         5
*/
void VirtualLinkageStructure::calcVL_2_2_Pos()
{
    // calcualte left linkage
    Eigen::Vector3f temp;
    Eigen::Vector3f reference(0,0,0); // 虚拟连杆参考点
    // 赋值关节3
    vlPosInVLS[2] = reference;

    // 计算关节2
    Eigen::Vector3f temp1(fLinkageLength[1] * cos(fBetaVector[1]) * cos(fGamaVector[1]),
            fLinkageLength[1] * cos(fBetaVector[1]) * sin(fGamaVector[1]),
            fLinkageLength[1] * sin(fBetaVector[1]));
    temp = reference - temp1;
    vlPosInVLS[1] = temp;

    // 计算关节1
    Eigen::Vector3f temp2(fLinkageLength[0] * cos(fBetaVector[0]) * cos(fGamaVector[0]),
            fLinkageLength[0]*cos(fBetaVector[0])*sin(fGamaVector[0]),
            fLinkageLength[0]*sin(fBetaVector[0]));
    temp = vlPosInVLS[1] - temp2;
    vlPosInVLS[0] = temp;

    // 计算关节4
    Eigen::Vector3f temp3(fLinkageLength[2] * cos(fBetaVector[2]) * cos(fGamaVector[2]),
            -fLinkageLength[2] * cos(fBetaVector[2]) * sin(fGamaVector[2]),
            fLinkageLength[2] * sin(fBetaVector[2]));
    temp = reference - temp3;
    vlPosInVLS[3] = temp;

    // 计算关节5
    Eigen::Vector3f temp4(fLinkageLength[3] * cos(fBetaVector[3]) * cos(fGamaVector[3]),
            -fLinkageLength[3] * cos(fBetaVector[3]) * sin(fGamaVector[3]),
            fLinkageLength[3] * sin(fBetaVector[3]));
    temp = vlPosInVLS[3] - temp4;
    vlPosInVLS[4] = temp;
}

/*
 * output: vlPosInWorld
*/
Eigen::MatrixXf VirtualLinkageStructure::calcVL_Pos()
{
    InitializeParameters();

    switch(nLinkType){
    case (LINK_NUM_1_1):
        calcVL_1_1_Pos();
        break;
    case (LINK_NUM_1_2):
        calcVL_1_2_Pos();
        break;
    case (LINK_NUM_2_1):
        calcVL_2_1_Pos();
        break;
    case (LINK_NUM_2_2):
        calcVL_2_2_Pos();
        break;
    }

    transform2WorldFrame();
    return vlMatPointWorld;
}

void VirtualLinkageStructure::transform2WorldFrame()
{
    vlPosInWorld.clear();

    Eigen::Vector3f eulerAngle(M_PI * yaw/180, M_PI * pitch/180, M_PI * roll/180);
    Eigen::AngleAxisf rollAngle(AngleAxisf(eulerAngle[2], Vector3f::UnitX()));
    Eigen::AngleAxisf pitchAngle(AngleAxisf(eulerAngle[1], Vector3f::UnitY()));
    Eigen::AngleAxisf yawAngle(AngleAxisf(eulerAngle[0], Vector3f::UnitZ()));

    // rotation_matrix = yawAngle * pitchAngle * rollAngle;
    rotation_vector = yawAngle * pitchAngle * rollAngle;
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity(); //虽然称为 3d ,实质上是 4*4 的矩阵
    T.rotate( rotation_vector );        //按照 rotation_vector 进行旋转
    T.pretranslate (vls_pointInWorld);  // 把平移向量设成 (1,3,4)
    cout << "Transform matrix = \n" << T.matrix() <<endl;

    // step2: transform robots position in vls into position in world.
    for(int i = 0; i < nVL_num + 1; i++){
        Eigen::Vector3f vlPos_transformed = T * vlPosInVLS[i]; // 相当于 R*v+t
        cout << "vlPos tranformed = " << vlPos_transformed.transpose() << endl;
        vlPosInWorld.push_back( vlPos_transformed );
        vlMatPointWorld.block(0,i,3,1) = vlPos_transformed;
    }
    cout << "vlMatPointWorld"  << endl << vlMatPointWorld << endl;
    cout << "vlPosInWorld.size   " << vlPosInWorld.size() << endl;
}






/*!
    * \brief  集中式一致性后半部分实现， 前半部分模板实现在.h文件中
    */

/*@
* 函数功能： 实例化基础参数 包括内部迭代次数 收敛因子
*    输入： 无
*    输出： 无
*/
/************************************************************************/
C_CentrInfoConcencus::C_CentrInfoConcencus()
{
    iterate_times = 10;//内部循环次数为10
    gama = 0.01; //收敛因子暂定0.01
}
/************************************************************************/
/************************************************************************/



/*@
* 函数功能： 设置智能体个数 和运动维度
*    输入： 智能体个数 运动维度 以及各种中间矩阵
*    输出： 无
*/
/************************************************************************/
void C_CentrInfoConcencus::setAgentNumAndDimNum(int _agent_num, int _dim_num)
{
    agent_num = _agent_num;
    dim_num = _dim_num;
    cur_pos = MatrixXf(dim_num, agent_num);
    cur_pos_d = MatrixXf(dim_num, agent_num);
    cur_pos_speed = MatrixXf(dim_num, agent_num);
    minus_laplace = MatrixXf(agent_num, agent_num);
    Identity_dn = MatrixXf::Identity(1, 1);
    kron_result = MatrixXf(agent_num * 1, agent_num * 1);
    cout << "the agent num is: "<< agent_num<< endl;
    cout << "the dim num  is: "<< dim_num<< endl;
}
/************************************************************************/
/************************************************************************/


/*@
* 函数功能： 设置收敛速度
*    输入： 收敛速度， 为整数类型
*    输出： 无
*/
/************************************************************************/
void C_CentrInfoConcencus::setConvSpeed(int _conv_speed)
{
    conv_speed = _conv_speed;
    gama = 0.01 * conv_speed;
    cout << "the conv speed is: "<< conv_speed<< endl;
}
/************************************************************************/
/************************************************************************/
/*!
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/



/*!
    * \brief  分布式一致性后半部分实现， 前半部分模板实现在.h文件中
    */

/*@
* 函数功能： 实例化基础参数 包括内部迭代次数 收敛因子
*    输入： 无
*    输出： 无
*/
/************************************************************************/
C_DistrInfoConcencus::C_DistrInfoConcencus()
{
    iterate_times = 10;//内部循环次数为10
    gama = 0.01; //收敛因子暂定0.01
}
/************************************************************************/
/************************************************************************/


/*@
* 函数功能： 设置机器人代号
*    输入： 机器人id号
*    输出： 无
*/
/************************************************************************/
void C_DistrInfoConcencus::setSelfId(int _self_id)
{
    self_id = _self_id;
    cout << "this robot id is: "<< self_id <<endl;
}
/************************************************************************/
/************************************************************************/



/*@
* 函数功能： 设置收敛速度
*    输入： 收敛速度， 为整数类型
*    输出： 无
*/
/************************************************************************/
void C_DistrInfoConcencus::setConvSpeed(int _squad_conv_speed)
{
    squad_conv_speed = _squad_conv_speed;
    gama = 0.01 * squad_conv_speed;
    cout << "the conv speed is: "<< squad_conv_speed<< endl;
}
/************************************************************************/
/************************************************************************/

vector<int> C_DistrInfoConcencus::getNeededId(void)
{
    int _count = 0;                 //下标的计数变量
    MatrixXf _middle_matrix(1,1);
    _middle_matrix << 0;            // 被用来比较的中间变量
    MatrixXf _minus_laplace_vec;    // 提取拉普拉斯向量的中间变量

    _minus_laplace_vec = minus_laplace.row(self_id - 1);

    cout<< "minus_laplace_vec: "<<_minus_laplace_vec<<endl;
    for(int i=0; i<_minus_laplace_vec.cols();i++)
    {

        _count += 1;  //先+1 机器人号从1开始
        if(_minus_laplace_vec.col(i) != _middle_matrix )
        {
            needed_id.push_back(_count);
        }

    }
    cout<< "the needed id is: "<< endl;
    for(int i=0; i<needed_id.size();i++)
    {
        cout<< "id: " <<  needed_id[i]  <<endl;
    }

    return needed_id;
}

/*!
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/





// 一阶一致性 期望速度输出版本 具体实现

C_SingleIntegConcensusFormation::C_SingleIntegConcensusFormation()
{
    cout <<"initial" <<endl;
}
C_SingleIntegConcensusFormation::C_SingleIntegConcensusFormation(int _self_id, int _robot_num, int _state_num)
{
    self_id = _self_id;
    robot_num = _robot_num;
    state_num = _state_num;

    laplace_n = MatrixXf(robot_num, robot_num);
    laplace_n_1 = MatrixXf(robot_num+1, robot_num+1);

    adjacent_n = MatrixXf(robot_num, robot_num);
    adjacent_n_1 = MatrixXf(robot_num+1, robot_num+1);


    cout << "the robot self id is: "<<  self_id  <<endl;
    cout << "the number of robots in formation is: " << robot_num <<endl;
    cout << "the dimention of this state is: "       << state_num <<endl;
}
C_SingleIntegConcensusFormation::~C_SingleIntegConcensusFormation()
{

}

void C_SingleIntegConcensusFormation::setLaplaceMatrix(MatrixXf _laplace_n, MatrixXf _laplace_n_1)
{
    laplace_n = _laplace_n;
    laplace_n_1 = _laplace_n_1;

    cout << "the n dimension laplace matrix is: " << endl << laplace_n  <<endl;
    cout << "the n+1 demension laplace matrix is: "<< endl << laplace_n_1 <<endl;

    // 将拉普拉斯矩阵转换成邻接矩阵
    adjacent_n = -laplace_n;  // 取相反数
    for(int i=0; i<robot_num; i++)
    {// 将主对角元素清0
        adjacent_n(i,i) = 0;
    }
    adjacent_n_1 = -laplace_n_1;// 取相反数
    for(int i=0; i<=robot_num; i++)
    {// 将主对角元素清0
        adjacent_n_1(i,i) = 0;
    }
    cout << "the n dimension adjacent matrix is: " <<endl<< adjacent_n  <<endl;
    cout << "the n+1 demension adjacent matrix is: "<<endl<< adjacent_n_1 <<endl;
}

void C_SingleIntegConcensusFormation::setGamma(float _gamma)
{
    gamma = _gamma;
    cout << "gamma: " << gamma <<endl;
}

VectorXf C_SingleIntegConcensusFormation::calFormationMaintainOutput(VectorXf _ref_pos, MatrixXf _devia_pos, MatrixXf _cur_pos)
{
    ref_pos = _ref_pos;
    devia_pos = _devia_pos;
    cur_pos = _cur_pos;

    // 参考位置 偏差 以及当前位置都要转置
    ref_pos_T = ref_pos.transpose();
    devia_pos_T = devia_pos.transpose();
    cur_pos_T = cur_pos.transpose();

    // 计算输出 对应行的拉普拉斯矩阵 乘以 当前位置和偏移位置误差
    // 再减掉n+1维拉普拉斯矩阵最后一列的元素与相应的误差相乘
    // 结果是一个行向量
    // 取出第i行，最后一列的那个数
    float __a_i_n_1 = adjacent_n_1(self_id - 1, robot_num);

    output_T = - laplace_n.row(self_id - 1)*( cur_pos_T - devia_pos_T )
               - __a_i_n_1*( cur_pos_T.row( self_id - 1 ) - devia_pos_T.row(self_id - 1) - 2*ref_pos_T );

    // 将输出转置 该输出为期望速度，只要跟踪上这个期望速度，就可以实现编队保持
    output = output_T.transpose();
    cout << self_id <<"-th formation maintainous expect speed is:"<<endl << output <<endl;

    return output;
}

VectorXf C_SingleIntegConcensusFormation::calFormationTrackingOutput(VectorXf _ref_pos, VectorXf _ref_speed,
                                                                     MatrixXf _devia_pos, MatrixXf _devia_speed,
                                                                     MatrixXf _cur_pos)
{
    ref_pos = _ref_pos;
    ref_speed = _ref_speed;
    devia_pos = _devia_pos;
    devia_speed = _devia_speed;
    cur_pos = _cur_pos;

    // 参考位置速度， 偏差位置速度， 当前位置全部转置
    ref_pos_T = ref_pos.transpose();
    ref_speed_T = ref_speed.transpose();
    devia_pos_T = devia_pos.transpose();
    devia_speed_T = devia_speed.transpose();
    cur_pos_T = cur_pos.transpose();

    // 计算输出
    output_T = ref_speed_T + devia_speed_T.row(self_id - 1)
             - gamma*( cur_pos_T.row(self_id - 1)-( ref_pos_T + devia_pos_T.row(self_id - 1)) )
             - laplace_n.row(self_id - 1) * (  cur_pos_T - devia_pos_T );

    // 这个输出本质上是期望速度
    output = output_T.transpose();

    return output;
}


VectorXf C_SingleIntegConcensusFormation::calFormationTrackingOutput_v2(VectorXf _ref_pos, VectorXf _ref_speed,
                                                                        MatrixXf _devia_pos, MatrixXf _devia_speed,
                                                                        MatrixXf _cur_pos, MatrixXf _cur_speed)
{
    ref_pos = _ref_pos;
    ref_speed = _ref_speed;
    devia_pos = _devia_pos;
    devia_speed = _devia_speed;
    cur_pos = _cur_pos;
    cur_speed = _cur_speed;

    float __eta = 0; // eta是i行i列的对角项
    __eta = laplace_n_1(self_id - 1, self_id - 1);

    // 所有矩阵向量，全部转置
    ref_pos_T = ref_pos.transpose();
    ref_speed_T = ref_speed.transpose();
    devia_pos_T = devia_pos.transpose();
    devia_speed_T = devia_speed.transpose();
    cur_pos_T = cur_pos.transpose();
    cur_speed_T = cur_speed.transpose();

    float __a_i_n_1 = adjacent_n_1(self_id - 1, robot_num);

    // 开始计算输出
    output_T = devia_speed_T.row(self_id - 1)
             + 1/__eta*(  adjacent_n.row(self_id - 1)*( cur_speed_T - devia_speed_T )
                          - gamma*laplace_n.row(self_id - 1)*( cur_pos_T - devia_pos_T ) );
             + 1/__eta* __a_i_n_1*( ref_speed_T - gamma*( cur_pos_T.row(self_id-1) - devia_pos_T.row(self_id-1) - ref_pos_T ));

    // 转置成为向量
    output = output_T.transpose();
    return output;

}
















}
