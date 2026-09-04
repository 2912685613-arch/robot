#include "../../include/asrobot/F_FourLedsSolvePnP.h"

#define FOUR_LED_PRINT_SWITCH 0
#define MARKER_PRINT_SWITCH 0

#define FOUR_LED_PRINT_DEBUG 0


//hcy
static Vec3f rotationMatrixToEulerAngles(Mat &R)
//Mat rotationMatrixToEulerAngles(Mat &R)
{
    // Mat posture2;

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) + R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    double x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    // posture2.at<double>(0,0) = x;
    // posture2.at<double>(1,0) = y;
    // posture2.at<double>(2,0) = z;
    //return Vec3f(x, y, z);
    return Vec3f(x, y, z)*100/3.14159;
    //return posture2;
}



/****************************************/
//*@ find the center point of the circle
//** input: the circle contours
//** output: the center_point
Point FindCenterPoint(vector<vector<Point>> contourss, int index)
{
    int min_x,min_y,max_x,max_y;
    Point min_x_point, min_y_point, max_x_point, max_y_point;
    int min_i,max_i;
    Point center_point;//º¯ÊýÊä³öµÄÖÐÐÄµê
    //Ñ­»·±ÈœÏ Êä³öÂÖÀªÖÐx×îÐ¡µÄÄÇžöµã
    min_x = contourss[index][0].x;
    min_i = 0;
    for(int i=0; i<contourss[index].size(); i++ )
    {
        if( contourss[index][i].x < min_x  )
        {
            min_x = contourss[index][i].x; //œ«×îÐ¡ÖµžüÐÂ
            min_i = i; //œ«ÕâžöµãµÄÐòºÅŒÇÂŒÏÂÀŽ
        }
    }
    min_x_point = contourss[index][min_i];//Ñ­»·œáÊøºó£¬œ«x×îÐ¡µÄÄÇžöµãžøÏàÓŠµÄ±äÁ¿

    //Ñ­»·±ÈœÏ Êä³öÂÖÀªÖÐy×îÐ¡µÄÄÇžöµã
    min_y = contourss[index][0].y;
    min_i = 0;
    for(int i=0; i<contourss[index].size(); i++ )
    {
        if( contourss[index][i].y < min_y  )
        {
            min_y = contourss[index][i].y; //œ«×îÐ¡ÖµžüÐÂ
            min_i = i; //œ«ÕâžöµãµÄÐòºÅŒÇÂŒÏÂÀŽ
        }
    }
    min_y_point = contourss[index][min_i];//Ñ­»·œáÊøºó£¬œ«y×îÐ¡µÄÄÇžöµãžøÏàÓŠµÄ±äÁ¿


    //Ñ­»·±ÈœÏ Êä³öÂÖÀªÖÐx×îŽóµÄÄÇžöµã
    max_x = contourss[index][0].x;
    max_i = 0;
    for(int i=0; i<contourss[index].size(); i++ )
    {
        if( contourss[index][i].x > max_x  )
        {
            max_x = contourss[index][i].x; //œ«×îŽóÖµžüÐÂ
            max_i = i; //œ«ÕâžöµãµÄÐòºÅŒÇÂŒÏÂÀŽ
        }
    }
    max_x_point = contourss[index][max_i];//Ñ­»·œáÊøºó£¬œ«x×îŽóµÄÄÇžöµãžøÏàÓŠµÄ±äÁ¿

    //Ñ­»·±ÈœÏ Êä³öÂÖÀªÖÐy×îŽóµÄÄÇžöµã
    max_y = contourss[index][0].y;
    max_i = 0;
    for(int i=0; i<contourss[index].size(); i++ )
    {
        if( contourss[index][i].y > max_y  )
        {
            max_y = contourss[index][i].y; //œ«×îŽóÖµžüÐÂ
            max_i = i; //œ«ÕâžöµãµÄÐòºÅŒÇÂŒÏÂÀŽ
        }
    }
    max_y_point = contourss[index][max_i];//Ñ­»·œáÊøºó£¬œ«x×îŽóµÄÄÇžöµãžøÏàÓŠµÄ±äÁ¿

    //ÏÖÔÚ×î×ó£¬×îÓÒ£¬×îÉÏ£¬×îÏÂµÄµãÒÑŸ­±»ÕÒµœ£¬ÕâÊ±¿ÉÒÔÈ¡ÆœŸùÀŽŒÆËãÖÐÐÄµãµÄ×ø±ê
    center_point.x = (min_x_point.x + max_x_point.x)/2;
    center_point.y = (min_y_point.y + max_y_point.y)/2; //×¢Òâ£¬ÕâÀïµÄx£¬yÊÇÏñËØ×ø±ê

    return center_point;
}
/***********************************************/



/*************************************************/
//*@ find the four key points we need the four points to locate the asrobot
//** input: a lot of points
//** output: the four key points, store in a vector

vector<Point> delete_mark_point; //È¥µôË®Ó¡ºóÊ£ÏÂµÄµã·ÅÔÚÕâžöÏòÁ¿
vector<Point> point_four; //ËÄµãÏòÁ¿
Point max_x_y_point; //Ë®Ó¡µã£¬ŸßÓÐ×îŽóµÄxºÍy¡£
int max_i; //Ë®Ó¡µã¶ÔÓŠµÄx×îŽóÏÂ±ê
int max_j;//Ë®Ó¡µã¶ÔÓŠµÄy×îŽóÏÂ±ê

int FindFourPoints(vector<Point> many_points, vector<Point>* four_points)
{
    if(many_points.size() == 0)//hy:如果“多个点”的数量是0，返回0
    {
        return 0;
    }

//    //µÚÒ»²œ È¥Ë®Ó¡ È¥µôÏñËØx£¬y×îŽóµÄÄÇžöµã
//    //ÏÈÕÒy
//    max_x_y_point = many_points[0];
//    max_j = 0;
//    for(int j=0 ; j< many_points.size();j++)
//    {
//        if(many_points[j].y > max_x_y_point.y)
//        {
//            max_x_y_point = many_points[j];
//            max_j = j;
//        }
//    }
    //³ýÁËÕâžöÏÂ±êµÄµã£¬ÆäÓàµÄÈ«²¿Ñ¹ÈëÏòÁ¿ÖÐ
    for(int j=0 ; j< many_points.size();j++)
    {
//        if( j!= max_j )
//        {
            delete_mark_point.push_back(many_points[j]);//hy:把所有“多个点”放进“删除标记点”数组里
//        }
    }

    if(delete_mark_point.size() >=4 )//hy:如果“删除标记点”的个数大于等于4
    {
        for(int i = 0; i< delete_mark_point.size(); i++)//hy:把所有“删除标记点”按y从大到小排列
        {
            for(int j = 0; j<delete_mark_point.size()-1-i; j++)
            {
                //if(delete_mark_point[j].y < delete_mark_point[j+1].y)//hy:如果一个点的y比下一个点的y小
                //                                                                                                                //(即这个点如果在上方)
                if(delete_mark_point[j].y > delete_mark_point[j+1].y)//zy
                {
                    Point temp = delete_mark_point[j];//hy:交换j和j+1两个点在数组里的位置
                    delete_mark_point[j] = delete_mark_point[j+1];
                    delete_mark_point[j+1] = temp;
                }
            }
        }

        for(int i=0 ; i<4 ;i++)
        {
            point_four.push_back(delete_mark_point[i]);//hy:把“删除标记点”的前4个放进“点四”里。
                                                                                                            //(即取出所有“删除标记点”中最下方的4个点)
        }
    }
    else//hy:如果“删除标记点”的个数小于4
    {
        point_four.clear();//hy:把“点四”清空
    }

    (*four_points).assign(point_four.begin(), point_four.end());//hy:把“点四”中的点赋值到“四点”中，

    point_four.clear();//hy:把“点四”清空
    delete_mark_point.clear();//hy:把“删除标记点”清空



    if((*four_points).size() == 4)//hy:如果“四点”的数量是4，返回4，否则返回0
    {
        return 4;
    }
    else
    {
        return 0;
    }
}
/******************************************************/



/****************************************************/
//*@ according to the led picture to locate the asrobot
//** input: source camera picture, four object points, camera matrix, dist coefficients
//** output: posture, position
//** if return value equals to 1, it represents that find location successfully

vector<vector<Point>> origin_contours, round_contours;//Ô­ÊŒÂÖÀª£¬ÉžÑ¡ºóµÄÔ²ÐÎÂÖÀª
vector<Vec4i> hierarchy;//ÂÖÀª²ãŒ¶

vector<Point> multi_points;//Ê¶±ðµœµÄ¶àžö¹ØŒüµã
vector<Point> four_points1;//×îÖÕÒªÓÃµÄËÄžö¹ØŒüµã
vector<Point2f> img_pnts; //ÏñËØ×ø±ê  ÕâžöÃ¿ŽÎÑ­»·Íê³ÉºóÇå0

vector<Point3f> obj_ref_pnts; //ÏñËØ×ø±ê  ÕâžöÃ¿ŽÎÑ­»·Íê³ÉºóÇå0
vector<Point2f> img_ref_pnts; //ÏñËØ×ø±ê  ÕâžöÃ¿ŽÎÑ­»·Íê³ÉºóÇå0

Point point_num_1;
Point point_num_2;
Point point_num_3;
Point point_num_4;


Mat origin_pic_clone;
Mat gray_pic;
Mat gauss_pic;
Mat threshold_pic;
Mat hsv_pic;

int hmin = 5, smin = 116, vmin = 7;//hcy
int hmax = 66, smax = 254, vmax = 241;

int FourLedLocation(Mat origin_pic,vector<Point3f> obj_pnts, Mat cam_mat, Mat dist_mat, Mat *posture, Mat *position)
//int FourLedLocation(Mat origin_pic,vector<Point3f> obj_pnts, Mat cam_mat, Mat dist_mat, Vec3f *angles, Mat *position)//hcy
{
    cvtColor(origin_pic, hsv_pic,COLOR_BGR2HSV);//hcy

    GaussianBlur(hsv_pic, gauss_pic, cv::Size(5, 5), 3, 3);//hcy

    Scalar  lower(hmin, smin, vmin);//hcy
    Scalar  upper(hmax, smax, vmax);//hcy

    inRange(gauss_pic, lower, upper, threshold_pic);//hcy

    // namedWindow("threshold_pic", WINDOW_NORMAL);//hcy
    // imshow("threshold_pic", threshold_pic);
/*
    //cvtColor(origin_pic, gray_pic,CV_BGR2GRAY);
    cvtColor(origin_pic, gray_pic,COLOR_BGR2GRAY);//hcy

    GaussianBlur(gray_pic, gauss_pic, cv::Size(5, 5), 3, 3);

    //threshold( gauss_pic,threshold_pic,230,255, CV_THRESH_BINARY);
    threshold( gauss_pic,threshold_pic,230,255, THRESH_BINARY);//hcy
*/

    //findContours(threshold_pic, origin_contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_NONE, Point(0,0));
    findContours(threshold_pic, origin_contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE, Point(0,0));//hcy
    
    // 找轮廓
    for(int i=0 ; i< origin_contours.size() ; i++)
    {
        // if((arcLength(origin_contours[i],true)> 40.0) && ( arcLength(origin_contours[i],true)<300.0 ))
        if((arcLength(origin_contours[i],true)> 0) && ( arcLength(origin_contours[i],true)<200.0 ))
        {
            round_contours.push_back(origin_contours[i]);//hy:把轮廓长度在一定范围内的图形放进“圆形轮廓”里
        }
#if FOUR_LED_PRINT_SWITCH
        printf("arclength: %f\n", arcLength(origin_contours[i],true) );//hy:打印二值化处理后所有
                                                                                                                                        //图形“原始轮廓”的轮廓长度
#endif
    }

    // 找到圆形轮廓的中心点
    for(int i=0; i< round_contours.size(); i++)
    {
        multi_points.push_back(FindCenterPoint(round_contours, i));//hy:找到“圆形轮廓”的中心点
                                                                                                                                          //并放在“多个点”里
    }
#if FOUR_LED_PRINT_SWITCH
    printf("round multi_points size: %d\n",multi_points.size() );//hy:打印“多个点”的数量
    printf("find center point successfully\n");
#endif


    if(FindFourPoints(multi_points,&four_points1) == 4)//hy:如果“多个点”经过找点函数返回的“四个点”的数量为4
    {
        printf("find four points successfully\n");
    }
    else
    {
        printf("find four points failed\n");
        return 0;
    }

    // 查看实际找到了几个点
#if FOUR_LED_PRINT_SWITCH
    printf("four_points1.size(): %d\n", four_points1.size());//hy:打印经过找点函数返回的“四个点”的数量
#endif

    // 对点重新排序
    if(four_points1[0].x < four_points1[1].x)
    {
        //point_num_3 = four_points1[0];
        //point_num_4 = four_points1[1];
        point_num_1 = four_points1[0];   //hcy
        point_num_2 = four_points1[1];
    }
    else
    {
        //point_num_4 = four_points1[0];
        //point_num_3 = four_points1[1];
        point_num_2 = four_points1[0];
        point_num_1 = four_points1[1];
    }

    if(four_points1[2].x < four_points1[3].x)
    {
        //point_num_1 = four_points1[2];
        //point_num_2 = four_points1[3];
        point_num_3 = four_points1[2];
        point_num_4 = four_points1[3];
    }
    else
    {
        //point_num_2 = four_points1[2];
        //point_num_1 = four_points1[3];
        point_num_4 = four_points1[2];
        point_num_3 = four_points1[3];
    }


    origin_pic_clone = origin_pic.clone();//hy:把所有“原始图像”克隆
    for(int i=0 ; i<round_contours.size(); i++)
    {
#if FOUR_LED_PRINT_SWITCH
        // 看看轮廓有多大
        cout<<arcLength(round_contours[i], true)<<endl;//hy:打印所有“圆形轮廓”(即
                                                                                                            //经过轮廓长度检测后的轮廓)的轮廓长度
        cout<<round_contours.size()<<endl;//打印所有“圆形轮廓”的数量
#endif
        // 画下来
        drawContours(origin_pic_clone,round_contours,i,CV_RGB(255,0,0),-1);//hy:把“圆形轮廓”画在“克隆图像”上
    }
    // 再用线连起来
    line(origin_pic_clone , point_num_1, point_num_2, Scalar(0,0,255),5);
    line(origin_pic_clone , point_num_2, point_num_3, Scalar(0,0,255),5);
    line(origin_pic_clone , point_num_3, point_num_4, Scalar(0,0,255),5);
    line(origin_pic_clone , point_num_4, point_num_1, Scalar(0,0,255),5);

    // 将点写死做测试

#if FOUR_LED_PRINT_DEBUG
    int formal_scalar = 10;
    point_num_1.x = point_num_1.x - point_num_1.x % formal_scalar;
    point_num_1.y = point_num_1.y - point_num_1.y % formal_scalar;
    point_num_2.x = point_num_2.x - point_num_2.x % formal_scalar;
    point_num_2.y = point_num_2.y - point_num_2.y % formal_scalar;
    point_num_3.x = point_num_3.x - point_num_3.x % formal_scalar;
    point_num_3.y = point_num_3.y - point_num_3.y % formal_scalar;
    point_num_4.x = point_num_4.x - point_num_4.x % formal_scalar;
    point_num_4.y = point_num_4.y - point_num_4.y % formal_scalar;
#endif


    // 压四个像素点
    img_pnts.push_back(point_num_1);
    img_pnts.push_back(point_num_2);
    img_pnts.push_back(point_num_3);
    img_pnts.push_back(point_num_4);

#if FOUR_LED_PRINT_SWITCH
    cout<< "point_num_1"<< point_num_1 << endl;
    cout<< "point_num_2"<< point_num_2 << endl;
    cout<< "point_num_3"<< point_num_3 << endl;
    cout<< "point_num_4"<< point_num_4 << endl;
#endif

    Mat Rvec(3,1, CV_64FC1), Tvec(3,1, CV_64FC1);
    Mat RMatrix(3,3, CV_64FC1), TMatrix(3,3, CV_64FC1);
    Mat RMatrixInv(3,3, CV_64FC1);

    solvePnP(obj_pnts,img_pnts,cam_mat,dist_mat, Rvec,Tvec);

    Mat unit_vec = (Mat_<double>(3,1) << 0,0,1);

    //Mat posture1;
    Mat posture1(3,1, CV_64FC1);//hcy
    Mat position1;

    Rodrigues(Rvec, RMatrix);//hy:将旋转向量变成旋转矩阵
    //transpose(Tvec, TMatrix);
    RMatrixInv = RMatrix.inv();//hy:将旋转矩阵取逆矩阵
    //posture1 = RMatrix.inv() * unit_vec;

    //Rodrigues(RMatrixInv, posture1);//hy:欧拉角向量的计算：将旋转矩阵的逆矩阵变成欧拉角向量

    //Vec3f angles = rotationMatrixToEulerAngles(RMatrix);//hcy:旋转矩阵转欧拉角向量
    //posture1 = rotationMatrixToEulerAngles(RMatrix);//hcy:旋转矩阵转欧拉角向量
    //float sy = sqrt(RMatrix.at<double>(0,0) * RMatrix.at<double>(0,0) + RMatrix.at<double>(1,0) * RMatrix.at<double>(1,0) );
    double sy = sqrt(RMatrix.at<double>(2,1) * RMatrix.at<double>(2,1) + RMatrix.at<double>(2,2) * RMatrix.at<double>(2,2) );

    bool singular = sy < 1e-6; // If

    double x, y, z;
    if (!singular)
    {
        x = atan2(RMatrix.at<double>(2,1) , RMatrix.at<double>(2,2));
        y = atan2(-RMatrix.at<double>(2,0), sy);
        z = atan2(RMatrix.at<double>(1,0), RMatrix.at<double>(0,0));
    }
    else
    {
        x = atan2(-RMatrix.at<double>(1,2), RMatrix.at<double>(1,1));
        y = atan2(-RMatrix.at<double>(2,0), sy);
        z = 0;
    }
    posture1.at<double>(0,0) = z;//roll
    posture1.at<double>(1,0) = x;//pitch
    posture1.at<double>(2,0) = y;//yaw

    position1 = RMatrix.inv() *(-Tvec);//hy:位置向量的计算：通过公式推导

#if FOUR_LED_PRINT_SWITCH
    cout<< "Rvec = " << Rvec<<endl;
    cout<< "Tvec = " << Tvec<<endl;

    cout << "cam_posture"<< posture1 << endl;
    cout << "cam_position"<< position1 << endl;
#endif
    *posture = posture1;
    *position = position1;

    //namedWindow("origin_pic_clone", CV_WINDOW_NORMAL);
    namedWindow("origin_pic_clone", WINDOW_NORMAL);

    imshow("origin_pic_clone", origin_pic_clone);

    // 清向量
    origin_contours.clear();
    round_contours.clear();
    hierarchy.clear();
    four_points1.clear();
    multi_points.clear();
    img_pnts.clear();


    return 1;
}
/**************************************************/



/*************************************************/
//*@ use marker to locate the position and attitude
//** input: origin image, camera parameters, size of marker;
//** output: trans vector, rotate vector
//** if return value is 0, it means there is a marker ahead


//## some global parameters we need  default marker size is 0.1565
bool initial_flag = false;
MarkerDetector m_Detector;
CvDrawingUtils MDraw;
vector<Marker> the_Markers;
CameraParameters cam_param;

Mat cam_mat = (Mat_<float>(3,3) << 381.3625, 0, 320.5,
                     0       , 381.3625, 240.5,
                 0       ,       0,        1);

Mat dist_mat = (Mat_<float>(5,1) << 0, 0, 0, 0, 0);

Mat rot_vec;
Mat trans_vec;

// Mat axis_image;
int MarkerLocation(Mat input_image, float marker_size, Mat* rot_vec, Mat* trans_vec)
{
    // axis_image = input_image;
    if(initial_flag == false)
    {       
        // cam_param.readFromXMLFile("/home/tortoise/gazebo_catkin_ws/src/gazebo_ros_robot/asrobot/include/asrobot/camera_pmt/out_camera_calibration.yml");
        cam_param.CameraMatrix=cam_mat;//hcy
        cam_param.Distorsion=dist_mat;
        cam_param.CamSize.width=640;
        cam_param.CamSize.height=480;

        m_Detector.setDictionary("ALL_DICTS");
        m_Detector.getParameters().detectEnclosedMarkers(1);
        initial_flag = true;
        cout<<"now it has initialed yet"<<endl;
    
        return -1;

    }

    else
    {
        //aquire these markers
        the_Markers = m_Detector.detect(input_image, cam_param, marker_size);
        // the_Markers = m_Detector.detect(axis_image, cam_param, marker_size);//hcy


        if(the_Markers.size()!=0)// if marker is not empty
        {
            Mat Rvec, Tvec;
            Mat RMatrix, TMatrix;
            Mat RMatrixInv;
            
            Rvec = the_Markers[0].Rvec;
            Tvec = the_Markers[0].Tvec;

#if MARKER_PRINT_SWITCH
            cout<<"Rvec: "<<Rvec<<endl;
            cout<<"Tvec: "<<Tvec<<endl;
            cout<<"Rvec.size: "<<Rvec.size()<<endl;
#endif

            Mat unit_vec = (Mat_<float>(3,1) << 0,0,1);
            
            //Mat posture;
            Mat posture(3,1, CV_64FC1);//hcy
            Mat position;
            
           float ouler1,ouler2,ouler3;
          
            Rodrigues(Rvec, RMatrix);
            //transpose(Tvec, TMatrix);
            RMatrixInv = RMatrix.inv();

            //Rodrigues(RMatrixInv, posture);

            ///hcy
            float sy = sqrt(RMatrix.at<float>(2,1) * RMatrix.at<float>(2,1) + RMatrix.at<float>(2,2) * RMatrix.at<float>(2,2) );

            bool singular = sy < 1e-6; // If

            float x, y, z;
            if (!singular)
            {
                x = atan2(RMatrix.at<float>(2,1) , RMatrix.at<float>(2,2));
                y = atan2(-RMatrix.at<float>(2,0), sy);
                z = atan2(RMatrix.at<float>(1,0), RMatrix.at<float>(0,0));
            }
            else
            {
                x = atan2(-RMatrix.at<float>(1,2), RMatrix.at<float>(1,1));
                y = atan2(-RMatrix.at<float>(2,0), sy);
                z = 0;
            }
            posture.at<float>(0,0) = z;//roll
            posture.at<float>(1,0) = x;//pitch
            posture.at<float>(2,0) = y;//yaw
            ///hcy

            position = RMatrix.inv() *(-Tvec);
            
//           ouler1 = atan2( RMatrixInv.at<float>(2,1), RMatrixInv.at<float>(2,2) ) ;
//           ouler2 = atan2( - RMatrixInv.at<float>(2,0), sqrt( RMatrixInv.at<float>(2,1)*RMatrixInv.at<float>(2,1) + RMatrixInv.at<float>(2,2)* RMatrixInv.at<float>(2,2) )) ;
//           ouler3 = atan2( RMatrixInv.at<float>(1,0), RMatrixInv.at<float>(0,0) );

//           cout<< "ouler1 = " << ouler1*57.3<<endl;
//           cout<< "ouler2 = " << ouler2 *57.3<<endl;
//           cout<< "ouler3 = " << ouler3*57.3<<endl;

//           posture.at<float>(0,0) = ouler1;
//           posture.at<float>(1,0) = ouler2;
//           posture.at<float>(2,0) = ouler3;
            

            // cout << "cam_posture"<< posture << endl;
            // cout << "cam_position"<< position << endl;

            *rot_vec = posture;
            *trans_vec = position;
            cout<<"now it is detecting successfully" <<endl;

            // MDraw.draw3dAxis(input_image , cam_param, Rvec, Tvec, 0.2  );
            // MDraw.draw3dAxis(axis_image , cam_param, Rvec, Tvec, 0.2  );//hcy
            //namedWindow(" axis ",CV_WINDOW_NORMAL);
            namedWindow("axis",WINDOW_NORMAL);

            imshow("axis", input_image);
            // imshow("axis", axis_image);//hcy

            return 0;
        }
        else
        {
            cout<<"there is no marker has been detected"<<endl;
            return -2;// it means we have not detected the marker yet
        }
    }


}
/****************************************************************/


























//Ö÷³ÌÐò
/******************************************/
//@  ËÄµÆÊ¶±ð³ÌÐò  //
//****************//
