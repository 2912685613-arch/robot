
// #include "stereocamera.h"
#include "../../include/asrobot/stereocamera.h"
//hcy:elas算法
// #include <time.h>
// #include "../../include/asrobot/elas.h"
// #include "../../include/asrobot/image.h"

StereoCamera::StereoCamera()
{

}

StereoCamera::StereoCamera(Mat cML, Mat dCL, Mat cMR, Mat, Mat dCR, Mat R, Mat T)
{
    this->cameraMatrixL = cML;
    this->cameraMatrixR = cMR;
    this->distCoeffL = dCL;
    this->distCoeffR = dCR;
    this->R = R;
    this->T = T;
}

StereoCamera::~StereoCamera()
{

}

void StereoCamera::loadParameters()
{
//    this->cameraMatrixL = cML;
//    this->cameraMatrixR = cMR;
//    this->distCoeffL = dCL;
//    this->distCoeffR = dCR;
//    this->R = R;
//    this->T = T;
}

void StereoCamera::loadParameters(string file)
{

}

void StereoCamera::drawBox(Mat image, Rect box, Scalar color, int thick)
{
    rectangle(image,Point(box.x,box.y),Point(box.x+box.width, box.y+box.height),color,thick);

}

void StereoCamera::imagePyrDown(Mat imageL, Mat imageR)
{
    pyrDown(imageL, imageL_down, Size(imageL.cols/2,imageL.rows/2));
//    pyrDown(imageL_d, imageL_down, Size(imageL_d.cols/2,imageL_d.rows/2));
    pyrDown(imageR, imageR_down, Size(imageR.cols/2,imageR.rows/2));
//    pyrDown(imageR_d, imageR_down, Size(imageL_d.cols/2,imageL_d.rows/2));
//    imshow("imageL_down", imageL_down);
//    imshow("imageR_down", imageR_down);
}

void StereoCamera::undistortRectifyMap(Mat cameraMatrixL, Mat distCoeffL, Mat cameraMatrixR, Mat distCoeffR ,Mat T, Mat R)
{
    imageSize = imageL_down.size();
    //hy:得到校正旋转矩阵R、投影矩阵P、重投影矩阵Q
    //      输出： Rl, Rr, Pl, Pr, Q
    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q,
                  CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);
    //hy:得出校准映射参数    
    //      输出： mapLx, mapLy，mapRx, mapRy         
    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
}

/*
//hcy:基于elas算法
void StereoCamera::stereoMatch_elas(Mat &rgbImageL, Mat &rgbImageR)
{
    cvtColor(rgbImageL, grayImageL, COLOR_BGR2GRAY);
    cvtColor(rgbImageR, grayImageR, COLOR_BGR2GRAY);
    //hy:重映射
    //      输出：rectifyImageL，rectifyImageR
    remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
    remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

	// 参数设置	
	Elas::parameters param;
	param.disp_min = 0;                                     // 最小视差	
	param.disp_max = 256;                                // 最大视差	
	param.support_threshold = 0.85;            // 比率测试：最低match VS 次低match  //uniquenessRatio   //重要   
	param.support_texture = 10;                     // 支持点的最小纹理   
	param.candidate_stepsize = 5;                  // 用于支持点的sobel特征匹配的邻域半径            
	param.incon_window_size = 5;                  // 不连续性窗口的尺寸
	param.incon_threshold = 5;                       // 不连续性窗口内的视差范围阈值    
	param.incon_min_support = 5;                 // 不连续性窗口内的最低支持点数量
	param.add_corners = true;                        // 是否添加角点
	param.grid_size = 20;                                  // 网格尺寸
	param.beta = 0.02;                                      // 图像相似性度量的参数
	param.gamma = 3;                                      // 先验概率常数
	param.sigma = 1;                                         // 先验概率的标准差
	param.sradius = 3;                                       // 标准差半径
	param.match_texture = 1;                         // 最低纹理
	param.lr_threshold = 1;                             // 左右一致性检验阈值    //重要
	param.speckle_sim_threshold = 1;          // 连通域判断阈值     //重要
	param.speckle_size = 200;                        // 连通域噪声尺寸判断阈值
	param.ipol_gap_width = 3;                       // 空洞宽      //重要
	param.filter_median = false;                     // 是否中值滤波
	param.filter_adaptive_mean = true;        // 是否自适应中值滤波
	param.postprocess_only_left = true;     // 是否只对左视差图后处理，设置为True可以节省时间
	param.subsampling = false;                     // 每个两个像素进行视差计算，设置为True可以节省时间，但是传入的D1和D2的分辨率必须为(w/2) x (h/2)


	int width = rectifyImageL.cols;  //宽度为图像纵列//640
	int height = rectifyImageL.rows; //长度为图像行列//480
	int dim[3] = { width, height, width };
	//cout<<width<<" "<<height<<" "<<dim[3]<<endl;

    Mat disp, disp8;

	Mat disp_left = Mat::zeros(Size(width, height), CV_32FC1); //初始化左右视差图
	Mat disp_right = Mat::zeros(Size(width, height), CV_32FC1);


	clock_t start = clock();
	Elas elas(param);
	elas.process(rectifyImageL.data, rectifyImageR.data, disp_left.ptr<float>(0), disp_right.ptr<float>(0), dim);
	clock_t end = clock();
	//cout << "running time：" << (double)(1000 * (end - start) / CLOCKS_PER_SEC) << endl;
 
	disp_left = disp_left * 3;
	disp_right = disp_right * 3;

    reprojectImageTo3D(disp_left, xyz, Q, true);
    xyz = xyz * 16;

    imshow("disparity", disp_left);	// show disparity

    // disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities * 16.));
    // reprojectImageTo3D(disp, xyz, Q, true);
    // xyz = xyz * 16;
    // imshow("disparity", disp8);	// show disparity

}
*/



//hy:基于bm算法
void StereoCamera::stereoMatch(Mat &rgbImageL, Mat &rgbImageR)
{
    cout<<"wwwwwwwww"<<endl;
    //hcy
    int SADWindowSize =  (2*blockSize+5);
    int numberOfDisparities = (numDisparities*16 + 16);
    
    Ptr<StereoBM> bm = StereoBM::create(16,9);

    cvtColor(rgbImageL, grayImageL, COLOR_BGR2GRAY);
    cvtColor(rgbImageR, grayImageR, COLOR_BGR2GRAY);
    //hy:重映射
    //      输出：rectifyImageL，rectifyImageR
    cout<<"wyyyyyyy"<<endl;

    remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
    remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
    cout<<"iiiiiiiiooooooooo"<<endl;

    bm->setROI1(validROIL);
    bm->setROI2(validROIR);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);

    Mat disp, disp8;
    bm->compute(rectifyImageL, rectifyImageR, disp);

    disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities * 16.));
    reprojectImageTo3D(disp, xyz, Q, true);
    xyz = xyz * 16;
    imshow("disparity", disp8);	// show disparity

/*
    cv::StereoBM bm;
    cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
    cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);
    remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
    remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
    bm.state->SADWindowSize = (2*blockSize+5);
    bm.state->roi1=validROIL;
    bm.state->roi2=validROIR;
    bm.state->preFilterCap = 31;
    bm.state->minDisparity = 0;
    bm.state->numberOfDisparities = (numDisparities*16 + 16);
    bm.state->textureThreshold = 10;
    bm.state->uniquenessRatio = 15;
    bm.state->speckleWindowSize = 100;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1;

    Mat img1border, img2border;

    copyMakeBorder(rectifyImageL, img1border, 0, 0, bm.state->numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    copyMakeBorder(rectifyImageR, img2border, 0, 0, bm.state->numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    Mat disp, disp8;
    bm(rectifyImageL, rectifyImageR, disp);

    disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));
    reprojectImageTo3D(disp, xyz, Q, true);
    xyz = xyz * 16;

   imshow("disparity", disp8);	// show disparity
*/
}

Mat StereoCamera::get3DPosition()
{

}

