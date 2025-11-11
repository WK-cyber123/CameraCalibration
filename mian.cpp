#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include "Calibration.h"
// #include "unPhase.h"
// #include "CalibrateProj.h"
using namespace cv;
using namespace std;

// 标定板规格
#define BOARD_SCALE 15       // 圆心距15 mm 
#define BOARD_HEIGHT 9       // 标定板  圆的个数  行列数
#define BOARD_WIDTH 11    
	/*
	代码功能: 1.对相机和投影仪进行标定，并将标定结果保存至文件中
			  2.获得特征点对应的理想包裹相位与重投影误差

	代码流程：1.标定相机并获得特征点像素坐标
					读取图像->圆心检测->创建特征点在世界坐标系下的坐标->进行标定并保存结果
			  2.求解绝对相位
					读取图像->计算包裹相位->计算合成相位-> 对某一频率条纹相位展开
			  3.标定投影仪
					读取图像->计算特征点对应的投影仪像素->进行标定并保存结果

	使用方式: 需设置的变量包括: 标定板规格，存储图像路径，位姿数，采用条纹频率以及投影仪分辨率
	*/
int main()
{
	// 图像路径
	string camImageString = "E:/Code/C++_Code/gamma=3标定/gamma=3标定/camera/";    // 标定相机图片地址
	// string projImageString = "D:/halcon-image/论文2数据/gamma=3标定";   // 标定投影仪图片地址

	/*
	1.创建相机标定类
	2.CalibMain(图片地址，标定板宽度，标定板高度，标定板圆心距)  返回值为特征点的像素坐标
	*/
	Calibration_cam* camera = new Calibration_cam();
	vector<vector<cv::Point2f>> m_ImagePoint = camera->CalibMain(camImageString, BOARD_WIDTH, BOARD_HEIGHT, BOARD_SCALE);

	delete camera;

}

