#pragma once
#include<iostream>
#include <opencv2/calib3d.hpp>
#include<opencv2/opencv.hpp>
#include "CannyDevernay.h"

using namespace cv;
using namespace std;

class Calibration_cam
{
public:
	// 读取标定图像  ReadImages(文件路径)
	vector<Mat> ReadImages(const string imageFolderPath);
	// 检测圆心     Size  行列   和matlab 是反着的   即hight是行数   wight是列数  
	// FindCorners(图像,标定板尺寸)
	vector<vector<cv::Point2f>> FindCorners(const vector<Mat>& chessboard, cv::Size patternSize);

	std::vector<std::vector<cv::Point2f>> FindCorners_sub(const vector<Mat>& chessboard, cv::Size patternSize);
	// 角点绘制 未用
	vector<Mat> DrawCorners(vector<Mat>  chessboard, cv::Size patternSize, vector<vector<cv::Point2f>>findCorner);
	// 计算对应角点的世界坐标
	// CalworldCorner(存放世界坐标系, 标定板行数, 标定板列数, cv::Size 标定板尺寸)
	vector<vector<cv::Point3f>> CalworldCorner(vector<vector<cv::Point3f>>& m_WorldCorner, int lenth_cols, int lenth_rows, cv::Size patternSize);

	// 执行函数
	// CalibMain(string 文件路径, int 标定板宽, int 标定板高, int 圆心距)
	vector<vector<cv::Point2f>> CalibMain(string imageFolderPath, int BOARD_WIDTH, int BOARD_HEIGHT, int BOARD_SCALE);
	//储存标定参数
	/*
	SaveCalibResult：
					vector<Mat> image：存储图像的容器
					vector<vector<cv::Point3f>>worldcorner：特征点的世界坐标
					vector<vector<cv::Point2f>>point：特征点的像素坐标
					cv::Mat cameraMatrix1：内参矩阵
					cv::Mat distCoeffs1：畸变系数
					std::vector<cv::Mat> tvecsMat1：旋转向量
					std::vector<cv::Mat> rvecsMat1：平移向量

	*/
	void SaveCalibResult(vector<Mat> image, vector<vector<cv::Point3f>>worldcorner, vector<vector<cv::Point2f>>point, cv::Mat cameraMatrix1, cv::Mat distCoeffs1,
		std::vector<cv::Mat> tvecsMat1, std::vector<cv::Mat> rvecsMat1);



	//接受存放图片的容器
	vector<Mat> m_Image1;
	//接受世界坐标的矩阵(共同使用)
	vector<vector<cv::Point3f>>m_WorldCorner_1;

	// 相机参数   重建时使用  所以放入public
	cv::Mat cameraInMatrix;   //内参
	cv::Mat distortion;       // 畸变系数
	std::vector<cv::Mat> tMat; // 旋转向量
	std::vector<cv::Mat> rMat; //平移向量
private:


	vector<Mat>  m_Image;  //存放图片的容器
	vector<Mat> m_ShowCorn; //接受提取角点后的图片
	cv::Size m_ImageSize; //接收图片大小
	bool flag;
	vector<cv::String> m_ImageList;//图片路径
	cv::Size image_size;        //结构体中包含两个成员 wight 和height  
	//------------------------------------------
	vector<vector<cv::Point2f>>m_Point; //接受提取出来的角点，对应Mat矩阵
	vector<vector<cv::Point2f>>m_SubCorner;//亚像素角点，对应Mat矩阵
	//------------------------------------------
	vector<vector<cv::Point3f>>m_WorldCorner;//存放世界坐标的矩阵

	cv::Mat cameraMatrix; /* 摄像机内参数矩阵 */
	cv::Mat distCoeffs; /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	std::vector<cv::Mat> tvecsMat;  /* 每幅图像的旋转向量 */
	std::vector<cv::Mat> rvecsMat;  /* 每幅图像的平移向量 */

};
