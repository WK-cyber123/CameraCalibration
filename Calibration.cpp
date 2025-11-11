#include"Calibration.h"

#include <fstream>
#include <vector>
#include <algorithm>


void writePointsToTxt(const std::vector<std::vector<cv::Point2f>>& all_points, const std::string& filename) {
	std::ofstream outFile(filename);

	if (!outFile.is_open()) {
		std::cerr << "无法打开文件: " << filename << std::endl;
		return;
	}

	// 遍历所有曲线
	for (size_t i = 0; i < all_points.size(); ++i) {
		//outFile << "曲线 " << i << ":" << std::endl;

		// 遍历当前曲线的所有点
		//for (size_t j = 0; j < all_points[i].size(); ++j) {
		//	outFile << "  点 " << j << ": ("
		//		<< all_points[i][j].x << ", "
		//		<< all_points[i][j].y << ")" << std::endl;
		//}
		for (size_t j = 0; j < all_points[i].size(); ++j) {
			outFile << 
				 all_points[i][j].x << ", "<<
				 all_points[i][j].y << std::endl;
		}

	}
}
void SortRowCol(std::vector<cv::Point2f>& points, int cols, float y_threshold = 5.0f)
{
	// Step 1: 先按 y 升序
	std::sort(points.begin(), points.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
		return a.y < b.y;
		});

	// step2:
	for (int i = 0; i < 9; i++) {
		std::sort(points.begin()+i*11, points.begin()+(i+1)*11, [](const cv::Point2f& a, const cv::Point2f& b) {
			return a.x < b.x;
			});
	}


	// Step 2: 按行分组（每行内按 x 排序）
	//std::vector<std::vector<cv::Point2f>> row_groups;
	//std::vector<cv::Point2f> current_row;

	//current_row.push_back(points[0]);
	//for (size_t i = 1; i < points.size(); ++i) {
	//	if (std::fabs(points[i].y - points[i - 1].y) < y_threshold) {
	//		current_row.push_back(points[i]);
	//	}
	//	else {
	//		// 当前行满了，按 x 排序
	//		std::sort(current_row.begin(), current_row.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
	//			return a.x < b.x;
	//			});
	//		row_groups.push_back(current_row);
	//		current_row.clear();
	//		current_row.push_back(points[i]);
	//	}
	//}

	//// 最后一行也排序
	//std::sort(current_row.begin(), current_row.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
	//	return a.x < b.x;
	//	});
	//row_groups.push_back(current_row);

	//// Step 3: 按行合并结果
	//points.clear();
	//for (auto& row : row_groups) {
	//	// 如果检测行数不稳定，可以加保护：
	//	if (row.size() == cols) {
	//		points.insert(points.end(), row.begin(), row.end());
	//	}
	//	else {
	//		std::cerr << "Warning: one row has " << row.size() << " points (expected " << cols << ")\n";
	//		points.insert(points.end(), row.begin(), row.end());
	//	}
	//}
}


//   读取图片
vector<Mat> Calibration_cam::ReadImages(const string imageFloderPath)
{
	cv::glob(imageFloderPath, m_ImageList, false);
	if (m_ImageList.size() == 0)
	{
		cout << "没有读取到图片，请检查文件路径" << endl;
		exit(0);
	}

	int count = (int)m_ImageList.size();
	Mat image;
	for (int i = 0; i < count; i++)
	{
		// 以灰度图来读取图像
		image = imread(m_ImageList.at(i), IMREAD_GRAYSCALE);
		cout << "相机标定图像编号" << m_ImageList.at(i) << endl;
		// 转化为double类型  
		image.convertTo(image, CV_8UC1);
		m_Image.push_back(image);
	}

	cout << "读取完成" << endl;

	return m_Image;
}

vector<vector<cv::Point2f>> Calibration_cam::FindCorners(const vector<Mat>& chessboard, cv::Size patternSize)
{
	// 斑点检测
	SimpleBlobDetector::Params params;
	//输出提取的角点

	//params.minThreshold = 10;    // 阈值控制
	params.maxThreshold = 255;    // 亮度阈值
	params.maxArea = 10e4;    // 圆斑的最大面积  像素
	params.minArea = 25;

	//params.filterByInertia = true;
	params.filterByArea = true;     // 根据面积过滤
	params.blobColor = 255;   // 选择较亮的区域
	/*params.minDistBetweenBlobs = 5;
	params.filterByInertia = false;
	params.minInertiaRatio = 0.5;*/  // 惯性比  圆的惯性为1  

	Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
	// 存储角点
	std::vector<cv::Point2f> corners;
	std::vector<vector<cv::Point2f>> vector_corners;

	for (int i = 0; i < chessboard.size(); i++)
	{
		flag = false;
		bool flag = findCirclesGrid(chessboard.at(i), patternSize, corners, CALIB_CB_SYMMETRIC_GRID, blobDetector);  // 整像素

		// 亚像素边缘提取

		if (flag == true)
		{
			cout << "第" << i << "张图片角点提取成功" << endl;
			vector_corners.push_back(corners);
			//// 绘制边缘
			//cv::Mat tempImage = chessboard.at(i).clone();
			//drawChessboardCorners(tempImage, patternSize, Mat(corners), flag);
			//string  saveStr = "F:/毕业论文/图片/" + std::to_string(i) +".bmp";
			//cv::imwrite(saveStr, tempImage);
		}

	}
	std::string name = "center_picel.txt";
	writePointsToTxt(vector_corners, name);

	return  vector_corners; // 返回的角点

}


// 亚像素边缘检测
std::vector<std::vector<cv::Point2f>> Calibration_cam::FindCorners_sub(const vector<Mat>& chessboard, cv::Size patternSize) {

	std::cout << "边缘检测" << std::endl;
	int num_image = chessboard.size();

	std::vector<std::vector<cv::Point2f>> all_center;
	for (int i = 0; i < num_image; i++) {
		// 图像预处理
		
		cv::Mat image;

		//cv::adaptiveThreshold(chessboard.at(i), image, 255, ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 11, 2);

		double* x, * y;
		int N, M;                //N 边缘点数   M曲线数
		int* curve_limits;   // 曲线的起始
		double S = 2.0, H = 10.0, L = 5.0;
		chessboard.at(i).convertTo(image, CV_64FC1);
		int X = image.cols;
		int Y = image.rows;
		devernay(&x, &y, &N, &curve_limits, &M, image, X, Y, S, H, L);

		std::vector<std::vector<cv::Point2f>> all_point;
		std::vector<cv::Point2f> min_point;
		//  合并亚像素点坐标
		for (int i = 0; i < M; i++) {
			min_point.clear();
			for (int j = curve_limits[i]; j < curve_limits[i + 1];j++) {

				
				cv::Point2f p(x[j], y[j]);
				min_point.push_back(p);
			}


			// 筛选圆心  圆度
			auto minmax_x = std::minmax_element(
				min_point.begin(), min_point.end(),
				[](const cv::Point2f& a, const cv::Point2f& b) {
					return a.x < b.x;
				}
			);
			auto minmax_y = std::minmax_element(
				min_point.begin(), min_point.end(),
				[](const cv::Point2f& a, const cv::Point2f& b) {
					return a.y < b.y;
				}
			);


			if (std::abs(minmax_x.first->x - minmax_x.second->x) / std::abs(minmax_y.first->y - minmax_y.second->y)  < 1.2 &&
				std::abs(minmax_x.first->x - minmax_x.second->x) / std::abs(minmax_y.first->y - minmax_y.second->y)  > 0.8
				&& min_point.size() > 100) {
				all_point.push_back(min_point);
			
			}
	
		}

		std::string name = "sub_pixel_" + std::to_string(i) + ".txt";
		writePointsToTxt(all_point, name);

		std::vector<cv::Point2f> min_center;
		
		for (int i = 0; i < all_point.size(); i++) {
			// 拟合圆心
			cv::RotatedRect ellipse = cv::fitEllipse(all_point.at(i));
			min_center.push_back(ellipse.center);
			// ellipse.center 为椭圆中心
			// ellipse.size.width / height 为长轴/短轴长度
			// ellipse.angle 为旋转角度（与 x 轴夹角）
			//std::cout << "Center: " << ellipse.center << std::endl;
			//std::cout << "Axes: " << ellipse.size.width / 2 << ", " << ellipse.size.height / 2 << std::endl;
			//std::cout << "Angle: " << ellipse.angle << std::endl;
		}

		// 排序

		SortRowCol(min_center, 11);


		all_center.push_back(min_center);
		min_center.clear();
	}


	std::string name = "center_sub.txt";
	writePointsToTxt(all_center, name);



	
	return all_center;
}




// 画出角点     N帧图像
vector<Mat> Calibration_cam::DrawCorners(vector<Mat> chessboard, cv::Size patternSize, vector<vector<cv::Point2f>> findCorner)
{
	for (int i = 0; i < chessboard.size(); i++)
	{

		cv::drawChessboardCorners(chessboard.at(i), patternSize, findCorner.at(i), flag);

		//cv::Mat draw;
		//cv::normalize(chessboard.at(i), draw,0,1,cv::NORMAL_CLONE);
		//cv::imshow("chessboard", draw);
		//cv::waitKey(0);

	}

	return chessboard;
}

// 提取世界坐标
vector<vector<cv::Point3f>>Calibration_cam::CalworldCorner(vector<vector<cv::Point3f>>& m_WorldCorner, int lenth_rows, int lenth_cols, cv::Size patternSize)
{
	cv::Size square_size = cv::Size(lenth_cols, lenth_rows);
	// 存放世界坐标的矩阵
	vector<vector<cv::Point3f>> worldcorner;
	vector<cv::Point3f> realPoint;
	ofstream  worldData("worldDatamy_3D.txt");

	for (int k = 0; k < m_Image1.size(); k++)
	{
		for (int i = 0; i < patternSize.height; i++)
		{
			for (int j = 0; j < patternSize.width; j++)
			{

				realPoint.push_back(cv::Point3f(j * square_size.width, i * square_size.height, 0)); // z轴为0
			}
		}
		worldcorner.push_back(realPoint);
		// 写入数据
		worldData << realPoint << endl;
		realPoint.clear();
	}
	m_WorldCorner = worldcorner;
	return m_WorldCorner;
}

// 标定并存储结果
void Calibration_cam::SaveCalibResult(vector<Mat> image, vector<vector<cv::Point3f>>worldcorner, vector<vector<cv::Point2f>>point, cv::Mat cameraMatrix1, cv::Mat distCoeffs1,
	std::vector<cv::Mat> tvecsMat1, std::vector<cv::Mat> rvecsMat1)
{
	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1);
	this->image_size.height = image.at(0).rows;
	this->image_size.width = image.at(0).cols;
	this->cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); // 内参矩阵
	this->distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
	double err = cv::calibrateCamera(worldcorner, point, image_size, cameraMatrix1, distCoeffs1, rvecsMat1, tvecsMat1, cv::CALIB_FIX_K3); //criteria
	ofstream fout("相机标定结果.txt");
	fout << "重投影误差" << err << "像素" << endl;
	cout << "相机标定标定完成！！！" << endl;
	cout << "评价标定结果" << endl;

	double total_err = 0.0;            // 所有图像的平均误差的总和 
	double err1 = 0.0;
	double totalErr = 0.0;
	//double totalPoints = 0.0;
	vector<Point2f> image_points_pro;  // 重投影得到的点
	// 计算重投影误差
	for (int i = 0; i < image.size(); i++)
	{
		projectPoints(worldcorner.at(i), rvecsMat1.at(i), tvecsMat1.at(i), cameraMatrix1, distCoeffs1, image_points_pro);
		err1 = norm(Mat(point.at(i)), Mat(image_points_pro), NORM_L2);
		totalErr += err1 * err1;
		//totalPoints += worldcorner.at(i).size();
		err1 = (err1 * err1) / worldcorner.at(i).size();
		fout << "第" << i + 1 << "幅图像的平均误差：" << sqrt(err1) << "像素" << endl;
		//total_err += err1;
	}

	cout << "开始保存定标结果………………" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
	fout << "相机内参数矩阵：" << endl;
	fout << cameraMatrix1 << endl << endl;
	this->cameraInMatrix = cameraMatrix1;
	fout << "畸变系数：\n";
	fout << distCoeffs1 << endl << endl << endl;
	this->distortion = distCoeffs1;
	for (int i = 0; i < image.size(); i++)
	{
		fout << "第" << i << "幅图像的旋转向量：" << endl;
		fout << rvecsMat1.at(i) << endl;

		/* 将旋转向量转换为相对应的旋转矩阵 */
		Rodrigues(rvecsMat1.at(i), rotation_matrix);
		fout << "第" << i << "幅图像的旋转矩阵：" << endl;
		fout << rotation_matrix << endl;
		this->tMat.push_back(rotation_matrix);
		fout << "第" << i << "幅图像的平移向量：" << endl;
		fout << tvecsMat1.at(i) << endl << endl;
		this->rMat.push_back(tvecsMat1.at(i));
	}
	cout << "相机标定结果完成保存！！！" << endl;
	fout << endl;
}




vector<vector<cv::Point2f>> Calibration_cam::CalibMain(const string imageFolderPath, int BOARD_WIDTH, int BOARD_HEIGHT, int BOARD_SCALE)
{
	cv::Size board_size = cv::Size(BOARD_WIDTH, BOARD_HEIGHT);
	//图片大小
	this->m_Image1 = ReadImages(imageFolderPath);
	//寻找圆心
	this->m_Point = FindCorners(m_Image1, board_size);

	// 提取亚像素边缘
	//this->m_Point = FindCorners_sub(m_Image1, board_size);
	// 绘制

	this->DrawCorners(m_Image1, board_size, m_Point);
	// 世界坐标
	this->m_WorldCorner_1 = CalworldCorner(m_WorldCorner, BOARD_SCALE, BOARD_SCALE, board_size);
	// 计算并保存结果
	SaveCalibResult(m_Image1, m_WorldCorner_1, m_Point, cameraMatrix, distCoeffs, tvecsMat, rvecsMat);
	return m_Point;
}