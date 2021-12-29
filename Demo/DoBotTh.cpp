// DoBotTh.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。

#include <iostream>
#include <string>
#include <list>
#include <vector>
#include <map>
#include <stack>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgproc/imgproc_c.h"
#include "DobotDll.h"
#include "DobotType.h"
#include <stdio.h>
#include <fstream> 
using namespace cv;
using namespace std;

struct Dobot_Pose {
	float x;
	float y;
};

uint64_t queuedCmdIndex = 0;
/*/
void OnMouseAction(int event, int x, int y, int flags, void* ustc) {

	if (event == cv::EVENT_LBUTTONDOWN) {
		ofstream streams;
		streams.open(R"(E:\智能机器人\DoBotC++\data.txt)", ios::app);
		streams << x  << " " << y  << endl;
	}
}
*/
cv::Point2f solveimage(cv::Mat& image) { //处理照片
	Mat grayimage;
	Mat binaryimage;
	// 将颜色转化为灰色并进二值化，得到二值图像
	cv::imshow("1", image);
	cvtColor(image, grayimage, cv::COLOR_BGR2GRAY);
	cv::imshow("2", grayimage);
	threshold(grayimage, binaryimage, 100, 255, THRESH_BINARY);
	auto kernal = getStructuringElement(MORPH_RECT, Size(15 * 2 + 1, 15 * 2 + 1));
	// 进行开操作
	morphologyEx(binaryimage, binaryimage, MORPH_OPEN, kernal);
	vector<vector<Point>>contours;
	vector<Vec4i>hierarchy;
	findContours(binaryimage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());
	Mat imageContours = Mat::zeros(binaryimage.size(), CV_8UC1);
	vector<Rect>boundRect(contours.size());
	cout << contours.size() << endl;
	Point2f center; //找出中心
	for (int i = 0; i < contours.size(); i++) {
		boundRect[i] = boundingRect(Mat(contours[i]));
		drawContours(imageContours, contours, i, Scalar(255, 0, 0), 1, 8, hierarchy);
		RotatedRect rect = minAreaRect(contours[i]);
		Point2f P[4];
		rect.points(P);
		center = rect.center;
		if (rect.size.height * rect.size.width < 10000.f) {
			continue;
		}
		std::cout << center.x << " " << center.y << endl;
	}
	resize(binaryimage, binaryimage, Size(image.cols, image.rows));
	cv::imshow("3", binaryimage);
	cv::waitKey(0);
	return center;
}

cv::Mat GetRotation() { //得到转换矩阵
	/*ifstream streams(R"(E:\智能机器人\DoBotT\data.txt)");
	int index[6];
	string lines;
	int i = 0;
	while (i < 3) {
		getline(streams, lines);
		stringstream ss(lines);
		ss >> index[i * 2] >> index[i * 2 + 1];
		i++;
	}
	float px1, px2, px3, py1, py2, py3;
	getline(streams, lines);
	stringstream sss(lines);
	sss >> px1 >> py1 >> px2 >> py2 >> px3 >> py3;
	Mat B = (Mat_<float>(3, 3) << px1, py1, 1, px2, py2, 1, px3, py3, 1);
	Mat A = (Mat_<float>(3, 3) << index[0], index[1], 1, index[2], index[3], 1, index[4], index[5], 1);
	*/
	//像素坐标 3 个点  
	Mat A = (Mat_<float>(3, 3) << 159, 620, 1, 390, 289, 1, 421, 566, 1);// 3*3 写法 x，y，1 
	//物理坐标 3 个点  
	Mat B = (Mat_<float>(3, 3) << -15.57, -295.41, 1, -34.93, -234.58, 1, 42.42, -283.06, 1);////3*3    
	//得到旋转矩阵 
	Mat X;
	solve(A, B, X, DECOMP_SVD);
	//solve(A, B, X, CV_SVD);
	return X;
}


Dobot_Pose solve(float fx, float fy, Mat& X) {
	Mat a1 = (Mat_<float>(1, 3) << fx, fy, 1);
	Mat b1 = a1 * X;
	Dobot_Pose pose;
	pose.x = b1.at<float>(0);
	pose.y = b1.at<float>(1);
	return pose;
}

//确定抓取点
PTPCmd PtpMovePost() { //const Dobot_Pose &pose
	PTPCmd cmd;
	cmd.ptpMode = 0;
	cmd.x = -119.876;
	cmd.y = -233.135;
	cmd.z = -54.82;
	//cmd.x = pose.x;
	//cmd.y = pose.y;
	//cmd.z = -61;
	cmd.r = 0;
	return cmd;
}
//确定放置点
PTPCmd PtpMovePost0() {
	PTPCmd cmd;
	cmd.ptpMode = 0;
	cmd.x = 129.86;
	cmd.y = -209.51;
	cmd.z = -42.6;
	cmd.r = 0;
	return cmd;
}
/*
int split(char** dst, char* str, const char* spl) {
	int n = 0;
	char* result = NULL;
	result = strtok(str, spl);
	while (result != NULL) {
		strcpy(dst[n++], result);
		result = strtok_s(NULL, spl);
	}
	return n;
}*/

int DobotHome() {
	HOMECmd home;
	SetHOMECmd(&home, true, &queuedCmdIndex);
	return 0;
}
//设置 PTP 模式下各笛卡尔坐标轴的速度和加速度 
void SetPTPParmas() {
	PTPCoordinateParams params;
	params.xyzVelocity = 200;
	params.xyzAcceleration = 200;
	SetPTPCoordinateParams(&params, false, NULL);
	return;
}
//设置 JUMP 运动方式的参数 
void SetPTPJump() {
	tagPTPJumpParams params;
	params.jumpHeight = 50;
	params.zLimit = 100;
	SetPTPJumpParams(&params, false, NULL);
	return;
}
int connect(void) { //连接机械臂
	int maxDevCount = 100;
	int maxDevLen = 20;
	char* devsChr = new char[maxDevCount * maxDevLen]();
	char** devsList = new char* [maxDevCount]();
	for (int i = 0; i < maxDevCount; i++)
		devsList[i] = new char[maxDevLen]();
	SearchDobot(devsChr, 1024);
	//split(devsList, devsChr, " ");
	if (ConnectDobot(devsList[0], 115200, NULL, NULL) != DobotCommunicate_NoError) {
		return 0;
	}
	return 1;
}
int InitDobot() {
	SetCmdTimeout(3000);//设置指令超时时间  
	SetQueuedCmdClear();//清空指令  
	SetQueuedCmdStartExec();//执行队列中的指令

	//获取设备序列号
	char deviceSN[64];
	GetDeviceSN(deviceSN, sizeof(deviceSN));
	char deviceName[64];
	GetDeviceName(deviceName, sizeof(deviceName));

	//获取设备版本信息
	uint8_t majorVersion, minorVersion, revision;
	GetDeviceVersion(&majorVersion, &minorVersion, &revision);
	return 0;
}
//吸盘吸取 
void OpenSuctionCup()
{
	SetEndEffectorSuctionCup(1, 1, 1, &queuedCmdIndex);
}
//吸盘关闭 
void CloseSuctionCup()
{
	SetEndEffectorSuctionCup(0, 0, 1, &queuedCmdIndex);
}
/*
void Calibretion(std::string name) {
	auto image = cv::imread(name);
	resize(image, image, Size(image.cols / 5, image.rows / 5));
	namedWindow("image");
	setMouseCallback("image", OnMouseAction, (void*)&image);
	imshow("image", image);
	waitKey(0);
}
*/
int main() {
	// 连接机械臂并且初始化
/*	connect();
	InitDobot();
	SetPTPParmas();
	SetPTPJump();
	DobotHome();*/
	int index = 0;
	int sensor_value = 0;
	// 读取处理图像，选取标定点
	auto image = cv::imread(R"(E:\智能机器人\DoBotC++\1.jpg)");

	// 找到要分类物体的中心
	/*
		得到旋转矩阵
		根据计算的中心得到物体的位姿
		移动到物体的位姿
		移动到要放置的位姿
		执行命令
	*/
	//Point2f center = solveimage(image);
	//Point2f center = ( , );
	//auto X = GetRotation();
	//用转换矩阵X将x，y进行转换
	//Dobot_Pose pose = solve(center.x, center.y, X);
	//Dobot_Pose pose = (-119.87, -233.13, -54.82);
	PTPCmd cmd1 = PtpMovePost(); //pose
	PTPCmd cmd2 = PtpMovePost0();
	SetQueuedCmdStartExec();
	SetPTPCmd(&cmd1, true, &queuedCmdIndex);
	//开启吸盘吸取
	OpenSuctionCup();
	//SetEndEffectorSuctionCup(true, true, true, &queuedCmdIndex);
	SetPTPCmd(&cmd2, true, &queuedCmdIndex);
	//关闭吸盘吸取
	CloseSuctionCup();
	//SetEndEffectorSuctionCup(false, false, true, &queuedCmdIndex);
	SetQueuedCmdStopExec();
	DisconnectDobot();
	ResetPose(0, 0, 0);
	return 0;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
