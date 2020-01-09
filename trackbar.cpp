//#define trackbar_test
#ifdef trackbar_test
#include"stdlib.h"   //system需要调用这个
#include "fstream"
#include "cv.h"
#include "highgui.h"
#include"iostream"
#include "highgui.h" 
#include "opencv2/opencv.hpp"   
using namespace cv;
using namespace std;
#define WINDOW_NAME "光线"

int H_max = 256;
int S_max = 256;
int V_max = 255;
int H_min = 0;
int S_min = 0;
int V_min = 0;
int a = 0;
int summ = 0;
int R_max = 256;
int G_max = 256;
int B_max = 255;
int R_min = 0;
int G_min = 0;
int B_min = 0;
void on_Trackbar(int, void *)
{
	Mat srcImg = imread("dafu_red.png", 1);
	Mat tempImg;
	CvScalar st;
	Mat dstImg = Mat::zeros(srcImg.size(), CV_8U);

	//hsv
	float maxV = 0, minV = 0;
	float V = 0, S = 0, H = 0;
	float v = 0, s = 0, h = 0;
	float R = 0, G = 0, B = 0;
	float delta = 0, tmp = 0;
	int all = 0;
	int max_ = 0, min_ = 0;
	int sumRGB = 0;
	cvtColor(srcImg, tempImg, CV_BGR2HSV);
	Scalar hsv_min(H_min, S_min, V_min);
	Scalar hsv_max(H_max, S_max, V_max);
	inRange(tempImg, hsv_min, hsv_max, dstImg);
	/*Mat tempImg = dstImage;
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

	morphologyEx(tempImg, tempImg, MORPH_ERODE, element, Point(-1, -1), 1);

	morphologyEx(tempImg, tempImg, MORPH_DILATE, element, Point(-1, -1), 4);*/
	

	imshow("result", dstImg);
	imshow(WINDOW_NAME, srcImg);
}
int main(int argc, char** argv)
{
	namedWindow(WINDOW_NAME, WINDOW_NORMAL);
	createTrackbar("h:", WINDOW_NAME, &H_min, 256, on_Trackbar);
	createTrackbar("s:", WINDOW_NAME, &S_min, 256, on_Trackbar);
	createTrackbar("v:", WINDOW_NAME, &V_min, 256, on_Trackbar);
	createTrackbar("H:", WINDOW_NAME, &H_max, 256, on_Trackbar);
	createTrackbar("S:", WINDOW_NAME, &S_max, 256, on_Trackbar);
	createTrackbar("V:", WINDOW_NAME, &V_max, 256, on_Trackbar);
	createTrackbar("L:", WINDOW_NAME, &a, 256, on_Trackbar);
	//createTrackbar("sum:", WINDOW_NAME, &summ, 256, on_Trackbar);
	//createTrackbar("r:", WINDOW_NAME, &R_min, 256, on_Trackbar);
	//createTrackbar("g:", WINDOW_NAME, &G_min, 256, on_Trackbar);
	//createTrackbar("b:", WINDOW_NAME, &B_min, 256, on_Trackbar);
	//createTrackbar("R:", WINDOW_NAME, &R_max, 256, on_Trackbar);
	//createTrackbar("G:", WINDOW_NAME, &G_max, 256, on_Trackbar);
	//createTrackbar("B:", WINDOW_NAME, &B_max, 256, on_Trackbar);

	waitKey(0);
	return 0;
}


#endif //trackbar_test


