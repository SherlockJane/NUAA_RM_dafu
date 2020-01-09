//#define dafu
#ifdef dafu 
//#define open_serial
#define video_test
//#define jpg_test

#include <iostream>
#ifdef open_serial
#include "serial.h"
#endif //open_serial
#include <thread>
#include <stdio.h>
#include "stdlib.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/opengl.hpp"
#include "opencv2/video.hpp" 

using namespace std;
using namespace cv;

int bgr2binary(Mat &srcImg, bool is_red, Mat &threadImg)
{
	if (srcImg.empty())
		return 0;
	//  dafu_ZS_img=srcImg.clone();
	std::cout << "copy bgr done" << std::endl;
	//method 1: split channels and substract
	vector<Mat> imgChannels;
	split(srcImg, imgChannels);
	Mat red_channel = imgChannels.at(2);
	Mat blue_channel = imgChannels.at(0);
	Mat mid_chn_img;
	if (is_red)
	{
		mid_chn_img = red_channel - blue_channel;
	}
	else
	{
		mid_chn_img = blue_channel +blue_channel - red_channel - red_channel;
		//imshow("blue", mid_chn_img);
		//waitKey(0);
	}
	threshold(mid_chn_img, threadImg, 80, 255, CV_THRESH_BINARY);
}
int video_write(Mat &tempImg, int rate, Size &size)
{
	VideoWriter writer;
	writer.open("E:\\RM\\tx2\\2019_win_dafu\\dafu\\dafu\\video\\blue_range.avi", CV_FOURCC('M', 'J', 'P', 'G'), rate, size, false);//color-true  gray-false
	writer<< tempImg;
	waitKey(10);
	return 1;
}


int main()
{
	Mat srcImg, threadImg;
	Mat tempImg, dstImg;
	
	namedWindow("src", CV_WINDOW_AUTOSIZE);
	namedWindow("threadImg", CV_WINDOW_AUTOSIZE);
	namedWindow("alphaImg", CV_WINDOW_AUTOSIZE);
#ifdef video_test
	VideoCapture cap;
	cap.open("E:\\RM\\tx2\\2019_win_dafu\\dafu\\dafu\\video\\dafu_blue_cut.avi");
	if (!cap.isOpened())//如果视频不能正常打开则返回
	{
		cout << "打开失败" << endl;
		return -1;
	} 
	//get cap information
	int rate = cap.get(CV_CAP_PROP_FPS);	//帧率
	Size size = Size(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));
	Mat alphaImg=Mat::zeros(size, CV_8UC3);
	Mat rangeImg = Mat::zeros(size, CV_8U);
	VideoWriter writer;
	//writer.open("E:\\RM\\tx2\\2019_win_dafu\\dafu\\dafu\\video\\.avi", CV_FOURCC('M', 'J', 'P', 'G'), rate, size_cut, true);//color-true  gray-false
	while (cap.read(srcImg))
	{
		imshow("src", srcImg);
		waitKey(5);

		//range
		cvtColor(srcImg, tempImg, CV_BGR2HSV);
		Scalar hsv_min(30, 100, 155);//135
		Scalar hsv_max(255, 255, 255);
		inRange(tempImg, hsv_min, hsv_max, rangeImg);
		//imshow("rangeImg", rangeImg);
		//waitKey(10);
		Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
		morphologyEx(rangeImg, dstImg, MORPH_DILATE, element, Point(-1, -1));
		//morphologyEx(dstImg, dstImg, MORPH_ERODE, element, Point(-1, -1), 1);
		
		imshow("dstImg", dstImg);
		waitKey(10);
		//write video
		/*writer << dstImg;
		waitKey(10);*/

		//blue-red
		//if (bgr2binary(srcImg, 0, threadImg))
		//{
		//	//imshow("threadImg", threadImg);
		//	//waitKey(5);		
		//}
		//else
		//{
		//	cout << "颜色处理时，图片为空" << endl;
		//	return -1;
		//}

		//alpha and light
		//double alpha = 3.0; /*< Simple contrast control */
		//int beta = 20;       /*< Simple brightness control */
		//for (int y = 0; y < srcImg.rows; y++) {
		//	for (int x = 0; x < srcImg.cols; x++) {
		//		for (int c = 0; c < 3; c++) {
		//			alphaImg.at<Vec3b>(y, x)[c] =
		//				saturate_cast<uchar>(alpha*(srcImg.at<Vec3b>(y, x)[c]) + beta);
		//		}
		//	}
		//}
		//imshow("alphaImg", alphaImg);
		//waitKey(10);

		Mat canny_output;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		/// 用Canny算子检测边缘
		Canny(dstImg, canny_output, 100, 100 * 2, 3);
		/// 寻找轮廓
		findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		/// 绘出轮廓
		Mat drawing_hou = Mat::zeros(canny_output.size(), CV_8UC3);
		Mat drawing_qian = Mat::zeros(canny_output.size(), CV_8UC3);
		for (int i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(0, 0, 255);
			drawContours(drawing_hou, contours, i, color, 2, 8, hierarchy[2], 0, Point());
		}
		imshow("drawing_hou", drawing_hou);
		for (int i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(0, 255, 0);
			drawContours(drawing_qian, contours, i, color, 2, 8, hierarchy[3], 0, Point());
		}
		imshow("drawing_qian", drawing_qian);
		/*writer << drawing;
		waitKey(10);*/
		int dafu_center_cnt = 0;
		vector <Point2f> DetectDafuCenter(contours.size());     //可能是大符中心的点
		for (int i = 0; i < hierarchy.size(); i++)
		{
			//绘制轮廓的最小外接圆
			float radius;
			if (hierarchy[i].val[2] == -1 && hierarchy[i].val[3] == -1)
			{

				minEnclosingCircle(contours[i], DetectDafuCenter[i], radius);
				if (radius > 5 || radius < 2)
				{
					DetectDafuCenter[i].x = -1;
				}
				else
				{
					dafu_center_cnt++;
					circle(srcImg, Point(DetectDafuCenter[i].x, DetectDafuCenter[i].y), radius, (255, 255, 255), 2);
					//drawContours(dstImage, contours, i, Scalar(0, 255, 0), 1, 8, hierarcy);
				}
			}
		}

		//没有父轮廓和子轮廓
		for (int i = 0; i < hierarchy.size(); i++)
		{
			if (hierarchy[i].val[2] == -1 && hierarchy[i].val[3] != -1)
			{
				Scalar color = Scalar(0, 0, 255);
				drawContours(rangeImg, contours, i, color, 1, 8, hierarchy);
			}
		}
	/*	imshow("find_1", srcImg);
		waitKey(10);*/
		waitKey(100);
		//rect
		vector<Point> rectPoint;
		for (int i = 0; i < contours.size(); i++)
		{
			//获得矩形外包围框
			Rect r = boundingRect(Mat(contours[i]));
			if (((float)r.width * r.height > 150) &&((float)r.width * r.height <220) )//&& ((((float)r.width / r.height < 2.0)&& ((float)r.width / r.height >1.5))|| (((float)r.width / r.height < 0.6) && ((float)r.width / r.height > 0.4)))
			{
				rectangle(srcImg, r, Scalar(0, 0, 255), 2);
			}
			
			//RotatedRect r = minAreaRect(Mat(contours[i]));
			//cout << "contours" << i << "height=" << r.height << "width =" << r.width << "rate =" << ((float)r.width / r.height) << endl;

			//根据矩形宽高比和面积筛选矩形
			//if ((float)r.width / r.height >= 1.5 && (float)r.width / r.height <= 2.2 && (float)r.width * r.height >= 3000 && (float)r.width * r.height <= 10000)
			//{
			//	cout << "r.x = " << r.x << "  r.y  = " << r.y << "rate =" << ((float)r.width / r.height) << " area = " << ((float)r.width * r.height) << endl;

			//	Point p1, p2, p3, p4;
			//	p1.x = r.x;
			//	p1.y = r.y;
			//	p2.x = r.x + r.width;
			//	p2.x = r.y;
			//	p3.x = r.x + r.width;
			//	p3.y = r.y + r.height;
			//	p4.x = r.x;
			//	p4.y = r.y + r.height;
			//	rectPoint.push_back(p1);
			//	rectPoint.push_back(p2);
			//	rectPoint.push_back(p3);
			//	rectPoint.push_back(p4);
			//	//画矩形
			//	rectangle(frame, r, Scalar(0, 0, 255), 2);

			//}
		}
		imshow("rect", srcImg);
		waitKey(10);
			
	}


	
#endif //video_test
#ifdef jpg_test
	srcImg = imread("dafu_blue.jpg");
	imshow("src", srcImg);
	waitKey(0);
	if (bgr2binary(srcImg, 0, threadImg))
	{
		imshow("threadImg", threadImg);
		waitKey(0);
	}
	else
	{
		cout << "颜色处理时，图片为空" << endl;
		return -1;
	}
#endif //jpg_test

	
	system("pause");
	return 0;
}
#endif //dafu