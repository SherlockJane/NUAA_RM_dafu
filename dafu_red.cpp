#define dafu_red
#ifdef dafu_red
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
		mid_chn_img = blue_channel- red_channel;
		imshow("blue", mid_chn_img);
		waitKey(0);
	}
	threshold(mid_chn_img, threadImg, 60, 255, CV_THRESH_BINARY);
}
int video_write(Mat &tempImg, int rate, Size &size)
{
	VideoWriter writer;
	writer.open("E:\\RM\\tx2\\2019_win_dafu\\dafu\\dafu\\video\\blue_range.avi", CV_FOURCC('M', 'J', 'P', 'G'), rate, size, false);//color-true  gray-false
	writer << tempImg;
	waitKey(10);
	return 1;
}
float GetPixelLength(Point PixelPointO, Point PixelPointA)
{
	float PixelLength;
	PixelLength = powf((PixelPointO.x - PixelPointA.x), 2) + powf((PixelPointO.y - PixelPointA.y), 2);
	PixelLength = sqrtf(PixelLength);
	return PixelLength;
}
int main()
{
	Mat srcImg, threadImg, cutImg, thresholdImg;
	Mat tempImg, dstImg;
	namedWindow("src", CV_WINDOW_AUTOSIZE);
	//namedWindow("threadImg", CV_WINDOW_AUTOSIZE);
	namedWindow("circle", CV_WINDOW_AUTOSIZE);
#ifdef video_test
	VideoCapture cap;
	cap.open("E:\\RM\\tx2\\2019_win_dafu\\dafu\\dafu\\video\\dafu_red_cut.avi");
	if (!cap.isOpened())//如果视频不能正常打开则返回
	{
		cout << "打开失败" << endl;
		return -1;
	}
	//get cap information
	int rate = cap.get(CV_CAP_PROP_FPS);	//帧率
	Size size = Size(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));
	Size size_cut = Size(500,300);
	Rect rect(54, 70, 500, 300);
	Mat alphaImg = Mat::zeros(size, CV_8UC3);
	Mat rangeImg = Mat::zeros(size, CV_8U);
	//write video
	VideoWriter writer;
	//writer.open("E:\\RM\\tx2\\2019_win_dafu\\dafu\\dafu\\video\\dafu_red_detect.avi", CV_FOURCC('M', 'J', 'P', 'G'), rate, size_cut, true);//color-true  gray-false
	while (cap.read(srcImg))
	{
		imshow("src", srcImg);
		waitKey(5);

//---------------------------------------range------------------------------------------------//
		cvtColor(srcImg, tempImg, CV_BGR2HSV);
		Scalar hsv_min(0, 0, 110);//135
		Scalar hsv_max(255, 255, 255);
		inRange(tempImg, hsv_min, hsv_max, rangeImg);
		//imshow("rangeImg", rangeImg);
		static Mat kernel_close = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
		morphologyEx(rangeImg, thresholdImg, MORPH_DILATE, kernel_close);
		/*Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
		morphologyEx(rangeImg, dstImg, MORPH_DILATE, element, Point(-1, -1));*/
		//morphologyEx(dstImg, dstImg, MORPH_ERODE, element, Point(-1, -1), 1);
		//imshow("dstImg", dstImg);
		//waitKey(10);
//---------------------------------------blue-red------------------------------------------------//
		/*if (bgr2binary(srcImg, 1, threadImg))
		{
			imshow("threadImg_q", threadImg);
			waitKey(5);
			static Mat kernel_close = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
			morphologyEx(threadImg, threadImg, MORPH_DILATE, kernel_close);
			imshow("threadImg", threadImg);
			waitKey(5);		
		}
		else
		{
			cout << "颜色处理时，图片为空" << endl;
			return -1;
		}*/

//--------------------------------------alpha and light--------------------------------------------//
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
//--------------------------------------Canny and contours--------------------------------------------//
		//Mat canny_output;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		// 用Canny算子检测边缘
		//Canny(thresholdImg, canny_output, 100, 100 * 2, 3);
		// 寻找轮廓
		findContours(thresholdImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		if (contours.size() <= 0) continue;
		//cout << "contours:" << contours.size() << "hierarchy" << hierarchy.size() << endl;
		// 绘出轮廓
		Mat drawing = Mat::zeros(thresholdImg.size(), CV_8UC3);
		Mat drawing_hou = Mat::zeros(thresholdImg.size(), CV_8UC3);
		Mat drawing_qian = Mat::zeros(thresholdImg.size(), CV_8UC3);
		for (int i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(0, 0, 255);
			drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
		}
		imshow("drawing", drawing);
		/*for (int i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(0, 0, 255);
			drawContours(drawing_hou, contours, i, color, 2, 8, hierarchy[2], 0, Point());
		}
		imshow("drawing_hou", drawing_hou);*/
		/*for (int i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(0, 255, 0);
			drawContours(drawing_qian, contours, i, color, 2, 8, hierarchy[3], 0, Point());
		}
		imshow("drawing_qian", drawing_qian);*/

//--------------------------------------------find armour------------------------------------------------------//
		Mat armourImg = Mat::zeros(size, CV_8UC3);
		vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合 
		//没有子轮廓，但是有父轮廓的——装甲板的第一个特征
		for (int i = 0; i < hierarchy.size(); i++)
		{
			if (hierarchy[i].val[2] == -1 && hierarchy[i].val[3] != -1)
			{
				Scalar color = Scalar(0,0,255);
				drawContours(srcImg, contours, i, color, 1, 8, hierarchy);
				box[i].center.x = 1;
			}
			else
			{
				box[i].center.x = -1;
			}
		}
		//求最小外接矩形
		Point2f rect[4];
		for (int i = 0; i < contours.size(); i++)
		{
			if (contours[i].size() <= 0) continue;
			if (box[i].center.x != -1)
			{
				box[i] = minAreaRect(Mat(contours[i]));
				box[i].points(rect);
			}
		}
		//根据长宽和面积来筛选装甲板
		int armor_cnt = 0;
		vector <Point2f> DetectArmourCenter(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			if (box[i].center.x != -1)
			{
				//长的为width，短的为height
				//cout << "width:" << box[i].size.width << "  height:" << box[i].size.height << endl;	
				if ((box[i].size.width * box[i].size.height>60)&& (box[i].size.width * box[i].size.height <180))
				{
					//长的为width，短的为height
					float detect_dafu_armour_pixel_width;
					float detect_dafu_armour_pixel_height;
					if (box[i].size.width > box[i].size.height)
					{
						detect_dafu_armour_pixel_width = box[i].size.width;
						detect_dafu_armour_pixel_height = box[i].size.height;
					}
					else
					{
						detect_dafu_armour_pixel_width = box[i].size.height;
						detect_dafu_armour_pixel_height = box[i].size.width;
					}
					if ((detect_dafu_armour_pixel_width*1.0 / detect_dafu_armour_pixel_height) < 2)
					{
						Scalar color = Scalar(255, 0, 0);
						drawContours(srcImg, contours, i, color, 2, 8, hierarchy);
						DetectArmourCenter[armor_cnt] = box[i].center;
						armor_cnt += 1;
					}
					else
					{
						box[i].center.x = -1;
					}
				}
				else
				{
					box[i].center.x = -1;
				}

			}
		}
		//imshow("armourImg", armourImg);
		if (armor_cnt == 0)
			continue;
		for (int i = 0; i < armor_cnt; i++)
		{
			circle(srcImg, Point(DetectArmourCenter[i].x, DetectArmourCenter[i].y), 2, Scalar(0, 0, 255), 1);
		}
//-------------------------------------------------find center------------------------------------------------------//
		int dafu_center_cnt = 0;
		int isDafu_center = 0;
		Point2f DafuCenter;
		DafuCenter.x = size.width / 2;
		DafuCenter.y = size.height / 2;
		vector <Point2f> DetectDafuCenter(contours.size());     //记录可能是大符中心的点
		for (int i = 0; i < hierarchy.size(); i++)
		{
			//绘制轮廓的最小外接圆
			float radius;
			if ((hierarchy[i].val[2] == -1 )&& (hierarchy[i].val[3] == -1))//
			{
				
				minEnclosingCircle(contours[i], DetectDafuCenter[i], radius);
				//circle(srcImg, Point(DetectDafuCenter[i].x, DetectDafuCenter[i].y), radius, (255, 255,0), 2);
				if (radius > 8 || radius < 4)
				{
					DetectDafuCenter[i].x = -1;
				}
				else
				{
					dafu_center_cnt++;
					circle(srcImg, Point(DetectDafuCenter[i].x, DetectDafuCenter[i].y), radius, Scalar(255, 0, 0), 2);		
					//cout << DetectDafuCenter[i].x << "  " << DetectDafuCenter[i].y << endl;
					//drawContours(dstImage, contours, i, Scalar(0, 255, 0), 1, 8, hierarcy);
				}
			}
			else
			{
				DetectDafuCenter[i].x = -1;
			}
		}
		if (!dafu_center_cnt)
		{
			cout << "!!! can not find center in step1!" << endl;
			continue;
		}
		//匹配圆心
		float real_armour_dafuCenter_pixel_length = 56;
		for (int i = 0; i < contours.size(); i++)
		{
			if (DetectDafuCenter[i].x != -1)
			{
				int flag_error_center = 1;
				for (int j = 0; j < armor_cnt; j++)
				{
					float pixel_length = GetPixelLength(DetectDafuCenter[i], DetectArmourCenter[j]);
					//cout << "pixel_length: " << pixel_length << endl;
					float transformation;
					//if (armor_cnt <= 2)
						//transformation = 0.15;
					//else
						transformation = 0.2;
					if (pixel_length > real_armour_dafuCenter_pixel_length*(1 + transformation) || pixel_length < real_armour_dafuCenter_pixel_length*(1 - transformation))
					{
						flag_error_center = 0;
						continue;
					}
				}
				if (flag_error_center)
				{
					circle(srcImg, Point(DetectDafuCenter[i].x, DetectDafuCenter[i].y), 3, Scalar(0, 0, 255), 2);
					DafuCenter = DetectDafuCenter[i];
					isDafu_center = 1;
					continue;
				}

			}
		}
		if (!isDafu_center)
		{
			cout << "!!! not find Dafu center!" << endl;
			continue;
		}
//--------------------------------------------find smallest circle-----------------------------------------------------//
		Point2f max_distance_Center=DafuCenter;
		int is_detect_armour_circle = 0;
		//cout << "center:" << max_distance_Center.x << "  " << max_distance_Center.y << endl;
		float max_distance = 0;
		for (int i = 0; i < contours.size(); i++)
		{
			if (DetectDafuCenter[i].x != -1)
			{cout << DetectDafuCenter[i].x << "  " << DetectDafuCenter[i].y << endl;
				float center_distance = GetPixelLength(DetectDafuCenter[i], DafuCenter);
				if (center_distance > max_distance)
				{
					max_distance = center_distance;
					max_distance_Center = DetectDafuCenter[i];
					is_detect_armour_circle = 1;
				}
			}
		}
		circle(srcImg, Point(max_distance_Center.x, max_distance_Center.y), 3, Scalar(0, 255, 255), 2);
		Point2f shootarmour = DafuCenter;
		float min_distance = size.width;
		int is_shoot_armour = 0;
		if (is_detect_armour_circle)
		{
			for (int i = 0; i < armor_cnt; i++)
			{
				float armour_circle_distance = GetPixelLength(DetectArmourCenter[i], max_distance_Center);
				if (armour_circle_distance < min_distance)
				{
					min_distance = armour_circle_distance;
					shootarmour = DetectArmourCenter[i];
					is_shoot_armour = 1;
				}
				
			}
			if (is_shoot_armour)
			{
				cout << "shoot_armour: " << shootarmour.x << "  " << shootarmour.y << endl;
				circle(srcImg, Point(shootarmour.x, shootarmour.y), 3, Scalar(0, 255, 0), 4);
			}
		}
		imshow("circle", srcImg);
		waitKey(5);
	/*	writer << srcImg;
		waitKey(10);*/
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