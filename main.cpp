//#define open_serial
//#define TX2

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
#include "opencv2/video.hpp"     //BackgroundSubtractor在里面

using namespace std;
using namespace cv;

double CvtRealLenghth2PixelLenghth(double RealLenghth_mm, double Distance_mm);
double CvtPixelLenghth2RealLenghth(double PixelLenghth, double Distance_mm);
void DisplayFps(Mat& img);//desplay FPS
int OpenVideoStream(int camWay);
void SendDataToInfantry();
Point2f CaculatePitchYawError(float Pixel_x, float Pixel_y); //通过像素坐标就算云台需要转过的角度
float getDistance(Point pointO, Point pointA);
void CoutFps();        //
void GetCameraPara();

int camWay = 2; // 0: camera1 1: camera2  2: vedio
VideoCapture capture;
String video_file_name = "blue.mp4"; //将要打开的视频路径  注意‘/’和‘\’
double myVideoCaptureProperties[50];   //存储摄像头参数
Point2f DetectedArmourYawPitchError[20];    //存储所有检测出来的装甲板角度偏差值
Point2f SendYawPitchError;            //存储需要发送的装甲板信息
int IsDetectedFlag = 0;                 //是否检测到装甲板信息

#define SerialPort_COM 1    //用于发送的串口的串口号

//不同的操作系统有些配置不一样
#define WINDOWS_OS
//#define LINUX_OS

//识别对象的一些参数
#define lightbar_length_mm 55.0f                //灯条的长度  单位mm
#define lightbar_distance_mini_mm  135.0f             //小装甲板灯条的宽度   单位mm
//#define lightbar_distance_larger_mm               //大装甲板灯条的宽度   单位mm

//识别条件
#define max_detect_distance_mm  3000.0f         //最远识别距离，超过此距离滤掉  单位mm
#define min_detect_distance_mm  500.0f         //最近识别距离，超过此距离滤掉   单位mm
#define max_inclination_degree   35.0f         //灯条对角线倾斜角超过一定度数，滤掉  单位度
#define max_transformation       0.3f        //

//摄像头的一些参数
#define Camera_fx 6.530675507960873e+02
#define Camera_fy 6.521106670863784e+02
#define Camera_fxy 6.525106670863784e+02


#define Camera_vertical_halfangle  20.0   //1/2垂直方向视角 单位度
#define Camera_lateral_halfangle  20.0   //1/2水平方向视角 单位度
#define Camera_image_area_height_um 2453   //
#define Camera_image_area_width_um 3896


//#define Camera_MaxPixel_x 640
//#define Camera_MaxPixel_y 480



//InputImage为三通道图像，将第三通道threshold
Mat mythreshold(Mat &InputImage, int threshold)
{
    Mat OutImage(400, 640, CV_8UC1);

    int channels = InputImage.channels();  //通道数
    int nRows = InputImage.rows;            //行数
    int nCols = InputImage.cols* channels;  //列数

    //if (InputImage.isContinuous())
    //{
    //	nCols *= nRows;
    //	nRows = 1;
    //}
    int i, j;
    uchar* pInputImage;
    uchar* pOutImage;
    for (i = 0; i < nRows; ++i)     //行数
    {
        pInputImage = InputImage.ptr<uchar>(i);
        pOutImage = OutImage.ptr<uchar>(i);
        for (j = 0; j < InputImage.cols; ++j)
        {
            if (pInputImage[j * 3 + 2] > threshold)
                pOutImage[j] = 255;
            else
                pOutImage[j] = 0;
        }
    }
    return OutImage;
}


void DrawAreaRexts(Mat &grayImage, Mat &dstImage)
{
    int detected_armour_cnt = 0;


    //找出轮廓及最小外接矩形
        vector<vector<Point>> contours;   //每一组Point点集就是一个轮廓
        vector<Vec4i> hierarcy;           //矩形集合

        findContours(grayImage, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        //vector<Rect> boundRect(contours.size());  //定义外接正矩形集合
        vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合
        Point2f rect[4];
        for (int i = 0; i < contours.size(); i++)
        {
            box[i] = minAreaRect(Mat(contours[i]));  //计算每个轮廓最小外接矩形
            //boundRect[i] = boundingRect(Mat(contours[i]));
            //circle(dstImage, Point(box[i].center.x, box[i].center.y), 5, Scalar(0, 255, 0), -1, 8);  //绘制最小外接矩形的中心点

            //rectangle(dstImage, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x + boundRect[i].width,
            //			boundRect[i].y + boundRect[i].height), Scalar(0, 255, 0), 2, 8);


        }
        if (box.empty())
            return;


    //筛选出符合条件的灯条
    //条件一：灯条的长度
    {
        float min_lightbar_PixelLength = CvtRealLenghth2PixelLenghth(lightbar_length_mm, max_detect_distance_mm); //最大距离处灯条的像素长度
        float max_lightbar_PixelLength = CvtRealLenghth2PixelLenghth(lightbar_length_mm, min_detect_distance_mm);
        //float testPixelLength_mm = CvtRealLenghth2PixelLenghth(lightbar_distance_mini_mm, 1350);
        for (int i = 0; i < box.size(); i++)
        {
            if (box[i].size.height > box[i].size.width)
            {
                if (box[i].size.height < (max_lightbar_PixelLength + 5) && box[i].size.height >(min_lightbar_PixelLength - 5))
                {
                    //box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
                    //for (int j = 0; j < 4; j++) {

                    //	line(dstImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
                    //}
                }
            }
            else
            {
                if (box[i].size.width < (max_lightbar_PixelLength + 5) && box[i].size.width >(min_lightbar_PixelLength - 5))
                {
                    //box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
                    //for (int j = 0; j < 4; j++) {
                    //	line(dstImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
                    //}
                }
                else
                {
                    box[i].center.x = -1;
                }
            }
        }

    }

    //条件二:灯条的倾斜角度
    for (int i = 0; i < box.size(); i++)
    {
        if (box[i].center.x != -1)
        {

            box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
            float err_vertical = abs(rect[0].y - rect[2].y);
            float err_lateral = abs(rect[0].x - rect[2].x);
            float tan_inclination = err_lateral / err_vertical;
            float tan_rect_inclination = tan(max_inclination_degree / 180.0*3.14);
            if (tan_inclination > tan_rect_inclination) {
                box[i].center.x = -1;
            }
            else
            {
                box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
                for (int j = 0; j < 4; j++) {
                    line(dstImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
                }

            }
        }

    }

    //for (int i = 0; i < box.size(); i++)
    //{
    //	if (box[i].center.x != -1)
    //	{
    //		box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
    //		for (int j = 0; j < 4; j++) {
    //			line(dstImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
    //		}
    //	}
    //}


    //配对：依据灯条长度和灯条之间距离的比值
    for (int i = 0; i < box.size() - 1; i++)    //最后一个不需要找
    {
        if (box[i].center.x != -1) //是否之前已经滤掉
        {
            //
            float lightbar_PixelLength; //第一个灯条的长度
            if (box[i].size.width > box[i].size.height)
                lightbar_PixelLength = box[i].size.width;
            else
                lightbar_PixelLength = box[i].size.height;

            float real_ratio = lightbar_length_mm / lightbar_distance_mini_mm; //灯条长度和灯条之间距离的比值
            for (int j = i + 1; j < box.size(); j++)  //向后搜索
            {
                if (box[j].center.x != -1)  //是否之前已经滤掉
                {
                    float pixel_ratio = lightbar_PixelLength / (float)(abs(box[i].center.x - box[j].center.x)); //灯条长度和灯条之间像素距离的比值

                    if (pixel_ratio< real_ratio*(max_transformation + 1) && pixel_ratio > real_ratio*(1 - max_transformation))  //判断比值
                    {


                        float lightbar_PixelLength2; //第二个灯条的长度
                        if (box[j].size.width > box[j].size.height)
                            lightbar_PixelLength2 = box[j].size.width;
                        else
                            lightbar_PixelLength2 = box[j].size.height;

                        if (lightbar_PixelLength2 / lightbar_PixelLength > (1 - max_transformation) && lightbar_PixelLength2 / lightbar_PixelLength < (1 + max_transformation)) //两个灯条的长度不能差太多
                        {
                            float err_vertical = abs(box[i].center.y - box[j].center.y);
                            float err_lateral = abs(box[i].center.x - box[j].center.x);
                            float tan_inclination = (err_vertical / err_lateral);

                            if (tan_inclination < tan(max_inclination_degree / 180.0*3.14)) //判断灯条中心连线的倾斜角
                            {
                                float dis_x = (int)((box[i].center.x + box[j].center.x) / 2);
                                float dis_y = (int)((box[i].center.y + box[j].center.y) / 2);
                                circle(dstImage, Point(dis_x, dis_y), 5, (0, 0, 255), 1);
                                Point2f PitchYawError = CaculatePitchYawError(dis_x, dis_y);
                                //cout << PitchYawError.x << "   " << PitchYawError.y << endl;

                                DetectedArmourYawPitchError[detected_armour_cnt] = PitchYawError;
                                detected_armour_cnt++;

                            }
                        }



                    }
                }
            }
        }

    }


    if (detected_armour_cnt == 0)
    {
       // SendYawPitchError = Point2f(0,0);
    }
    else
    {//找出离中心最近的装甲板，发送给下位机
        float max_error = 0;
        for (int i = 0; i < detected_armour_cnt;i++)
        {
            float error = DetectedArmourYawPitchError[i].x*DetectedArmourYawPitchError[i].x +
                DetectedArmourYawPitchError[i].y*DetectedArmourYawPitchError[i].y*0.5;   //减小垂直方向的权重
            if (error > max_error)
            {
                max_error = error;
                SendYawPitchError =DetectedArmourYawPitchError[i];

            }

        }



    }


    cout << SendYawPitchError.x << "   " << SendYawPitchError.y << endl;

    imshow("dst", dstImage);

}



int main()
{
#ifdef open_serial
    {//打开串口
         Serial sel;
        if(sel.setPara(115200,8,1,'n'))
         cout<<"config success"<<endl;
        else
         cout<<"config failed"<<endl;
    }
#endif //open_serial

    Mat frame_read, frame_read2gray;
    Mat threshold_frame;             //二值化图像存贮

    OpenVideoStream(camWay);
    float TickFrequency = cv::getTickFrequency();

    //SendTypedef(mySerialPort);

    double time1 = 0.001;
    double time2 = 0.002;

    //namedWindow("input video", WINDOW_AUTOSIZE);
    //namedWindow("OUT", WINDOW_AUTOSIZE);
    //namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"
    //createTrackbar("LowH", "Control", &test_present, 10000); //Hue (0 - 179)

    GetCameraPara();
   // capture.set(CAP_PROP_IRIS, test_present);  //
    while (capture.read(frame_read)) {
        //imshow("input video", frame_read);
        //cvtColor(frame_read, frame_read2gray, COLOR_BGR2GRAY);
        //imshow("frame_read2gray", frame_read2gray);


        time1 = (double)cv::getTickCount();
        cout << "读取图像耗时" << (time2 - time1) / TickFrequency << endl;

        threshold_frame = mythreshold(frame_read, 200);  //图像二值化
        DrawAreaRexts(threshold_frame, frame_read);   //画灯条，求装甲板位置，算云台Yaw和Pitch偏角
        time2 = (double)cv::getTickCount();
             cout << "计算耗时" << (time1 - time2) / TickFrequency << endl;

        char c = waitKey(1);

         time1 = (double)cv::getTickCount();
           SendDataToInfantry();
        time2 = (double)cv::getTickCount();
        cout << "发送数据耗时" << (time1 - time2) / TickFrequency << endl;

        time2 = (double)cv::getTickCount();
        CoutFps();
    }

    return 0;
}

int OpenVideoStream(int camWay)
{


    if (camWay == 2) {
        capture.open(video_file_name); return true;
    }
    else if (camWay == 0) {
        capture.open(0); return true;
    }
    else if (camWay == 1) {
        capture.open(1); return true;
    }

    if (!capture.isOpened())
    {
        printf("can not open camera or video file\n");
        return -1;
    }



}


float getDistance(Point pointO, Point pointA)
{
    float distance;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
    distance = sqrtf(distance);
    return distance;
}

void CoutFps()
{
    double  fps = 0;
    double t = 0;

    static double time1 = 0.001;
    static double time2 = 0.002;
    time2 = time1;
    time1 = (double)cv::getTickCount();

    t = (time1 - time2) / cv::getTickFrequency();
    fps = 1.0 / t;

    //std::cout << t << "\n";
    cout <<"fps"<< fps << "\n";
    //std::cout << cv::getTickFrequency() << "\n\n";

}

void DrawAreaCircles(Mat &grayImage, Mat &dstImage)
{
    Point my_circle, real_circle;
    my_circle.x = 136;
    my_circle.y = 120;
    //转换为灰度图并平滑滤波
    //cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);

    //定义变量
    vector<vector<Point>>contours;
    vector<Vec4i>hierarchy;

    grayImage = grayImage > 100;
    findContours(grayImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    //绘制轮廓图

    //dstImage = Mat::zeros(grayImage.size(), CV_8UC3);
    //for (int i = 0; i < hierarchy.size(); i++)
    //{
    //	Scalar color = Scalar(rand() % 255, rand() % 255, rand() % 255);
    //	drawContours(dstImage, contours, i, color, FILLED, 8, hierarchy);
    //}


    Point2f MaxMeanCenter; //
    double  MaxMean = 0.0;
    for (int i = 0; i < contours.size(); i++)
    {
        //绘制轮廓的最小外接圆
        Point2f center; float radius;
        minEnclosingCircle(contours[i], center, radius);



        if (radius > 25 && radius < 35) {                //getDistance(my_circle, center) <100 &&
            circle(dstImage, center, radius, Scalar(255), 2);



            //设置ROI区域
            Rect rect;
            rect.x = center.x - 10, rect.y = center.y - 10, rect.width = 20, rect.height = 20;//ROI0 的坐标

            Mat ROI = grayImage(rect);
            Mat means, stddev, covar;
            meanStdDev(ROI, means, stddev);//计算src图片的均值和标准差

            cout << "mean" << means.at<double>(0) << endl;
            if ((means.at<double>(0)) > MaxMean) {
                MaxMean = means.at<double>(0);
                MaxMeanCenter = center;
            }
            imshow("ROI", ROI);

        }

    }

    cout << "MaxMean" << MaxMean << endl;
    circle(dstImage, MaxMeanCenter, 50, Scalar(255), 2);

    imshow("轮廓图", dstImage);


}

void GetCameraPara()
{

    //capture.set(CAP_PROP_IRIS, 10);
    capture.set(CAP_PROP_FRAME_WIDTH, 640);//宽度
    capture.set(CAP_PROP_FRAME_HEIGHT, 400);//高度  分辨率设置成640*400时帧率是240
    capture.set(CAP_PROP_EXPOSURE, 1);
    capture.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));




    myVideoCaptureProperties[CAP_PROP_IRIS] = capture.get(CAP_PROP_IRIS);
    cout << "CAP_PROP_IRIS:" << myVideoCaptureProperties[CAP_PROP_IRIS] << endl;


    myVideoCaptureProperties[CAP_PROP_FRAME_WIDTH] = capture.get(CAP_PROP_FRAME_WIDTH);
    cout << "FRAME_WIDTH:" << myVideoCaptureProperties[CAP_PROP_FRAME_WIDTH] << endl;

    myVideoCaptureProperties[CAP_PROP_FRAME_HEIGHT] = capture.get(CAP_PROP_FRAME_HEIGHT);
    cout << "FRAME_HEIGHT:" << myVideoCaptureProperties[CAP_PROP_FRAME_HEIGHT] << endl;

    myVideoCaptureProperties[CAP_PROP_FPS] = capture.get(CAP_PROP_FPS);
    cout << "CAP_PROP_FPS:" << myVideoCaptureProperties[CAP_PROP_FPS] << endl;


    myVideoCaptureProperties[CAP_PROP_BRIGHTNESS] = capture.get(CAP_PROP_BRIGHTNESS);
    cout << "CAP_PROP_BRIGHTNESS:" << myVideoCaptureProperties[CAP_PROP_BRIGHTNESS] << endl;


    myVideoCaptureProperties[CAP_PROP_CONTRAST] = capture.get(CAP_PROP_CONTRAST); //对比度
    //cout << "CAP_PROP_CONTRAST:" << myVideoCaptureProperties[CAP_PROP_CONTRAST] << endl;


    myVideoCaptureProperties[CAP_PROP_SATURATION] = capture.get(CAP_PROP_SATURATION); //饱和度
    myVideoCaptureProperties[CAP_PROP_HUE] = capture.get(CAP_PROP_HUE);

    myVideoCaptureProperties[CAP_PROP_EXPOSURE] = capture.get(CAP_PROP_EXPOSURE);   //曝光
    cout << "CAP_PROP_EXPOSURE:" << myVideoCaptureProperties[CAP_PROP_EXPOSURE] << endl;

    myVideoCaptureProperties[CAP_PROP_FRAME_COUNT] = capture.get(CAP_PROP_FRAME_COUNT);//视频帧数

    myVideoCaptureProperties[CAP_PROP_CONVERT_RGB] = capture.get(CAP_PROP_CONVERT_RGB);//
    cout << "CAP_PROP_CONVERT_RGB:" << myVideoCaptureProperties[CAP_PROP_CONVERT_RGB] << endl;

    for (int i = 0; i < 40; i++)
    {
        myVideoCaptureProperties[i] = capture.get(i);

    }

    cout<<"set camera Parameter over"<<endl;

}

//Distance: the distance of camera and object
//简单的长尺度转换，未考虑相机的畸变和旋转
double CvtRealLenghth2PixelLenghth(double RealLenghth_mm, double Distance_mm)
{
    double PixelLenghth_mm = 0;
    PixelLenghth_mm = Camera_fxy / Distance_mm * RealLenghth_mm;
    return PixelLenghth_mm;
}

//Distance: the distance of camera and object
double CvtPixelLenghth2RealLenghth(double PixelLenghth, double Distance_mm)
{
    double RealLenghth = 0;
    RealLenghth = Distance_mm * PixelLenghth / Camera_fxy;
    return RealLenghth;
}

//计算云台需要转的Yaw和Pitch使得摄像头的中轴线到指定点
Point2f CaculatePitchYawError(float Pixel_x, float Pixel_y)
{
    float PitchAngle = 0;
    float YawAngle = 0;
    float tan_pitch = (Pixel_y - myVideoCaptureProperties[CAP_PROP_FRAME_HEIGHT] / 2) / Camera_fy;
    float tan_yaw = (Pixel_x - myVideoCaptureProperties[CAP_PROP_FRAME_WIDTH] / 2) / Camera_fx;

    PitchAngle = atan(tan_pitch);
    YawAngle = atan(tan_yaw);

    PitchAngle = -PitchAngle / 3.14 * 180; //转化成单位度
    YawAngle = -YawAngle / 3.14 * 180; //转化成单位度
    return Point2f(YawAngle, PitchAngle);
}






//Functions Declare
void Data_disintegrate_u16(unsigned int Data, unsigned char *LData,
                       unsigned char *HData);
void Data_disintegrate_s16(int Data, unsigned char *LData,
                       unsigned char *HData);

unsigned char Add_CRC(unsigned char InputBytes[], unsigned char data_lenth) ;
void Data_Code( int x_Data,  int y_Data);
void sel_send();
string num2str(double i);


int status;
int  capIdx = 1;
#ifdef TX2
    // vector<int> CalculateXYPixel(Rect location);
    int messnum = 0;
    bool transmit_message = 1;
    Serial sel;
    unsigned char data_send_buf[8];
#endif // TX2




    void frame()
    {
        VideoCapture capture;
        if (camWay == 1)
        {
          capture.open(0);
          if (!capture.isOpened())
              {
                cout << "Loading error!\n";
                abort();
              }
          else
              {
                 cout << "Loading success!\n";
              }
        }

        Mat frame;

        while(1)
        {
            capture >> frame;
            imshow("video_camera",frame);

           // sel_send();
            waitKey(1);


        }


    }



string num2str(double i)
{
	stringstream ss;
	ss << i;
	return ss.str();
}

#ifdef open_serial
////--------------------------------------
unsigned char Add_CRC(unsigned char InputBytes[], unsigned char data_lenth) {
  unsigned char byte_crc = 0;
  for (unsigned char i = 0; i < data_lenth; i++) {
    byte_crc += InputBytes[i];
  }
  return byte_crc;
}

void Data_disintegrate_u16(unsigned int Data, unsigned char *LData,  unsigned char *HData) {
  *LData = Data & 0XFF;          // 0xFF = 1111 1111
  *HData = (Data & 0xFF00) >> 8; // 0xFF00 = 1111 1111 0000 0000
}

#define BYTE0(dwTemp) (*((char*)&(dwTemp)))
#define BYTE1(dwTemp) (*((char*)&(dwTemp)+1))
#define BYTE2(dwTemp) (*((char*)&(dwTemp)+2))
#define BYTE3(dwTemp) (*((char*)&(dwTemp)+3))

void Data_disintegrate_s16(int Data, unsigned char *LData,unsigned char *HData)
{
   *LData=BYTE0(Data);
   *HData=BYTE1(Data);
}
//

void Data_Code( int x_Data,  int y_Data) {
  int length = 8;
  data_send_buf[0] = 0xFF;
  data_send_buf[1] = length;
  data_send_buf[2] = 0x02;
  //
  Data_disintegrate_s16(x_Data, &data_send_buf[3], &data_send_buf[4]);
  Data_disintegrate_s16(y_Data, &data_send_buf[5], &data_send_buf[6]);
  //
  data_send_buf[length - 1] = Add_CRC(data_send_buf, length - 1);
}
////----------------------




void sel_send() {
  while (1) {
#ifdef TX2
    // transmit msg


#else
    // cout << "vs_test" << endl;

#endif // TX2
    this_thread::sleep_for(chrono::milliseconds(1));
  }
}


void SendDataToInfantry()
{

   // PitchYawError.x=-9.2;
    //PitchYawError.y=-8.2;
    char buff[8];

    Data_Code( (int)(SendYawPitchError.x*1000),  (int)(SendYawPitchError.y*1000));
    for (int i = 0; i < 8; i++)
        buff[i] = data_send_buf[i];
    sel.writeData(buff, 8);


}
#endif //open_serial
