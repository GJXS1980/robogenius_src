/* Copyright by Huong Do Van - 10/11/2018
Any suggestion or advice, pls send via email: vanhuong.robotics@gmail.com
*/
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>  //图像处理
#include <opencv2/highgui/highgui.hpp>	//Gui界面
#include <opencv2/opencv.hpp>
#include "opencv_object_tracking/position_publish.h"
#include <sensor_msgs/PointCloud2.h>    //消息类型
#include <geometry_msgs/Point.h>	//消息类型
#include "std_msgs/String.h"
#include<sstream>			//字符串流
// Add new topic
#include "geometry_msgs/Point.h"
#include <cmath>
#include <iostream>
#include <vector>
// Create new for circle drawing
#include <image_transport/image_transport.h>	//图形传输
#include <cv_bridge/cv_bridge.h>	//CV桥
#include <sensor_msgs/image_encodings.h>	//图形编码
#include "eigen3/Eigen/Dense"
// Add global variable for pixel.
bool flag;
int posX = 0;
int posY = 0;
float X_111 = 0.0;
float Y_111 = 0.0;
float Z_111 = 0.0;
float x_value, y_value, z_value;
int x_position, y_position, z_position;
double object_ref_angle = 0;
//End of global variable.
cv_bridge::CvImagePtr cv_ptr;
sensor_msgs::PointCloud2 my_pcl;
using namespace std;
using namespace cv;
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 16; //16
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//Publishe new topic
ros::Publisher *pub;
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
const int MAX_NUM_OBJECTS = 50;
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

static const std::string OPENCV_WINDOW = "Image Window";
static const std::string windowName1 = "HSV image";
static const std::string windowName2 = "Thresholded Image";
static const std::string windowName3 = "After Morphological Operations";
static const std::string trackbarWindowName = "Track bars";

void on_trackbar(int, void*){}
string intToString(int number)
{
        std::stringstream ss;
        ss << number;
        return ss.str();
}

// 创建拖动条
void createTrackbars()
{
        //Create window for trackbars
        namedWindow("Track_bars", 0);
        char TrackbarName[50];
	//CV_EXPORTS int createTrackbar(const string& trackbarname, const string&winname,
	//	int*value, int count,TrackbarCallback onChange = 0,void* userdata = 0;
        createTrackbar("H_MIN", "Track_bars", &H_MIN, H_MAX, on_trackbar);    // Hue 色相
        createTrackbar("H_MAX", "Track_bars", &H_MAX, H_MAX, on_trackbar);    
        createTrackbar("S_MIN", "Track_bars", &S_MIN, S_MAX, on_trackbar);    // Saturation 饱和度
        createTrackbar("S_MAX", "Track_bars", &S_MAX, S_MAX, on_trackbar);
        createTrackbar("V_MIN", "Track_bars", &V_MIN, V_MAX, on_trackbar);    // Value 亮度
        createTrackbar("V_MAX", "Track_bars", &V_MAX, V_MAX, on_trackbar);
}

// 在图片上绘制图案标识
// 输入参数为 x y（像素坐标） 还有图像
void drawObject(int x, int y, Mat &frame)
{ 
        //cvCircle(CvArr* img, CvPoint center, int radius, CvScalar color, int thickness=1, int lineType=8, int shift=0)
        // img 源图像指针 center 圆心 radius 半径 color 颜色（RGB） thickness 正数表示粗细，否则为填充 line_Type 线条类型 shift 坐标和半径小数点位数
        circle(frame, Point(x, y), 40, Scalar(0, 255, 0), 2); //50

        // void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
        // img 源图像指针 pt1 线段起点 pt2 线段终点 color 颜色 thickness 正数表示粗细，否则为填充 line_Type 线条类型 shift 坐标和半径小数点位数
        // 在识别对象的中心分别构建往四个方向的线段形成十字标记
        if (y - 25 > 0)
                line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
        if (y + 25 < FRAME_HEIGHT)
                line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
        if (x - 25 > 0)
                line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
        if (x + 25 < FRAME_WIDTH)
                line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

        // 绘制参考坐标系
        putText(frame,"x(640)",Point(555, 380), 1, 1, Scalar(0, 255, 0), 2);
        putText(frame,"y(480)",Point(585, 450), 1, 1, Scalar(0, 255, 0), 2);
        line(frame,Point(550,430),Point(550,380),Scalar(0, 255, 0), 2);
        line(frame,Point(550,430),Point(600,430),Scalar(0, 255, 0), 2);

        // void putText( Mat& img, const string& text, Point org, int fontFace,double fontScale，  Scalar color, int thickness=1, int lineType=8 );
        // img 源图像指针 text 写入的内容 org 第一个字符左下角坐标 fontFace 字体类型 fontScale 字体大小 color 字体颜色 thickness 正数表示粗细，否则为填充 line_Type 线条类型
        putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
        ::posX = x;
        ::posY = y;
        // 相对于相机坐标系的 xyz
        putText(frame, "X_Y_Z coordinate", Point(20, 375), 1, 1, Scalar(0, 255, 0), 2);
        putText(frame, "X = " + intToString(x_position) + "(mm)" , Point(20, 400), 1, 1, Scalar(0, 255, 0), 2);
        putText(frame, "Y = " + intToString(y_position) + "(mm)" , Point(20, 425), 1, 1, Scalar(0, 255, 0), 2);
        putText(frame, "Z = " + intToString(z_position) + "(mm)" , Point(20, 450), 1, 1, Scalar(0, 255, 0), 2);

}


void drawOrintation(int x0, int y0, int x,int y,Mat &frame,int index)
{ 
    line(frame, Point(x0, y0), Point(x, y), Scalar(0, 0, 255), 2);
    putText(frame,std::to_string(index),Point(x+50, y), 1, 1, Scalar(0, 255, 0), 2);
}

void morphOps(Mat &thresh)
{
        Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
        Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

        // 腐蚀函数
        // 输入图像 输出图像 腐蚀操作的结构元素
        erode(thresh, thresh, erodeElement);
        erode(thresh, thresh, erodeElement);

        // 膨胀函数
        // 输入图像 输出图像 膨胀操作的结构元素
        dilate(thresh, thresh, dilateElement);
        dilate(thresh, thresh, dilateElement);
}

double calculate_angle(Eigen::Vector2d ref_vector,Eigen::Vector2d object_vector)
{
    // Vector2f v1,Vector2f v2;
    // double cosValNew=v1.dot(v2) /(v1.norm()*v2.norm()); //角度cos值
    // double angleNew = acos(cosValNew) * 180 / M_PI;     //弧度角
    double cosval = ref_vector.dot(object_vector)/(ref_vector.norm()*object_vector.norm());
    double angle = acos(cosval)*180/M_PI;
    return angle;
}

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed)
{
    Mat temp;
    threshold.copyTo(temp);
    //These two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //Find contours of filtered image using openCV findContours function
    // findContours( InputOutputArray image, OutputArrayOfArrays contours,OutputArray hierarchy, int mode,int method, Point offset=Point());  
    // image 图像 
    // contours 向量内每个元素保存了一组由连续的Point点构成的点的集合的向量，每一组Point点集就是一个轮廓。  
    // hierarchy分别表示第i个轮廓的后一个轮廓、前一个轮廓、父轮廓、内嵌轮廓的索引编号。

    // mode 定义轮廓检索模式
    //CV_RETR_EXTERNAL(只检测最外围轮廓，包含在外围轮廓内的内围轮廓被忽略) 
    //CV_RETR_LIST(检测所有的轮廓，包括内围、外围轮廓，但是检测到的轮廓不建立等级关 系，彼此之间独立，没有等级关系) 
    //CV_RETR_CCOMP(检测所有的轮廓，但所有轮廓只建立两个等级关系，外围为顶层，若外围内的内围轮廓还包含了其他的轮廓信息，则内围内的所有轮廓均归属于顶) 
    //CV_RETR_TREE(检测所有轮廓，所有轮廓建立一个等级树结构。外层轮廓包含内层轮廓，内层轮廓还可以继续包含内嵌轮廓)
    
    // method  定义轮廓的近似方法
    //CV_CHAIN_APPROX_NONE 保存物体边界上所有连续的轮廓点到contours向量内
    //CV_CHAIN_APPROX_SIMPLE 仅保存轮廓的拐点信息，把所有轮廓拐点处的点保存入contours 向量内，拐点与拐点之间直线段上的信息点不予保留
    //CV_CHAIN_APPROX_TC89_L1 使用teh-Chinl chain 近似算法
    //CV_CHAIN_APPROX_TC89_KCOS 使用teh-Chinl chain 近似算法
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //Use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;
    // Mat resultImage = Mat ::zeros(temp.size(),CV_8U);
    drawContours(cameraFeed, contours, -1, Scalar(0, 255, 0),2,8);
    if (hierarchy.size() > 0)
    {
        int numObjects = hierarchy.size();
        int wkr = contours.size();
        // printf("numObjects is %d\n",numObjects);
        // printf("contours is %d\n",wkr); 
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter

        // for(int i = 0;i<4;i++)
        // {
        //     printf("hierarchy is %d\n",hierarchy[3][i]);
        // }
        
        // drawObject(x04, y04, cameraFeed);    // 绘制目标标记图案

        if (numObjects < MAX_NUM_OBJECTS)
        {
            for (int index = 0; index >= 0; index = hierarchy[index][0])
            {
                // printf("**************\n");	
                // printf("MAX_OBJECT_AREA = %d\n",MAX_OBJECT_AREA);
                //printf("index = %d\n",index);
                // printf("index is %d\n",index);  // 0 6 12 18
                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;
                // printf("**************\n");
                // printf("area = %f\n",area);
                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if (area > MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea)
                {
                        x = moment.m10 / area;
                        y = moment.m01 / area;
                        objectFound = true;
                        ::flag = true;
                        //refArea = area;
                        // printf("object is find!");
                        // printf("index is %d,%d\n",index+3,hierarchy[index+1][0]);
                        
                }
                else
                {
                    objectFound = false;
                    ::flag = false;
                }
                if (objectFound == true)
                {
                        //::flag = true;
                        putText(cameraFeed, "Position Object tracking ", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                        //draw object location on screen
                        drawObject(x, y, cameraFeed);    // 绘制目标标记图案
                        if(hierarchy[index][0] >0)
                        {
                            for(int contours_num=index+3;contours_num<hierarchy[index][0];contours_num++)
                            {
                                // printf("wkr %d\n",hierarchy[18][0]);
                                Moments moment_num = moments((cv::Mat)contours[contours_num]);
                                double area_num = moment_num.m00;
                                double x_num = moment_num.m10/moment_num.m00;
                                double y_num = moment_num.m01/moment_num.m00;
                                double distance = sqrt(pow(abs(x_num-x),2)+pow(abs(y_num-y),2));
                                // printf("distance is %f\n",distance); // 7 21 27 7 is min
                                if(distance < 10)
                                {
                                    drawOrintation(x,y,x_num,y_num,cameraFeed,index);
                                    // calculate_angle(Eigen::Vector2d ref_vector,Eigen::Vector2d object_vector);
                                    Eigen::Vector2d ref_vector = {0,10};
                                    Eigen::Vector2d object_vector = {(x_num-x),(y_num-y)};
                                    object_ref_angle = calculate_angle(ref_vector,object_vector);
                                    // printf(" angle is %f\n",object_ref_angle);
                                }
                                    
                                    // printf("return the contours_num %d and index %d\n",contours_num,index);
                            }
                        }
                        else
                        {
                            for(int contours_num=index+3;contours_num<hierarchy.size();contours_num++)
                            {
                                // printf("wkr %d\n",hierarchy[18][0]);
                                Moments moment_num = moments((cv::Mat)contours[contours_num]);
                                double area_num = moment_num.m00;
                                double x_num = moment_num.m10/moment_num.m00;
                                double y_num = moment_num.m01/moment_num.m00;
                                double distance = sqrt(pow(abs(x_num-x),2)+pow(abs(y_num-y),2));
                                // printf("distance is %f\n",distance); // 7 21 27 7 is min
                                if(distance < 10)
                                {
                                    drawOrintation(x,y,x_num,y_num,cameraFeed,index);
                                    Eigen::Vector2d ref_vector = {0,10};
                                    Eigen::Vector2d object_vector = {(x_num-x),(y_num-y)};
                                    object_ref_angle = calculate_angle(ref_vector,object_vector);
                                    // printf(" angle is %f\n",object_ref_angle);
                                }
                                    // printf("return the contours_num %d and index %d\n",contours_num,index);
                                
                            }
                        }
                }
            }
            //let user know you found an object
            // if (objectFound == true)
            // {
            //         //::flag = true;
            //         putText(cameraFeed, "Position Object tracking ", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
            //         //draw object location on screen
            //         drawObject(x, y, cameraFeed);    // 绘制目标标记图案
            // }

        }
        else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
            cv_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
    }
    bool trackObjects = true; //false
    bool useMorphOps = true;  //false 标志位，是否进行腐蚀膨胀操作
    Mat HSV;
    Mat threshold;
    int x = 0, y = 0;
    // createTrackbars();  //  创建拖动条界面
    // std::cout << " The output of Object tracking by OpenCV!\n";
    cvtColor(cv_ptr->image, HSV, COLOR_BGR2HSV);
    Mat threshold1,threshold2,threshold3;
    // inRange(HSV, Scalar(H_MIN, 0, 0), Scalar(H_MAX, 256, 256), threshold1);
    // imshow("H",threshold1);

    // inRange(HSV, Scalar(0, S_MIN, 0), Scalar(256, S_MAX, 256), threshold2);
    // imshow("S",threshold2);

    // inRange(HSV, Scalar(0, 0, V_MIN), Scalar(256, 256, V_MAX), threshold3);
    // imshow("V",threshold3);

    // inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
    // imshow("HSV",threshold);

    // inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
    inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);

    // if (useMorphOps)
    //     morphOps(threshold);

    if (trackObjects)
        trackFilteredObject(x, y, threshold, cv_ptr->image);
    //show frames
    //imshow(windowName2, threshold);
    //imshow(OPENCV_WINDOW, cameraFeed);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //imshow(windowName1, HSV);
    cv::waitKey(3);

}

void getXYZ(int x, int y)
{
    // printf("row_step %d,%d",my_pcl.row_step,my_pcl.point_step);
    int arrayPosition = y*my_pcl.row_step + x*my_pcl.point_step;
    int arrayPosX = arrayPosition + my_pcl.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + my_pcl.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + my_pcl.fields[2].offset; // Z has an offset of 8

    printf("array %d,%d,%d",arrayPosX,arrayPosY,arrayPosZ);
    printf("data %d,%d,%d",my_pcl.data[arrayPosX],my_pcl.data[arrayPosY],my_pcl.data[arrayPosZ]);

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    geometry_msgs::Point p;

    memcpy(&X, &my_pcl.data[arrayPosX], sizeof(float));
    memcpy(&Y, &my_pcl.data[arrayPosY], sizeof(float));
    memcpy(&Z, &my_pcl.data[arrayPosZ], sizeof(float));

    p.x = X;
    p.y = Y;
    p.z = Z;
    ::X_111 = X;
    ::Y_111 = Y;
    ::Z_111 = Z;
    ::x_position = int(X*1000);
    ::y_position = int(Y*1000);
    ::z_position = int(Z*1000);

    //printf("Position in X coordinate X = %.4f\n", X);
    //printf("Position in Y coordinate Y = %.4f\n", Y);
    //printf("Position in Z coordinate Z = %.4f\n", Z);

    //return 0;
}

void depthcallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    my_pcl = *cloud_msg;   // 获取相机点云数据
    getXYZ(posX , posY);   // 调用函数获取
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "image_converter_object");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/realsense_metal/color/image_raw",1, imageCallback);
    ros::Subscriber dep;
    // 深度话题订阅与回调函数
    dep = nh.subscribe ("/realsense_metal/depth/points", 1, depthcallback);
    // Publish new topic. 发布识别对象的位置
    ros::Publisher pub = nh.advertise<opencv_object_tracking::position_publish>("position_object_two", 1);
    // Set the loop period with 0.1 second.
    ros::Rate loop_rate(10);
    // 定义话题发布的消息
    opencv_object_tracking::position_publish msg;
    msg.counter = 0;
    // int count = 0;
    while ((ros::ok()))
        {
            // if ((posX != 0) && (posY != 0) && (flag == true))
            // int count = 0;
            // flag 为识别标志位，识别到对象时为真
            if ((flag == true))
            {
                // 定义整形变量用于存储坐标
                // int posX_1, posY_1;
                // printf("Position in X coordinate X = %.4f\n", X_111);
                // printf("Position in Y coordinate Y = %.4f\n", Y_111);
                // printf("Position in Z coordinate Z = %.4f\n", Z_111);
                // int count = 0;
                // posX posY 为全局变量，像素坐标
                // posX_1 = posX;
                // posY_1 = posY;
                msg.Position_XYZ.clear();
                msg.center_pixel_x =  posX;
                msg.center_pixel_y =  posY;
                msg.angle = object_ref_angle;
                msg.counter = 1;

                geometry_msgs::Point Position_XYZ;       //定义Point 数据类型存储目标位置变量
                Position_XYZ.x = 1.417 - X_111;          //X_111为相对于相机坐标系的物体位置，前面的补偿为转到世界坐标系
                Position_XYZ.y = -1.541 + Y_111;
                Position_XYZ.z = Z_111;
                msg.Position_XYZ.push_back(Position_XYZ); 

                pub.publish(msg);  // 发布话题消息内容
                loop_rate.sleep(); // 延时睡眠
                //++count;
            }
            // 未识别到目标对象
            if (flag == false)
            {
                //count = 0;
                msg.Position_XYZ.clear();
                msg.center_pixel_x = 0;
                msg.center_pixel_y = 0;
                msg.counter = 0;

                geometry_msgs::Point Position_XYZ;      //定义Point 数据类型存储目标位置变量
                Position_XYZ.x = 0;
                Position_XYZ.y = 0;
                Position_XYZ.z = 0;
                msg.Position_XYZ.push_back(Position_XYZ);

                pub.publish(msg);    // 发布话题消息内容
                loop_rate.sleep();   // 延时睡眠
            }
        ros::spinOnce();
        }
    return 0;
}

