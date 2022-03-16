#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv_object_tracking/position_publish.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/String.h"
#include<sstream>
// Add new topic
#include "geometry_msgs/Point.h"
#include <iostream>
#include <vector>
// Create new for circle drawing
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cstdlib>
#include <string>


// Add global variable for pixel.
bool flag;
bool blue;
bool yellow;
bool green;
int posX = 0;
int posY = 0;
float X_111 = 0.0;
float Y_111 = 0.0;
float Z_111 = 0.0;
float x_value, y_value, z_value;
float k,kx,ky;
int x_position, y_position, z_position;
//End of global variable.
cv_bridge::CvImagePtr cv_ptr;
sensor_msgs::PointCloud2 my_pcl;
using namespace std;
using namespace cv;
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
int rec_topic;
//Publishe new topic
ros::Publisher *pub;
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
int wide_min=0;
int heigh_min=0;
int wide_max=FRAME_WIDTH;
int heigh_max=FRAME_HEIGHT;
const int MAX_NUM_OBJECTS = 50;
const int MIN_OBJECT_AREA = 5 * 5;    //defult 20
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;
static const std::string OPENCV_WINDOW = "Image Window";
static const std::string windowName1 = "HSV image";
static const std::string windowName2 = "Thresholded Image";
static const std::string windowName3 = "After Morphological Operations";
static const std::string trackbarWindowName = "Track bars";
static const std::string select_area= "select area";
int hide_pic=0;
int depth=0;

Mat threshold_z = Mat::zeros(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
int color_type; 
string color;
void on_trackbar(int, void*){}


string intToString(int number)
{
        std::stringstream ss;
        ss << number;
        return ss.str();
}


void adjust_area()
{
        //Create window for trackbars
        namedWindow("select area", 0);
        //char TrackbarName[50];
        createTrackbar("wide_min", "select area", &wide_min, wide_max, on_trackbar); //opencv创建滑动控件
        createTrackbar("wide_max", "select area", &wide_max, wide_max, on_trackbar);
        createTrackbar("heigh_min", "select area", &heigh_min, heigh_max, on_trackbar);
        createTrackbar("heigh_max", "select area", &heigh_max, heigh_max, on_trackbar);
        createTrackbar("show_HSV", "select area", &hide_pic, 1, on_trackbar);
        createTrackbar("depth", "select area", &depth, 618, on_trackbar);
}




void drawObject(int x, int y, Mat &frame)
{
        circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2); //50     //opencv 画圆
        //圆心画十字
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
        putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);   //opencv
        ::posX = x;
        ::posY = y;
        putText(frame, "X_Y_Z coordinate", Point(20, 200), 1, 2, Scalar(0, 255, 0), 2);
        putText(frame, "X = " + intToString(x_position) + "(mm)" , Point(20, 250), 1, 2, Scalar(0, 255, 0), 2);
        putText(frame, "Y = " + intToString(y_position) + "(mm)" , Point(20, 300), 1, 2, Scalar(0, 255, 0), 2);
        putText(frame, "Z = " + intToString(z_position) + "(mm)" , Point(20, 350), 1, 2, Scalar(0, 255, 0), 2);
        putText(frame, "color :" + color  , Point(20, 400), 1, 2, Scalar(0, 255, 0), 2);

}
void morphOps(Mat &thresh)
{

        //para of erode and dilate
        // Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
        // Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8,8));
        // erode(thresh, thresh, erodeElement); //腐蚀
        // erode(thresh, thresh, erodeElement);
        // imshow("erode", thresh);
        // dilate(thresh, thresh, dilateElement);  //膨胀
        // dilate(thresh, thresh, dilateElement);
        Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
        Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

        //erode(thresh, thresh, erodeElement);
        //erode(thresh, thresh, erodeElement);

        dilate(thresh, thresh, dilateElement);
        dilate(thresh, thresh, dilateElement);
        dilate(thresh, thresh, dilateElement);
        dilate(thresh, thresh, dilateElement);
}


//获取主要方向
void getOrientation(vector<Point> &pts, Mat &img)
{       
        Point pos_s,pos_e,pos_e1;  
        pos_s.x=posX;
        pos_s.y=posY;
        vector<RotatedRect> box(1); //定义最小外接矩形集合
        Point2f rect[4];
        box[0] = minAreaRect(pts);
        box[0].points(rect);
        for(int j=0; j<4; j++)
        {
        line(img, rect[j], rect[(j+1)%4], Scalar(0, 0, 255), 2, 8);  //绘制最小外接矩形每条
        }
        float a,b;
         a=sqrt((rect[0].x-rect[1].x)*(rect[0].x-rect[1].x)+(rect[0].y-rect[1].y)*(rect[0].y-rect[1].y));
         b=sqrt((rect[1].x-rect[2].x)*(rect[1].x-rect[2].x)+(rect[1].y-rect[2].y)*(rect[1].y-rect[2].y));
         if (a>b)
         {
                 kx=(rect[0].x-rect[1].x);
                 ky=(rect[0].y-rect[1].y);
         }
         else
         {
                 kx=(rect[1].x-rect[2].x);
                 ky=(rect[1].y-rect[2].y);
         }
         k=-atan(ky/kx)*180/3.1415926;
         cout<<"angle:"<<k<<endl;
        //draw the orientation
        pos_e.x=pos_s.x+200*cos(atan(ky/kx));
        pos_e.y=pos_s.y+200*sin(atan(ky/kx));
        pos_e1.x=pos_s.x+100*cos(atan(-kx/ky));
        pos_e1.y=pos_s.y+100*sin(atan(-kx/ky));
        line(img, pos_s,pos_e, Scalar(255, 255, 0), 2, 8);
        line(img, pos_s,pos_e1, Scalar(0, 255, 255), 2, 8);
}




void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed)
{       

        //draw the zone
        rectangle(cameraFeed, Point(wide_min, heigh_min), Point(wide_max, heigh_max), Scalar(255, 0, 0), 5, LINE_8, 0);
        Mat temp;
        threshold.copyTo(temp);
        //These two vectors needed for output of findContours
        vector< vector<Point> > contours;
        vector<Vec4i> hierarchy;
        //Find contours of filtered image using openCV findContours function
         findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        //Use moments method to find our filtered object
        bool objectFound = false;
        if (hierarchy.size() >0)
        {
                int numObjects = hierarchy.size();
                //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
                if (numObjects < MAX_NUM_OBJECTS)
                {       
                       //cout << "for_loop" <<endl;
                       for (int index = 0; index >= 0; index = hierarchy[index][0])
                        {
                                Moments moment = moments((cv::Mat)contours[index]);
                                double area = moment.m00; //m00是轮廓面积
                                //if the area is less than 20 px by 20px then it is probably just noise
                                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                                //we only want the object with the largest area so we safe a reference area each
                                //iteration and compare it to the area in the next iteration.
                                if (area > MIN_OBJECT_AREA && area<MAX_OBJECT_AREA )
                                {
                                        x = moment.m10 / area;
                                        y = moment.m01 / area;
                                        if((x>wide_min)&&(x<wide_max) && (y>heigh_min)&&(y<heigh_max))
                                        {
                                          cout << "****************"<<endl;
                                          objectFound = true;
                                          ::flag = true;
                                          ///add
                                          //   drawContours(cameraFeed, contours, index, CV_RGB(255, 0, 0), 2, 8, hierarchy, 0);
                                         //   getOrientation(contours[index], cameraFeed);
                                          getOrientation(contours[index], cameraFeed);
                                          //add
                                          break;
                                        }
                                        else
					{
					   objectFound = false;
                                           ::flag = false;
					}
                                }
                                else
                                {
                                 objectFound = false;
                                 ::flag = false;
                                }
                           }
                           cout << "flag" <<  ::flag<<endl;                      
                        //let user know you found an object
                        if (objectFound == true)
                        {
                                //::flag = true;
                                putText(cameraFeed, "Position Object tracking ", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                                //draw object location on screen
                                drawObject(x, y, cameraFeed);
                        }

                }
                else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
        }        
        else 
        {
                ::flag=false;
        }
}


int z_filter(int x, int y)
{

    int arrayPosition1 = y*my_pcl.row_step + x*my_pcl.point_step;
    int arrayPosZ1 = arrayPosition1 + my_pcl.fields[2].offset; // Z has an offset of 8
    float Z1 = 0.0;
   // float Z2=depth/1000.000;
    float Z2=618.0/1000.000;
    memcpy(&Z1, &my_pcl.data[arrayPosZ1], sizeof(float));
     if (Z1>Z2)
     {
    return 0;
    }
    else 
    {
        return 1;
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
	bool useMorphOps = true;  //false
	Mat HSV;
	Mat threshold;
	int x = 0, y = 0;
        adjust_area();
	// createTrackbars();

            H_MIN = 0 ;
            H_MAX = 11;
            S_MIN = 16;
            S_MAX = 256;
            V_MIN = 0;
            V_MAX = 256;
            color="...";
            color_type=1;
    std::cout << " The output of Object tracking by OpenCV!\n";
    //颜色空间转换
    cvtColor(cv_ptr->image, HSV, COLOR_BGR2HSV); 
    // imshow("hsv ",HSV);
    Mat threshold1;
    Mat threshold2;
    Mat threshold3;
    //二值化
    if (hide_pic==1)
    {
    inRange(HSV, Scalar(H_MIN, 0, 0), Scalar(H_MAX, 256, 256), threshold1);
    inRange(HSV, Scalar(0, S_MIN, 0), Scalar(256, S_MAX, 256), threshold2);
    inRange(HSV, Scalar(0, 0, V_MIN), Scalar(256, 256, V_MAX), threshold3);
    //imshow("H",threshold1);
    //imshow("S",threshold2); 
    //imshow("V",threshold3);
    }
    inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
   
 //imshow("HSV_threshold",threshold);
 //imshow("threshold  HSV",threshold);
 //imshow("threshold_z ",threshold_z);
for(int r=1;r<FRAME_HEIGHT;r++)
     {
             for(int c=1;c<FRAME_WIDTH;c++)
             { 
                     int pp1,pp2;
                     pp1=threshold.ptr(r)[c];
                     pp2=threshold_z.ptr(r)[c];

                     if (pp1>0 && pp2>0 )
                     {
                        threshold.ptr(r)[c] = 255;
                     }
                     else
                     {
                        threshold.ptr(r)[c] = 0;
                     }
                }          
     }
     //imshow("threshold  HSV+filter_z",threshold);

    if (useMorphOps)
        morphOps(threshold);
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
    int arrayPosition = y*my_pcl.row_step + x*my_pcl.point_step;
    int arrayPosX = arrayPosition + my_pcl.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + my_pcl.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + my_pcl.fields[2].offset; // Z has an offset of 8
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
//     cout<<"z"<<Z<<endl;
    ::X_111 = X;
    ::Y_111 = Y;
    ::Z_111 = Z;
    ::x_position = int(X*1000);
    ::y_position = int(Y*1000);
    ::z_position = int(Z*1000);

}


void depthcallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    my_pcl = *cloud_msg;
     for(int r=1;r<FRAME_HEIGHT;r++)
     {
             for(int c=1;c<FRAME_WIDTH;c++)
             { 
                     int z_flag=0;
                     z_flag=z_filter(c,r);
                     if(z_flag)
                     {
                        threshold_z.ptr(r)[c] = 255;
                     }
                     else
                     {
                        threshold_z.ptr(r)[c] = 0;
                     }
                }          
     }
     
//     cv::threshold(threshold_z, threshold_z, 127, 255, THRESH_BINARY);
    getXYZ(posX , posY);
    rec_topic=1;
}

int main(int argc, char** argv)
{

    ros::init (argc, argv, "image_converter");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/realsense_cobot/color/image_raw",1, imageCallback);
    ros::Subscriber dep;
    dep = nh.subscribe ("/realsense/depth/points", 1, depthcallback);
    //Publish new topic.
    ros::Publisher pub = nh.advertise<opencv_object_tracking::position_publish>("position_object", 1);
    //Set the loop period.
    ros::Rate loop_rate(5);

    opencv_object_tracking::position_publish msg;
    msg.counter = 0;
    //int count = 0;
    while ((ros::ok()))
      {
        //cout<<"before_ rec_topic="<<rec_topic<<endl;

       if ((flag == true))
       {
       int posX_1, posY_1;
       printf("Position in X coordinate X = %.4f\n", X_111);
       printf("Position in Y coordinate Y = %.4f\n", Y_111);
       printf("Position in Z coordinate Z = %.4f\n", Z_111);
       //int count = 0;
       posX_1 = posX;
       posY_1 = posY;
       msg.Position_XYZ.clear();
       msg.center_pixel_x = posX;
       msg.center_pixel_y = posY;
       msg.counter = 1;
       msg.color_type=color_type;
       msg.angle=k;
       geometry_msgs::Point Position_XYZ;
       Position_XYZ.x = X_111;
       Position_XYZ.y = Y_111;
       Position_XYZ.z = Z_111;
	if (isnan(X_111) || rec_topic==0)
{
	msg.counter=0;
}
       msg.Position_XYZ.push_back(Position_XYZ);
       pub.publish(msg);
       //++count;
       }

       if (flag == false)
       {
        //count = 0;
        msg.Position_XYZ.clear();
        msg.center_pixel_x = 0;
        msg.center_pixel_y = 0;
        msg.counter = 0;
        geometry_msgs::Point Position_XYZ;
        Position_XYZ.x = 0;
        Position_XYZ.y = 0;
        Position_XYZ.z = 0;
        msg.Position_XYZ.push_back(Position_XYZ);
        pub.publish(msg);

       }
        loop_rate.sleep();
        rec_topic=0;
      ros::spinOnce();

      }


    return 0;
}


