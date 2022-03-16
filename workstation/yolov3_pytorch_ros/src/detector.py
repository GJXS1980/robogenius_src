#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import division

# Python imports
import numpy as np
from numpy.core.records import array
import scipy.io as sio
import os, sys, cv2, time
from skimage.transform import resize

# ROS imports
import rospy
import std_msgs.msg
from rospkg import RosPack
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image,PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Polygon, Point32,Point
from yolov3_pytorch_ros.msg import BoundingBox, BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError

package = RosPack()
package_path = package.get_path('yolov3_pytorch_ros')  # 获取功能包所在路径
# print("package_path: %s"%package_path)
# /home/pickle/melodic_package/jetsim_pro/jetyolo_ws/src/yolov3_pytorch_ros

# Deep learning imports
import torch
from torch.utils.data import DataLoader
from torchvision import datasets
from torch.autograd import Variable

reload(sys)
sys.setdefaultencoding('utf8')


from models import *    # 加载训练模型
from utils.utils import *

# my_pcl = PointCloud2()
cx = 0
cy = 0
target_x = 0
target_y = 0
deal_flag = 1

# Detector manager class for YOLO
class DetectorManager():
    def __init__(self):
        # Load weights parameter
        weights_name = rospy.get_param('~weights_name', 'yolov3.weights')   # 从服务器获取权重的名字
        self.weights_path = os.path.join(package_path, 'models', weights_name)  # 字符串拼接
        # /home/pickle/melodic_package/jetsim_pro/jetyolo_ws/src/yolov3_pytorch_ros/models/yolov3.weights
        rospy.loginfo("Found weights, loading %s", self.weights_path)

        # Raise error if it cannot find the model
        if not os.path.isfile(self.weights_path):
            raise IOError(('{:s} not found.').format(self.weights_path))

        # Load image parameter and confidence threshold 加载相机参数（话题）及置信度阈值 0.7
        self.image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        #print("1---------------------------------------\n")
        self.confidence_th = rospy.get_param('~confidence', 0.7)
        self.nms_th = rospy.get_param('~nms_th', 0.3)

        # Load publisher topics 从参数服务器加载要发布的话题名称
        self.detected_objects_topic = rospy.get_param('~detected_objects_topic')
        self.published_image_topic = rospy.get_param('~detections_image_topic')

        # Load other parameters 从参数服务器加载其它参数：yolo配置文件，分类名字
        config_name = rospy.get_param('~config_name', 'yolov3.cfg')
        self.config_path = os.path.join(package_path, 'config', config_name)
        classes_name = rospy.get_param('~classes_name', 'screw.names')
        self.classes_path = os.path.join(package_path, 'classes', classes_name)
        self.gpu_id = rospy.get_param('~gpu_id', 0)
        self.network_img_size = rospy.get_param('~img_size', 608)
        self.publish_image = rospy.get_param('~publish_image') # 标志位 是否发布话题

        # Initialize width and height 初始化宽度和高度
        self.h = 0
        self.w = 0
        #print("2---------------------------------------\n")

        # Load net 加载网络
        self.model = Darknet(self.config_path, img_size=self.network_img_size)

        #print("3---------------------------------------\n")
        self.model.load_weights(self.weights_path)
        #print("4---------------------------------------\n")
        if torch.cuda.is_available():
            print("cuda\n")
            self.model.cuda()
            print("cuda\n")
        else:
            raise IOError('CUDA not found.')
        print("continue")
        self.model.eval() # Set in evaluation mode 设置为评价模式
        #print("5---------------------------------------\n")
        rospy.loginfo("Deep neural network loaded") #日志输出提示 深度神经网络加载成功

        #print("6---------------------------------------\n")
        # Load CvBridge 加载 CvBridge
        self.bridge = CvBridge()
        #print("---------------------------------------\n")
        # Load classes  加载分类
        self.classes = load_classes(self.classes_path) # Extracts class labels from file
        self.classes_colors = {}

        # Define subscribers    # 定义订阅者，订阅相机的话题消息， imageCb 为回调函数
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.imageCb, queue_size = 1, buff_size = 2**24)
        self.dep_sub = rospy.Subscriber("/realsense/depth/points",PointCloud2,self.depthcallback, queue_size = 1)

        # Define publishers     # 定义发布者发布识别对象的像素坐标以及发布处理后的识别图像
        self.pub_ = rospy.Publisher(self.detected_objects_topic, BoundingBoxes, queue_size=10)
        self.pub_viz_ = rospy.Publisher(self.published_image_topic, Image, queue_size=10)
        self.pub_pos = rospy.Publisher("/detected_objects_pose",Point,queue_size=10)
        rospy.loginfo("Launched node for object detection")

        # Spin
        rospy.spin()

    def imageCb(self, data):    # 相机图像回调函数
        global cx
        global cy
        global deal_flag
        count = 0
        # Convert the image to OpenCV   转换图像为 Opencv可操作图像
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

        # Initialize detection results  初始化发布识别结果的话题消息 BoundingBoxes()
        detection_results = BoundingBoxes()
        detection_results.header = data.header
        detection_results.image_header = data.header

        # Configure input   配置输入
        input_img = self.imagePreProcessing(self.cv_image)  # 图像预处理
        input_img = Variable(input_img.type(torch.cuda.FloatTensor))

        # Get detections from network   通过网络获取检测结果
        with torch.no_grad():
            detections = self.model(input_img)
            detections = non_max_suppression(detections, 80, self.confidence_th, self.nms_th) # 非极大值抑制

        # Parse detections  解析检测
        if detections[0] is not None:
            for detection in detections[0]:
                # Get xmin, ymin, xmax, ymax, confidence and class
                xmin, ymin, xmax, ymax, _, conf, det_class = detection
                pad_x = max(self.h - self.w, 0) * (self.network_img_size/max(self.h, self.w))
                pad_y = max(self.w - self.h, 0) * (self.network_img_size/max(self.h, self.w))
                unpad_h = self.network_img_size-pad_y
                unpad_w = self.network_img_size-pad_x
                xmin_unpad = ((xmin-pad_x//2)/unpad_w)*self.w
                xmax_unpad = ((xmax-xmin)/unpad_w)*self.w + xmin_unpad
                ymin_unpad = ((ymin-pad_y//2)/unpad_h)*self.h
                ymax_unpad = ((ymax-ymin)/unpad_h)*self.h + ymin_unpad
                w = xmax_unpad - xmin_unpad
                h = ymax_unpad - ymin_unpad

                # Populate darknet message
                detection_msg = BoundingBox()
                detection_msg.xmin = xmin_unpad
                detection_msg.xmax = xmax_unpad
                detection_msg.ymin = ymin_unpad
                detection_msg.ymax = ymax_unpad
                detection_msg.cx = (xmax_unpad - xmin_unpad)/2 + xmin_unpad
                detection_msg.cy = (ymax_unpad - ymin_unpad)/2 + ymin_unpad

                if detection_msg.cx > 50:
                    if (count == 0)&(deal_flag==1):
                        cx = detection_msg.cx
                        cy = detection_msg.cy
                        #print("get cx %d,cy %d"%(cx,cy))
                        count += 1
                        deal_flag = 0

                if w > h:
                    detection_msg.degree = True
                else:
                    detection_msg.degree = False
                detection_msg.probability = conf
                detection_msg.Class = self.classes[int(det_class)]

                # Append in overall detection message
                detection_results.bounding_boxes.append(detection_msg)

        # Publish detection results
        self.pub_.publish(detection_results)
        

        # Visualize detection results
        if (self.publish_image):
            self.visualizeAndPublish(detection_results, self.cv_image)
        return True

    def depthcallback(self,msg):
        global cx
        global cy
        global target_x
        global target_y
        global deal_flag
        # my_pcl = PointCloud2()
        # my_pcl.height
        # print("row_step,point_step,height",msg.row_step,msg.point_step,msg.height) #  20480*32*480   9830400
        # print("deal1 cx %d,cy%d"%(cx,cy))
        # print("deal1 row_step %d,point_step%d"%(msg.row_step,msg.point_step))
        arrayposition = int(cy)*int(msg.row_step) + int(cx)*int(msg.point_step)          # data = height*row_step/32 307200
        #print(arrayposition)
        # arrayposition = int(cy*msg.row_step)
        # print("deal2 cx %d,cy%d"%(cx,cy))
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)   # 307200
        count = 0
        for p in gen:
            if count == int(int(arrayposition)/32):
                target_x = 1.103+p[1]
                target_y = -1.057+p[0]
                # print(count,arrayposition)
                #print (" x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2]))
                deal_flag = 1
            count+=1
        # print(count)
        pos = Point()
        pos.x = target_x
        pos.y = target_y
        self.pub_pos.publish(pos)
        # 612,242
        # global cx
        # global cy
        # # x = 431
        # # y = 238
        # print("row_step,point_step",msg.row_step,msg.point_step) #  20480*32
        # arrayposition = int(y*msg.row_step + x*msg.point_step)
        # x_position = arrayposition + msg.fields[0].offset
        # y_position = arrayposition + msg.fields[1].offset
        # z_position = arrayposition + msg.fields[2].offset
        # # my_pcl = PointCloud2()
        # print (" x : %d  y: %d  z: %d" %(int(x_position),int(y_position),int(z_position)))
        # print(type(msg.data[int(x_position)]))
        # print(msg.data[int(x_position)])
        # x = msg.data[int(x_position)]
        # y = int(msg.data[int(y_position)])
        # z = int(msg.data[int(z_position)])
        # print (" x : %d  y: %d  z: %d" %(x,y,z))


        # print (" x : %.3f  y: %.3f  z: %.3f" %(x,y,z))
        # gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)   # 307200
        # time.sleep(1)
        # gen = point_cloud2.read_points(msg)
        # count = 0
        # # print (type(gen))
        # for p in gen:
        # #     # print("wkr",int(arrayposition),"count",count)
        #     if count == int(int(arrayposition)/24):
        #         print (" x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2]))
        #         time.sleep(10)
        #     count+=1
            # print("count",int(int(arrayposition)/24),count)
        #     # print (" x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2]))
        # time.sleep(10)


    def imagePreProcessing(self, img):
        # Extract image and shape
        img = np.copy(img)
        img = img.astype(float)
        height, width, channels = img.shape # 480*640

        if (height != self.h) or (width != self.w):
            self.h = height
            self.w = width

            # Determine image to be used
            self.padded_image = np.zeros((max(self.h,self.w), max(self.h,self.w), channels)).astype(float)

        # Add padding
        if (self.w > self.h):
            self.padded_image[(self.w-self.h)//2 : self.h + (self.w-self.h)//2, :, :] = img
        else:
            #self.padded_image[:, (self.h) : self.w, :] = img
            self.padded_image[:, (self.h-self.w)//2 : self.w + (self.h-self.w)//2, :] = img

        # Resize and normalize
        input_img = resize(self.padded_image, (self.network_img_size, self.network_img_size, 3))/255.

        # Channels-first
        input_img = np.transpose(input_img, (2, 0, 1))

        # As pytorch tensor
        input_img = torch.from_numpy(input_img).float()
        input_img = input_img[None]

        return input_img


    def visualizeAndPublish(self, output, imgIn):   # 可视化发布 输入参数：识别结果，cv图像
        # Copy image and visualize  赋值图像与可视化
        imgOut = imgIn.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX     #获取字体效果
        fontScale = 0.6     # 字体比例系数
        thickness = 2       # 字体线粗
        for index in range(len(output.bounding_boxes)): # 从输出消息中依次读取识别到的目标对象
            label = output.bounding_boxes[index].Class  # 识别对象类别
            x_p1 = output.bounding_boxes[index].xmin    # 识别对象的x像素坐标最小值
            y_p1 = output.bounding_boxes[index].ymin    # 识别对象的y像素坐标最小值
            x_p3 = output.bounding_boxes[index].xmax    # 识别对象的x像素坐标最大值
            y_p3 = output.bounding_boxes[index].ymax    # 识别对象的y像素坐标最大值
            confidence = output.bounding_boxes[index].probability   # 获取识别对象的置信阈值
            w = int(x_p3 - x_p1)    # 计算识别对象的中心像素坐标
            h = int(y_p3 - y_p1)    
            center = (int(((x_p1)+(x_p3))/2),int(((y_p1)+(y_p3))/2))    # 识别对象的像素中心
            #print(center)
            # if w > h:   # 判断物体所在位置，根据 xy的关系修改矩形目标表示框的尺寸
            #     cv2.rectangle(imgOut, (int(center[0])-5, int(center[1])-20), (int(center[0])+5, int(center[1])+20), (0,255,0),2)
            # elif w < h:
            #     cv2.rectangle(imgOut, (int(center[0])+20, int(center[1])+5), (int(center[0])-20, int(center[1])-5), (0,255,0),2)
            cv2.rectangle(imgOut, (int(center[0])+10, int(center[1])+10), (int(center[0])-10, int(center[1])-10), (0,255,0),2)

            # Find class color
            if label in self.classes_colors.keys():
                color = self.classes_colors[label]
            else:
                # Generate a new color if first time seen this label
                color = np.random.randint(0,198,3)
                #color[0] = (255,255,255)
                #color[1] = (198,198,198)
                #color[2] = (27,27,27)

                self.classes_colors[label] = color

            # Create rectangle
            #cv2.rectangle(imgOut, (int(x_p1), int(y_p1)), (int(x_p3), int(y_p3)), (color[0],color[1],color[2]),2)
            #print(color)
            text = ('{:s}: {:.3f}').format(label,confidence)
            cv2.putText(imgOut, text, (int(x_p1), int(y_p1-10)), font, fontScale, (0,0,0), thickness ,cv2.LINE_AA)

            # Create center
            cv2.circle(imgOut,(center), 2, (255,255,0), 4)
        # Publish visualization image
        image_msg = self.bridge.cv2_to_imgmsg(imgOut, "rgb8")
        self.pub_viz_.publish(image_msg)


if __name__=="__main__":
    # Initialize node   初始化 yolo 识别节点 detector_manager_node
    rospy.init_node("detector_manager_node")
    # Define detector object    定义检测对象类
    dm = DetectorManager()
