#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, RegionOfInterest
from object_detect.msg import Center_msg
from std_msgs.msg import Int64
from std_msgs.msg import Int32MultiArray

import numpy as np
import time
import math as m


class Findposition:
    def __init__(self):
        self.tag1 = 0
        rospy.on_shutdown(self.cleanup);

        # 初始化ROS节点
        rospy.init_node("object_detect", anonymous=True)

        # 创建cv_bridge话题
        self.bridge = CvBridge()

        # 创建图像发布话题
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        # 创建识别物体中心坐标的发布话题
        self.pub = rospy.Publisher('chatter', Center_msg, queue_size=10)   
        
        rate = rospy.Rate(10) # 发布频率为10hz

        #   订阅语音识别的话题
        rospy.Subscriber("voice/object_color", Int32MultiArray, self.getColor)


        rospy.spin()
        while not rospy.is_shutdown():
            # self.move_base.wait_for_result()
            rospy.sleep(1)

    #   从语音里识别要抓取的物体的id
    def getColor(self,data):
        self.get_color = data.data
        rospy.loginfo("object_detect is started.. \n Please subscribe the ROS image.")
        #   机械臂初始化位姿成功时，进行图像识别
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        # 延时1s
        time.sleep(1)

        #   发布识别成功的信号
        if self.tag1 == 8 and self.center_ob != [0, 0]:
            self.pub.publish(self.center_ob)
            print(self.center_ob)
        else:
            pass


#####################      获取物体的中心坐标回调函数　　   ######################
    def image_callback(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            frame = np.array(self.cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

        # 将图像从RGB转成灰度图
        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  
        #　将图像从RGB转成HSV
        self.hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

        #   识别物体并获取其像素中心坐标
        morph = self.Get_contour_Color()
        Color = self.Get_Color(self.get_color)

        points = self.Find_contour_Color(morph)
        mask = self.Draw_contour(points)
        center_x, center_y = self.Get_center(points)

        self.center_ob = [center_x, center_y, self.s]
        self.tag1 = 8

        #while not rospy.is_shutdown():      #用于检测程序是否退出，是否按Ctrl-C 或其他
            # rospy.loginfo("center is %s", center_ob)  #在屏幕输出识别的物体中心位置坐标
        #self.pub.publish(self.center_ob)      #发布信息到主题
        #self.pub1.publish(self.tag1)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))

            #rate.sleep()        #睡眠一定持续时间，如果参数为负数，睡眠会立即返回

    #   获取颜色的阈值范围
    def Get_Color(self, get_color):
        if self.get_color == 0:
            low_Color = np.array([156, 43, 46])  #粉红
            high_Color = np.array([180, 255, 255])
        elif self.get_color == 1:
            low_Color = np.array([100,50,50])     #蓝色
            high_Color = np.array([140,255,255])
        elif self.get_color == 2:
            low_Color = np.array([35, 43, 46])    #绿色
            high_Color = np.array([77, 255, 255])
        elif self.get_color == 3:
            low_Color = np.array([11, 43, 46])  #黄色
            high_Color = np.array([25, 255, 255])
        elif self.get_color == 4:
            low_Color = np.array([125, 43, 46]) #紫色
            high_Color = np.array([155, 255, 255])
        elif self.get_color == 5:
            low_Color = np.array([0, 43, 46])   #橙色
            high_Color = np.array([10, 255, 255])

        mask = cv2.inRange(self.hsv, low_Color, high_Color)
        Color = cv2.bitwise_and(self.hsv, self.hsv,mask=mask)
        return Color


    #将区域进行二值化处理 
    def Get_contour_Color(self):
        #change to gray
        Color = self.Get_Color(self.get_color)
        Color_gray = cv2.cvtColor(Color, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(Color_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_Color(self,frame):
        img_cp = self.Get_contour_Color()
        _, cnts, _ = cv2.findContours(img_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(cnts) == 0:
            img_boxpoints = cnts
        else:
            cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0] #当没有检测到图像的时候报错，要修改
            box =cv2.minAreaRect(cnt_second)    #生成最小外接矩形
            img_boxpoints = np.int0(cv2.boxPoints(box))  #返回最小外接矩形4 个顶点
            self.pxPoint(img_boxpoints)

            # print img_boxpoints
        return img_boxpoints

    #绘制轮廓
    def Draw_contour(self,points):
        mask = np.zeros(self.gray.shape,np.uint8)
        if len(points) == 0:
            pass
        else:
            cv2.drawContours(mask,[points],-1,255,2)
        return mask

    #获取中心位置
    def Get_center(self,points):
        # global center
        if len(points) == 0:
            cen_tag = 0
            center = (0, 0)
        else:
            cen_tag = 1
            p1x,p1y=points[0,0],points[0,1]
            p3x,p3y=points[2,0],points[2,1]
            center_x,center_y=(p1x+p3x)/2,(p1y+p3y)/2
            center=(center_x,center_y)
        return center

    #绘制中心点
    def Draw_center(self,center,mask):
        # global mask1        
        if cen_tag == 0:
            pass
        else:
            cv2.circle( mask,center,1,(255,255,255),2)


############################    计算像素与实际距离之比     ##############################################     
    def pxPoint(self, data):
        st = []
        for i in range(0, 4):
            if i <= 2:
                s = m.sqrt((data[i][0] - data[i+1][0])**2 + (data[i][1] - data[i+1][1])**2)
                st = np.append(st, s)
            else:
                s = m.sqrt((data[i][0] - data[0][0])**2 + (data[i][1] - data[0][1])**2)
                st = np.append(st, s)
        sum = 0
        for x in st:
            sum = sum + x
        self.s = sum/len(st)
        print self.s
        return self.s


    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()




################################################################################            
if __name__== '__main__' :
    # global center_x, center_y
    try:
        Findposition()
    except rospy.ROSInterruptException:
        rospy.loginfo("object_detect test finished.")
    
    rospy.sleep(5)
