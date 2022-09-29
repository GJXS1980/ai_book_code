#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math as m
import time



class Findposition:
    def __init__(self, img, get_color):
    	self.get_color = get_color

    #获取图片
        #self.img=cv2.imread(path)
        self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 将图像从RGB转成灰度图
        self.hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #　将图像从RGB转成HSV

#####################################################################################################
#######################      获取黑色区域　　   #############################
#####################################################################################################
 #   获取颜色的阈值范围
    def Get_Color(self):
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
        Color = self.Get_Color()
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
            # print img_boxpoints
            self.pxPoint(img_boxpoints)

        return img_boxpoints

################################################################################ 
#######################      获取轮廓的中心点坐标　　   #############################
################################################################################ 
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
            self.cen_tag = 0
            center = (0, 0)
        else:
            self.cen_tag = 1
            p1x,p1y=points[0,0],points[0,1]
            p3x,p3y=points[2,0],points[2,1]
            center_x,center_y=(p1x+p3x)/2,(p1y+p3y)/2
            center=(center_x,center_y)
        return center

    #绘制中心点
    def Draw_center(self,center,mask):
    	cv2.circle(mask,center,1,(255,255,255),2)
        # if self.cen_tag == 0:
        #     pass
        # else:
        #     cv2.circle( mask,center,1,(255,255,255),2)

################################################################################ 
#######################      识别红色１区域及中心点坐标主程序　　   ######################
################################################################################     #主函数

    def main_process_color(self):
		color = self.Get_Color()
		morph = self.Get_contour_Color()
		points = self.Find_contour_Color(morph)
		mask = self.Draw_contour(points)
		center = self.Get_center(points)

		cv2.imshow('img',img)

		if self.cen_tag == 0:
			pass
		else:
			center_x,center_y = self.Get_center(points)
			print(center_x,center_y)
			cv2.imshow('color', color)
			cv2.imshow('morph',morph)
			print(points)
			# draw_center = self.Draw_center(center,mask)
			# cv2.imshow('contour',draw_center)


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
        # return self.s


    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

############################	主函数	##############################################     
       
if __name__== '__main__' :
    #cv2.namedWindow("Image") #创建窗口
    #抓取摄像头视频图像
    cap = cv2.VideoCapture(0)  #创建内置摄像头变量
 
    while(cap.isOpened()):  #isOpened()  检测摄像头是否处于打开状态
        ret, img = cap.read()  #把摄像头获取的图像信息保存之img变量
        if ret == True:       #如果摄像头读取图像成功
            # time.sleep(1)
            # cv2.imshow('Image',img)
            d = Findposition(img, 4)
            d.main_process_color()	#紫色

            k = cv2.waitKey(100)

            if k == ord('a') or k == ord('A'):
                cv2.imwrite('test.jpg',img)
                break
	cap.release()  #关闭摄像头
	cv2.waitKey(0)
	#cv2.destroyAllWindow()