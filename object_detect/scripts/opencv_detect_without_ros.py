#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

class Findposition:
    def __init__(self, img):
    #获取图片
        #self.img=cv2.imread(path)
        self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 将图像从RGB转成灰度图
        self.hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #　将图像从RGB转成HSV

#####################################################################################################
#######################      获取黑色区域　　   #############################
#####################################################################################################
    #提取黑色的区域 
    def Get_black(self):
        #get black area
        low_black = np.array([0,0,0])   #　创建黑色最低的hsv的范围数组
        high_black = np.array([180,255,46])    #　创建黑色最高的hsv的范围数组
        mask = cv2.inRange(self.hsv, low_black, high_black) # 利用cv2.inRange函数设阈值，去除背景部分
        black = cv2.bitwise_and(img, img, mask=mask)    #对图形的二进制数据进行“与”操作
        # black = cv2.bitwise_and(self.hsv, self.hsv, mask=mask)    #对图形的二进制数据进行“与”操作
        return black

    #将黑色区域进行二值化处理 
    def Get_contour_black(self):
        #change to gray
        black=self.Get_black()
        # black_gray=cv2.cvtColor(black, cv2.COLOR_HSV2BGR)
        black_gray=cv2.cvtColor(black_gray,cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh=cv2.threshold(black_gray,10,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph=cv2.morphologyEx(thresh,cv2.MORPH_OPEN,(3,3))
        # cv2.erode(img_morph,(3,3),img_morph,iterations=2)
        # cv2.dilate(img_morph,(3,3),img_morph,iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_black(self,img):
        img_cp = self.Get_contour_black()
        _, cnts, _ = cv2.findContours(img_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print (len(cnts))
        if len(cnts) == 0:
            img_boxpoints = cnts
        else:
            cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0]	#当没有检测到图像的时候报错，要修改
            box =cv2.minAreaRect(cnt_second)    #生成最小外接矩形y
            img_boxpoints = np.int0(cv2.boxPoints(box))  #返回最小外接矩形4 个顶点
        # print img_boxpoints
        return img_boxpoints
################################################################################
#######################      获取蓝色区域　　   #############################
################################################################################
    #提取蓝色的区域 
    def Get_blue(self):
        #get black area
        low_blue = np.array([100,50,50])    #　创建黑色最低的hsv的范围数组
        high_blue = np.array([140,255,255]) #　创建黑色最高的hsv的范围数组
        mask = cv2.inRange(self.hsv, low_blue, high_blue)   # 利用cv2.inRange函数设阈值，去除背景部分
        blue = cv2.bitwise_and(img, img, mask=mask) #对图形的二进制数据进行“与”操作
        # blue = cv2.bitwise_and(self.hsv, self.hsv, mask=mask)

        return blue

    #将蓝色区域进行二值化处理 
    def Get_contour_blue(self):
        #change to gray
        blue = self.Get_blue()  #获取带有颜色区域的图片
        # blue_gray = cv2.cvtColor(blue, cv2.COLOR_HSV2BGR)
        # blue_gray = cv2.cvtColor(blue_gray, cv2.COLOR_BGR2GRAY)
        blue_gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)  #将图形转成灰色图
        
        #binaryzation
        _, thresh = cv2.threshold(blue_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)    #对灰度图进行固定阈值二值化（使用Otsu’s 二值化）
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3)) #利用形态学滤波开运算(open)来进行图形处理，先腐蚀后膨胀的
        # open = cv2.erode(img_morph, (3,3), img_morph, iterations=2)    #腐蚀图像
        # img_morph = cv2.dilate(open, (3,3), open, iterations=2)   #膨胀图像
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_blue(self,img):
        img_cp = self.Get_contour_blue()    #img_cp不是灰度图而是二值图
        _, cnts, _ = cv2.findContours(img_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print (len(cnts))
        if len(cnts) == 0:
            img_boxpoints = cnts
        else:
            cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0]	#当没有检测到图像的时候报错，要修改
            box =cv2.minAreaRect(cnt_second)    #生成最小外接矩形
            img_boxpoints = np.int0(cv2.boxPoints(box))  #返回最小外接矩形4 个顶点
        # print img_boxpoints
        return img_boxpoints
################################################################################
#######################      获取青色区域　　   #############################
################################################################################
    #提取青色的区域 
    def Get_Cyan(self):
        #get black area
        low_Cyan = np.array([78, 43, 46])
        high_Cyan = np.array([99, 255, 255])
        mask = cv2.inRange(self.hsv, low_Cyan, high_Cyan)
        Cyan = cv2.bitwise_and(img, img,mask=mask)
        return Cyan

    #将青色区域进行二值化处理 
    def Get_contour_Cyan(self):
        #change to gray
        Cyan = self.Get_Cyan()
        # Cyan_gray = cv2.cvtColor(Cyan, cv2.COLOR_HSV2BGR)
        Cyan_gray = cv2.cvtColor(Cyan, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(Cyan_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_Cyan(self,img):
        img_cp = self.Get_contour_Cyan()
        _, cnts, _ = cv2.findContours(img_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print (len(cnts))
        if len(cnts) == 0:
            img_boxpoints = cnts
        else:
            cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0]	#当没有检测到图像的时候报错，要修改
            box =cv2.minAreaRect(cnt_second)    #生成最小外接矩形
            img_boxpoints = np.int0(cv2.boxPoints(box))  #返回最小外接矩形4 个顶点
        # print img_boxpoints
        return img_boxpoints

################################################################################
#######################      获取橙色区域　　   #############################
################################################################################
    #提取橙色的区域 
    def Get_orange(self):
        #get black area
        low_orange = np.array([11, 43, 46])
        high_orange = np.array([25, 255, 255])
        mask = cv2.inRange(self.hsv, low_orange, high_orange)
        orange = cv2.bitwise_and(img, img,mask=mask)
        return orange

    #将橙色区域进行二值化处理 
    def Get_contour_orange(self):
        #change to gray
        orange = self.Get_orange()
        # orange_gray = cv2.cvtColor(orange, cv2.COLOR_HSV2BGR)
        orange_gray = cv2.cvtColor(orange, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(orange_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_orange(self,img):
        img_cp = self.Get_contour_orange()
        _, cnts, _ = cv2.findContours(img_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print (len(cnts))
        if len(cnts) == 0:
            img_boxpoints = cnts
        else:
            cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0]	#当没有检测到图像的时候报错，要修改
            box =cv2.minAreaRect(cnt_second)    #生成最小外接矩形
            img_boxpoints = np.int0(cv2.boxPoints(box))  #返回最小外接矩形4 个顶点
        # print img_boxpoints
        return img_boxpoints

################################################################################
#######################      获取黄色区域　　   #############################
################################################################################
    #提取黄色的区域 
    def Get_yellow(self):
        #get black area
        low_yellow = np.array([26, 43, 46])
        high_yellow = np.array([34, 255, 255])
        mask = cv2.inRange(self.hsv, low_yellow, high_yellow)
        yellow = cv2.bitwise_and(img, img,mask=mask)
        return yellow

    #将黄色区域进行二值化处理 
    def Get_contour_yellow(self):
        #change to gray
        yellow = self.Get_yellow()
        # yellow_gray = cv2.cvtColor(yellow, cv2.COLOR_HSV2BGR)
        yellow_gray = cv2.cvtColor(yellow, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(yellow_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_yellow(self,img):
        img_cp = self.Get_contour_yellow()
        _, cnts, _ = cv2.findContours(img_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print (len(cnts))
        if len(cnts) == 0:
            img_boxpoints = cnts
        else:
            cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0]	#当没有检测到图像的时候报错，要修改
            box =cv2.minAreaRect(cnt_second)    #生成最小外接矩形
            img_boxpoints = np.int0(cv2.boxPoints(box))  #返回最小外接矩形4 个顶点
        # print img_boxpoints
        return img_boxpoints

################################################################################
#######################      获取紫色区域　　   #############################
################################################################################
    #提取紫色的区域 
    def Get_Violet(self):
        #get black area
        low_Violet = np.array([125, 43, 46])
        high_Violet = np.array([155, 255, 255])
        mask = cv2.inRange(self.hsv, low_Violet, high_Violet)
        Violet = cv2.bitwise_and(img, img,mask=mask)
        return Violet

    #将紫色区域进行二值化处理 
    def Get_contour_Violet(self):
        #change to gray
        Violet = self.Get_Violet()
        # Violet_gray = cv2.cvtColor(Violet, cv2.COLOR_HSV2BGR)
        Violet_gray = cv2.cvtColor(Violet, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(Violet_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_Violet(self,img):
        img_cp = self.Get_contour_Violet()
        _, cnts, _ = cv2.findContours(img_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print (len(cnts))
        if len(cnts) == 0:
            img_boxpoints = cnts
        else:
            cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0]	#当没有检测到图像的时候报错，要修改
            box =cv2.minAreaRect(cnt_second)    #生成最小外接矩形
            img_boxpoints = np.int0(cv2.boxPoints(box))  #返回最小外接矩形4 个顶点
        print img_boxpoints
        return img_boxpoints  
################################################################################
#######################      获取红色区域　　   #############################
################################################################################
    #提取红色的区域 
    def Get_red(self):
        #get black area
        low_red = np.array([0, 43, 46])
        high_red = np.array([10, 255, 255])
        mask = cv2.inRange(self.hsv, low_red, high_red)
        red = cv2.bitwise_and(img, img,mask=mask)
        return red

    #将红色区域进行二值化处理 
    def Get_contour_red(self):
        #change to gray
        red = self.Get_red()
        # red_gray = cv2.cvtColor(red, cv2.COLOR_HSV2BGR)
        red_gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(red_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_red(self,img):
        img_cp = self.Get_contour_red()
        _, cnts, _ = cv2.findContours(img_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print (len(cnts))
        if len(cnts) == 0:
            img_boxpoints = cnts
        else:
            cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0]	#当没有检测到图像的时候报错，要修改
            box =cv2.minAreaRect(cnt_second)    #生成最小外接矩形
            img_boxpoints = np.int0(cv2.boxPoints(box))  #返回最小外接矩形4 个顶点
        # print img_boxpoints
        return img_boxpoints

################################################################################
#######################      获取红色1区域　　   #############################
################################################################################
    #提取红色1的区域 
    def Get_red1(self):
        #get black area
        low_red1 = np.array([156, 43, 46])
        high_red1 = np.array([180, 255, 255])
        mask = cv2.inRange(self.hsv, low_red1, high_red1)
        red1 = cv2.bitwise_and(img, img,mask=mask)
        return red1

    #将红色1区域进行二值化处理 
    def Get_contour_red1(self):
        #change to gray
        red1 = self.Get_red1()
        # red1_gray = cv2.cvtColor(red1, cv2.COLOR_HSV2BGR)
        red1_gray = cv2.cvtColor(red1, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(red1_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_red1(self,img):
        img_cp = self.Get_contour_red1()
        _, cnts, _ = cv2.findContours(img_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print (len(cnts))
        if len(cnts) == 0:
            img_boxpoints = cnts
        else:
            cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0]	#当没有检测到图像的时候报错，要修改
            box =cv2.minAreaRect(cnt_second)    #生成最小外接矩形
            img_boxpoints = np.int0(cv2.boxPoints(box))  #返回最小外接矩形4 个顶点
        print img_boxpoints
        return img_boxpoints
################################################################################ 
#######################      获取绿色区域　　   #############################
################################################################################
    #提取绿色的区域 
    def Get_green(self):
        #get black area
        low_green = np.array([35, 43, 46])
        high_green = np.array([77, 255, 255])
        mask = cv2.inRange(self.hsv, low_green, high_green)
        green = cv2.bitwise_and(img, img,mask=mask)
        return green

    #将绿色区域进行二值化处理 
    def Get_contour_green(self):
        #change to gray
        green = self.Get_green()
        # green_gray = cv2.cvtColor(green, cv2.COLOR_HSV2BGR)
        green_gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(green_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_green(self,img):
        img_cp = self.Get_contour_green()
        _, cnts, _ = cv2.findContours(img_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print (len(cnts))
        if len(cnts) == 0:
            img_boxpoints = cnts
        else:
            cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0]	#当没有检测到图像的时候报错，要修改
            box =cv2.minAreaRect(cnt_second)    #生成最小外接矩形
            img_boxpoints = np.int0(cv2.boxPoints(box))  #返回最小外接矩形4 个顶点
        # print img_boxpoints
        return img_boxpoints
################################################################################ 
#######################      获取轮廓的中心点坐标　　   #############################
################################################################################ 
    #绘制轮廓
    def Draw_contour(self,points):
        global mask
        if len(points) == 0:
            pass
        else:
            mask = np.zeros(self.gray.shape,np.uint8)
            cv2.drawContours(mask,[points],-1,255,2)
        # return mask

    #获取中心位置
    def Get_center(self,points):
        global center
        if len(points) == 0:
            center = 0
        else:
            p1x,p1y=points[0,0],points[0,1]
            p3x,p3y=points[2,0],points[2,1]
            center_x,center_y=(p1x+p3x)/2,(p1y+p3y)/2
            center=(center_x,center_y)
        return center

    #绘制中心点
    def Draw_center(self,center,mask):
        # global mask1        
        if center == 0:
            pass
        else:
            cv2.circle( mask,center,1,(255,255,255),2)
        # return mask

################################################################################ 
#######################      识别黑色区域及中心点坐标主程序　　   ######################
################################################################################ 
    #主函数
    def main_process_black(self):
        morph = self.Get_contour_black()
        black = self.Get_black()

        points = self.Find_contour_black(morph)
        mask = self.Draw_contour(points)
        center = self.Get_center(points)

        cv2.imshow('img',img)

        if center == 0:
            pass
        else:
            center_x,center_y = self.Get_center(points)
            print(center_x,center_y)
            cv2.imshow('black', black)
            cv2.imshow('morph',morph)
            # print(points)
            # if len(points) == 0:
            #     pass
            # else:
            #     draw_center = self.Draw_center(center,mask)
            #     cv2.imshow('contour',draw_center)


################################################################################ 
#######################      识别蓝色区域及中心点坐标主程序　　   ######################
################################################################################  
    #主函数
    def main_process_blue(self):
        morph = self.Get_contour_blue()

        blue = self.Get_blue()

        points = self.Find_contour_blue(morph)
        mask = self.Draw_contour(points)
        center = self.Get_center(points)

        cv2.imshow('img',img)

        if center == 0:
            pass
        else:
            center_x,center_y = self.Get_center(points)
            print(center_x,center_y)
            cv2.imshow('blue', blue)
            cv2.imshow('morph',morph)
            # print(points)
            # if len(points) == 0:
            #     pass
            # else:
            #     draw_center = self.Draw_center(center,mask)
            #     cv2.imshow('contour',draw_center)


################################################################################ 
#######################      识别青色区域及中心点坐标主程序　　   ######################
################################################################################  
    #主函数
    def main_process_Cyan(self):
        morph = self.Get_contour_Cyan()

        Cyan = self.Get_Cyan()

        points = self.Find_contour_Cyan(morph)
        mask = self.Draw_contour(points)
        center = self.Get_center(points)

        cv2.imshow('img',img)

        if center == 0:
            pass
        else:
            center_x,center_y = self.Get_center(points)
            print(center_x,center_y)
            cv2.imshow('Cyan', Cyan)
            cv2.imshow('morph',morph)
            # print(points)
            # if len(points) == 0:
            #     pass
            # else:
            #     draw_center = self.Draw_center(center,mask)
            #     cv2.imshow('contour',draw_center)

################################################################################ 
#######################      识别橙色区域及中心点坐标主程序　　   ######################
################################################################################     #主函数
    def main_process_orange(self):
        morph = self.Get_contour_orange()

        orange = self.Get_orange()

        points = self.Find_contour_orange(morph)
        mask = self.Draw_contour(points)
        center = self.Get_center(points)


        cv2.imshow('img',img)

        if center == 0:
            pass
        else:
            center_x,center_y = self.Get_center(points)
            print(center_x,center_y)
            cv2.imshow('orange', orange)
            cv2.imshow('morph',morph)
            # print(points)
            # if len(points) == 0:
            #     pass
            # else:
            #     draw_center = self.Draw_center(center,mask)
            #     cv2.imshow('contour',draw_center)


################################################################################ 
#######################      识别黄色区域及中心点坐标主程序　　   ######################
################################################################################     #主函数
    def main_process_yellow(self):
        morph = self.Get_contour_yellow()

        yellow = self.Get_yellow()

        points = self.Find_contour_yellow(morph)
        mask = self.Draw_contour(points)
        center = self.Get_center(points)

        cv2.imshow('img',img)

        if center == 0:
            pass

        else:
            center_x,center_y = self.Get_center(points)
            print(center_x,center_y)
            cv2.imshow('yellow', yellow)
            cv2.imshow('morph',morph)
            # print(points)
            # if len(points) == 0:
            #     pass
            # else:
            #     draw_center = self.Draw_center(center,mask)
            #     cv2.imshow('contour',draw_center)


################################################################################ 
#######################      识别紫色区域及中心点坐标主程序　　   ######################
################################################################################     #主函数
    def main_process_Violet(self):
        morph = self.Get_contour_Violet()

        Violet = self.Get_Violet()

        points = self.Find_contour_Violet(morph)
        mask = self.Draw_contour(points)
        center = self.Get_center(points)
        # print(center)
        cv2.imshow('img',img)

        if center == 0:
            pass
        else:
            center_x,center_y = self.Get_center(points)
            print(center_x,center_y)
            cv2.imshow('Violet', Violet)
            cv2.imshow('morph',morph)
            # print(points)
            # if len(points) == 0:
            #     pass
            # else:
            #     draw_center = self.Draw_center(center,mask)
            #     cv2.imshow('contour',draw_center)


################################################################################ 
#######################      识别红色区域及中心点坐标主程序　　   ######################
################################################################################     #主函数
    def main_process_red(self):
        morph = self.Get_contour_red()
        red = self.Get_red()

        points = self.Find_contour_red(morph)
        mask = self.Draw_contour(points)
        center = self.Get_center(points)

        cv2.imshow('img',img)

        if center == 0:
            pass
        else:
            center_x,center_y = self.Get_center(points)
            print(center_x,center_y)
            cv2.imshow('red', red)
            cv2.imshow('morph',morph)
            # print(points)
            # if len(points) == 0:
            #     pass
            # else:
            #     draw_center = self.Draw_center(center,mask)
            #     cv2.imshow('contour',draw_center)


################################################################################ 
#######################      识别红色１区域及中心点坐标主程序　　   ######################
################################################################################     #主函数
    def main_process_red1(self):
        morph = self.Get_contour_red1()
        red1 = self.Get_red1()

        points = self.Find_contour_red1(morph)
        mask = self.Draw_contour(points)
        center = self.Get_center(points)

        cv2.imshow('img',img)

        if center == 0:
            pass
        else:
            center_x,center_y = self.Get_center(points)
            print(center_x,center_y)
            cv2.imshow('red1', red1)
            cv2.imshow('morph',morph)
            # print(points)
            # if len(points) == 0:
            #     pass
            # else:
            #     draw_center = self.Draw_center(center,mask)
            #     cv2.imshow('contour',draw_center)

################################################################################ 
#######################      识别绿色区域及中心点坐标主程序　　   ######################
################################################################################     #主函数
    def main_process_green(self):
        morph = self.Get_contour_green()
        green = self.Get_green()

        points = self.Find_contour_green(morph)
        mask = self.Draw_contour(points)
        center = self.Get_center(points)


        cv2.imshow('img',img)

        if center == 0:
            pass
        else:
            center_x,center_y = self.Get_center(points)
            print(center_x,center_y)
            cv2.imshow('green', green)
            cv2.imshow('morph',morph)
            # print(points)
            # if len(points) == 0:
            #     pass
            # else:
            #     draw_center = self.Draw_center(center,mask)
            #     cv2.imshow('contour',draw_center)

################################################################################     
       
if __name__== '__main__' :
    #cv2.namedWindow("Image") #创建窗口
    #抓取摄像头视频图像
    cap = cv2.VideoCapture("/dev/video0")  #创建内置摄像头变量
 
    while(cap.isOpened()):  #isOpened()  检测摄像头是否处于打开状态
        ret, img = cap.read()  #把摄像头获取的图像信息保存之img变量
        if ret == True:       #如果摄像头读取图像成功
            # cv2.imshow('Image',img)
            d = Findposition(img)
            # d.main_process_black()	#黑色
            #d.main_process_blue()	#蓝色
            # d.main_process_Cyan()	#青色
            # d.main_process_orange()
            # d.main_process_yellow()	#	黄色（用不上）
            # d.main_process_Violet()	#紫色
            #d.main_process_red()
            d.main_process_red1()
            # d.main_process_green()

            k = cv2.waitKey(100)

            if k == ord('a') or k == ord('A'):
                cv2.imwrite('test.jpg',img)
                break
    cap.release()  #关闭摄像头
    cv2.waitKey(0)
   # cv2.destroyAllWindow()
