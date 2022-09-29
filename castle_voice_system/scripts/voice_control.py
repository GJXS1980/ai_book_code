#!/usr/bin/python
# -*- coding: UTF-8 -*-

from xml.dom.minidom import parse
import xml.dom.minidom

import rospy
from aip import AipSpeech
from playsound import playsound
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64, Int32

class XML_Analysis():
    def __init__(self):
        rospy.init_node('XML_Analysis', log_level=rospy.INFO)
        #在launch文件中获取参数
        self.file_path = rospy.get_param("~file_path", "/params/voice_control.xml")
        self.sub = rospy.Subscriber('/voice/castle_xf_cmd_topic', Int32 , self.cmd_callback)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)        
        self.DOMTree = None
        self.nlp = None
        self.version = None
        self.rawtext = None
        self.rawtext_data = None
        self.confidence = None
        self.confidence_data = None
        self.engine = None
        self.focus = None
        self.result_confidence = None
        self.result_confidence_data = None
        self.voice_cmd = None
        self.voice_cmd_id = None

        self.voice1 = rospy.get_param("~failed_file_path", "/params/voice/failed.mp3")
        self.voice2 = rospy.get_param("~Received_file_path", "/params/voice/Received.mp3")
        self.voice3 = rospy.get_param("~ReEnterAuido_file_path", "/params/voice/ReEnterAuido.mp3")

        self.cmd_flag = False
        self.control_flag = False
        self.r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.cmd_flag is True:
                self.analysis_file()
                self.Process_Speech_cmd_to_Speed()
                self.cmd_flag = False
            if self.control_flag:
                self.cmd_vel.publish(self.vel)
            self.r.sleep()

    # 使用minidom解析器打开 XML 文档
    def open_xml_file(self):
        self.DOMTree = xml.dom.minidom.parse(self.file_path)
        self.nlp = self.DOMTree.documentElement #获取nlp节点

    def analysis_file(self):
        self.open_xml_file()
        if self.nlp.hasChildNodes():
            self.version = self.nlp.getElementsByTagName('version')[0]
            #print "version: %s" % version.childNodes[0].data
            self.rawtext = self.nlp.getElementsByTagName('rawtext')[0]
            self.rawtext_data = self.rawtext.childNodes[0].data
            #print "rawtext: %s" % self.rawtext.childNodes[0].data
            self.confidence = self.nlp.getElementsByTagName('confidence')[0]
            self.confidence_data = self.confidence.childNodes[0].data
            #print "confidence: %s" % self.confidence.childNodes[0].data
            self.engine = self.nlp.getElementsByTagName('engine')[0]
            #print "engine: %s" % self.engine.childNodes[0].data

            self.result = self.nlp.getElementsByTagName("result")[0] #获取result节点

            self.focus = self.result.getElementsByTagName('focus')[0]
            self.focus_data = map(str,self.focus.childNodes[0].data.split('|'))

            self.result_confidence = self.result.getElementsByTagName('confidence')[0]
            self.result_confidence_data = map(int,self.result_confidence.childNodes[0].data.split('|'))
            print "confidence:", self.result_confidence_data

            self.object = self.nlp.getElementsByTagName("object")[0] #获取object节点
            try:
                self.voice_cmd = self.object.getElementsByTagName('action')[0]
                print "command: %s" % self.voice_cmd.childNodes[0].data
                if self.voice_cmd.hasAttribute("id"):
                    self.voice_cmd_id = int(self.vocie_cmd.getAttribute("id"))
                    print "vocie_cmd id: %s" %  self.vocie_cmd.getAttribute("id")
            except IndexError:
                self.vocie_cmd = None
                self.vocie_cmd_id = 306
                print 'No command'

            print "解析完成.....\n"

        else:
            print "No XML"

    def cmd_callback(self, req):
        # print "命令词识别成功，正在解析命令词结果..."
        self.cmd_flag = True

    def Process_Speech_cmd_to_Speed(self):
        print "文本： ", self.rawtext_data #打印会话结果
        self.vel = Twist()
        if (self.vocie_cmd_id==306):
            print '您输入的信息不完整，请重新输入！'
            playsound(self.voice3)
        else:
            if self.result_confidence_data[self.focus_data.index('vocie_cmd')]>40:
                self.control_flag = True
                if self.vocie_cmd_id = 0:
                    self.vel.linear.x = 0.2
                    
                elif self.vocie_cmd_id = 1:
                    self.vel.linear.x = -0.2
                    
                elif self.vocie_cmd_id = 2:
                    self.vel.angular.z = 0.5
                   
                else:
                    self.vel.angular.z = -0.5
                    
                rospy.sleep(1)
            self.sontrol_flag = False
                
                print '小谷：好的，收到!'
                playsound(self.voice2)                

            else:
                print '语音识别未通过，请重新输入！'
                playsound(self.voice1)    

if __name__ == '__main__':
    XML_Analysis()
        
