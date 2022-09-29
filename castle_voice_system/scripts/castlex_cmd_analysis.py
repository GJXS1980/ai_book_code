#!/usr/bin/python
# -*- coding: UTF-8 -*-

'''
作者:赵家俊
最后修改日期:2019.05.18
'''
 
from xml.dom.minidom import parse
import xml.dom.minidom

import rospy

from math import sin, cos, pi, sqrt
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32, String, Int32MultiArray

class XML_Analysis():
    def __init__(self):
        rospy.init_node('XML_Analysis', log_level=rospy.INFO)
        #在launch文件中获取参数
        self.file_path = rospy.get_param("~file_path", "/params/c.xml")
        self.sub = rospy.Subscriber('/voice/castle_xf_cmd_topic', Int32 , self.cmd_callback)
        self.pub = rospy.Publisher('/voice/goal_point', Int32MultiArray, queue_size = 1)
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
        self.f = None
        self.f_id = None
        self.start = None
        self.start_id = None
        self.pick = None
        self.pick_id = None
        self.block = None
        self.block_id = None
        self.t = None
        self.t_id = None
        self.terminal = None
        self.terminal_id = None

        self.cmd_flag = False

        self.r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.cmd_flag is True:
                self.analysis_file()
                self.Process_Speech_cmd_to_Speed()
                self.cmd_flag = False
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
            '''
            try:
                print "focus: ", map(str,focus_data).index('start')
            except ValueError:
                print 'None'
            '''

            self.result_confidence = self.result.getElementsByTagName('confidence')[0]
            self.result_confidence_data = map(int,self.result_confidence.childNodes[0].data.split('|'))
            print "confidence:", self.result_confidence_data

            self.object = self.nlp.getElementsByTagName("object")[0] #获取object节点

            try:
                self.f = self.object.getElementsByTagName('f')[0]
                print "f: %s" % self.f.childNodes[0].data
                if self.f.hasAttribute("id"):
                    self.f_id = int(self.f.getAttribute("id"))
                    print "f id: %s" % self.f.getAttribute("id")
            except IndexError:
                self.f = None
                self.f_id = None
                print 'No f'

            try:
                self.start = self.object.getElementsByTagName('start')[0]
                print "start: %s" % self.start.childNodes[0].data
                if self.start.hasAttribute("id"):
                    self.start_id = int(self.start.getAttribute("id"))
                    print "start id: %s" %  self.start.getAttribute("id")
            except IndexError:
                self.start = None
                self.start_id = None
                print 'No start'
            
            try:
                self.pick = self.object.getElementsByTagName('pick')[0]
                print "pick: %s" % self.pick.childNodes[0].data
                if self.pick.hasAttribute("id"):
                    self.pick_id = int(self.pick.getAttribute("id"))
                    print "pick id: %s" %  self.pick.getAttribute("id")
            except IndexError:
                print 'No pick'

            try:
                self.block = self.object.getElementsByTagName('block')[0]
                print "block: %s" % self.block.childNodes[0].data
                if self.block.hasAttribute("id"):
                    self.block_id = int(self.block.getAttribute("id"))
                    print "block id: %s" %  self.block.getAttribute("id")
            except IndexError:
                self.block = None
                self.block_id = None
                print 'No block'

            try:
                self.t = self.object.getElementsByTagName('t')[0]
                print "t: %s" % self.t.childNodes[0].data
                if self.t.hasAttribute("id"):
                    self.t_id = int(self.t.getAttribute("id"))
                    print "t id: %s" %  self.t.getAttribute("id")
            except IndexError:
                self.t = None
                self.t_id = None
                print 'No t'

            try:
                self.terminal = self.object.getElementsByTagName('terminal')[0]
                print "terminal: %s" % self.terminal.childNodes[0].data
                if self.terminal.hasAttribute("id"):
                    self.terminal_id = int(self.terminal.getAttribute("id"))
                    print "terminal id: %s" %  self.terminal.getAttribute("id")
            except IndexError:
                self.terminal = None
                self.terminal_id = None
                print 'No terminal'
            print "解析完成.....\n"
        
        else:
            print "No XML"

    def cmd_callback(self, req):
        print "命令词识别成功，正在解析命令词结果..."
        self.cmd_flag = True

    def Process_Speech_cmd_to_Speed(self):
        print "文本： ", self.rawtext_data #打印会话结果
        print "可信度： ", self.confidence_data #打印可信度
        if self.confidence_data>=50:
            #if self.result_confidence_data[self.focus_data.index('start')]>50 and self.result_confidence_data[self.focus_data.index('block')]>50 and self.result_confidence_data[self.focus_data.index('terminal')]>50:
                print "pass!"
                self.goal_point_msg = Int32MultiArray()
                self.goal_point_msg.data = [self.start_id, self.block_id,self.terminal_id]#self.terminal_id, self.block_id]
                #rospy.loginfo(self.goal_point_msg)
                self.pub.publish(self.goal_point_msg)

if __name__ == '__main__':
    my_XML_Analysis = XML_Analysis()
        
