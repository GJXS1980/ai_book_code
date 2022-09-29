#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import os
from actionlib_msgs.msg import *
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped, PoseWithCovarianceStamped, PoseArray
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

class Setpoints():
    def __init__(self):

        self.waypoint_file = os.environ['WAY_POINTS']  # 根据环境变量中的'WAY_POINTS'变量寻找文件
        self.waypoints_classify = {0:'A'，1:'B',2:'C',3:'D'}   #路点根据序号字母进行分类

        rospy.on_shutdown(self.shutdown)

        #   初始化ROS节点
        rospy.init_node('nav_setpoints', anonymous = False)

        #   订阅语音输入的命令词
        rospy.Subscriber('nav_position', Int64, self.getGoalPoint)

        self.setpose_pub = rospy.Publisher("amcl_pose", PoseWithCovarianceStamped, latch=True, queue_size=1)

        #   发布导航成功之后的话题
        self.pub = rospy.Publisher('castlex_pick_up_tag', Int64, queue_size=1)

        #   订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base cation server ...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move_base server")
        # 打开路点文件
        self.openfile()
        # 将路点以目的地进行分类
        self.route = self.load_data()

        while not rospy.is_shutdown():
            # self.move_base.wait_for_result()
            rospy.sleep(1)

    #   获取语音输入的命令词
    def getGoalPoint(self,data):
        self.first_point = data.data
        # 获取话题数据，转换为路点的字母序号
        i = self.match[self.first_point]
        self.sent_goals(i)

    def openfile(self):
        try:
            self.f = open(self.waypoint_file, 'r')
        except NameError:
            print('默认文件为：{},默认文件打开失败,请再次确认文件名称和路径'.format(waypoint_file))
            print('请输入路点记录文件（完整路径）：')
            self.waypoint_file = input()      #获取输入的文件
            self.openfile()       #再次尝试打开文件

    def load_data(self):
        # 路点数据
        waypoints = []
        # 根据字母不同，将路点分成几条路线
        route = dict()
        # 读取路点文件的数据到waypoints列表
        waypoints = f.readlines()
        # 将路点数据以空格分隔为列表，并添加到self.waypoints中
        for i in range(len(waypoints)):
            waypoint_data = waypoints[i].split(' ')[:-1]
            waypoints.append(waypoint_data)
        # 将路点以字母分类到字典中，方便发布路点时调用
        for point in waypoints:
        # 检查路点中第一个元素的首字母
            letter = point[0][0]
        #如果字母在字典route中则在route[letter]的值的列表中添加一个元素
            if letter in route:
                route[letter].append(point)
        # 如果字母不在route中，则添加键值对 route[letter]：[]，再往列表中添加元素
            else:
                route[letter] = []
                route[letter].append(point)
        return route

    def sent_goals(self,i):
        goalpoints = []
        # 将route[i]中的列表数据转化为goalpoint的格式
        for point in self.route[i]:
            goalpoints.append(Pose(Point(float(point[1]), float(point[2]), float(point[3])),
                                   Quaternion(float(point[4]), float(point[5]), float(point[6]), float(point[7]))))
        # 格式化goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        for waypoint in goalpoints:
            goal.target_pose.pose = waypoint
        # 发送目标点
            self.move_base.send_goal(goal)
        # 等待导航结果
            self.move_base.wait_for_result()
        # 获取导航结果，得到结果后继续完成下一个路点
            state = self.move_base.get_state()
            if state == 0:
                rospy.loginfo('The goal has yet to be processed')
            if state == 1:
                rospy.loginfo('The goal is currentle being processed')
            if state == 2:
                rospy.loginfo('Received a cancel request')
            if state == 3:
                rospy.loginfo('The goal was achieved successfully')
            if state == 4:
                rospy.loginfo('The goal was aborted during execution')
            if state == 5:
                rospy.loginfo('The goal is invalid')
            if state == 6:
                rospy.loginfo('The goal received a cancel request and has not yet compelet')
            if state == 7:
                rospy.loginfo(
                    'The goal received a cancel request before it started but the action server has not yet confirmed')
            if state == 8:
                rospy.loginfo(
                    'The goal received a cancel request before it started executiong and was successfully cancelled')
            if state == 9:
                rospy.loginfo('The goal is lost')

            # 设定5分钟的时间限制
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

            # 如果5分钟之内没有到达，放弃目标
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("You have reached the goal!")
                    self.tag = 1
                    self.pub.publish(self.tag)

    def shutdown(self):
        rospy.loginfo("stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)

if __name__ == '__main__':

    try:
        Setpoints()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    rospy.sleep(5)
