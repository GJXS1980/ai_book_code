#! /usr/bin/env python
# -*- coding: utf-8 -*-

import MySQLdb 
import rospy
import actionlib
from actionlib_msgs.msg import *
from math import radians, pi
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

class Setpoints():
    def __init__(self):
        rospy.init_node('nav_setpoints', anonymous = False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 5)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base cation server ...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move_base server")
        rospy.loginfo("Starting navigation test")

        self.readData()
        
        
        for i in range(len(self.waypoints)):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.waypoints[i]
            self.move_base.send_goal(goal)
            ##等待导航结果
            self.move_base.wait_for_result()
            ##读取move_base状态
            state = self.move_base.get_state()
            if state == 0:
                rospy.loginfo("The goal has yet to be processed")
            if state == 1:
                rospy.loginfo("The goal is currentle being processed")
            if state == 2:
                rospy.loginfo("Received a cancel request")
            if state == 3:
                rospy.loginfo("The goal was achieved successfully")
            if state == 4:
                rospy.loginfo("The goal was aborted during execution")
            if state == 5:
                rospy.loginfo("The goal is invalid")
            if state == 6:
                rospy.loginfo("The goal received a cancel request and has not yet completed")
            if state == 7:
                rospy.loginfo("The goal received a cancel request before it started executing but the action server has not yet confirmed that the goal is canceled")
            if state == 8:
                rospy.loginfo("The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)")
            if state == 9:
                rospy.loginfo("The goal is lost!")
          
    def shutdown(self):
        rospy.loginfo("stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)

    def readData(self):
        db = MySQLdb.connect("localhost", "root", "12345678", "mysql", charset = 'utf8')
        self.cursor = db.cursor()
        self.cursor.execute('select * from goalpoints')
        self.waypoints = list()
        self.header = list()
        data = self.cursor.fetchall()
        for row in data:
            seq = row[0]
            frame_id = row[1]
            position_x = row[2] 
            position_y = row[3]
            position_z = row[4]
            orientation_x = row[5]
            orientation_y = row[6]
            orientation_z = row[7]
            orientation_w = row[8]

            point = (position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)
            self.header.append(seq)
            # 特别注意话题格式要如何设置
            self.waypoints.append(Pose(Point(position_x, position_y, position_z),Quaternion(orientation_x, orientation_y, orientation_z, orientation_w)))
            print self.waypoints
        self.cursor.close()
        db.close()

if __name__ == '__main__':

    try:
        Setpoints()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    
    rospy.loginfo("read waypoint in mysql")
    rospy.sleep(5)