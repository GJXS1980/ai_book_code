#! /usr/bin/env python
# -*- coding: utf-8 -*-

import MySQLdb 
import rospy
import actionlib
from actionlib_msgs.msg import *
from math import radians, pi
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

class Setpoints():
    def __init__(self):
        rospy.init_node('nav_setpoints', anonymous = False)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('/voice/goal_point',Int32MultiArray,self.getGoalPoint)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base cation server ...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move_base server")
        rospy.loginfo("Starting navigation test")
        while not rospy.is_shutdown():
            # self.move_base.wait_for_result()
            rospy.sleep(1)
        

    def getGoalPoint(self,data):
        
        self.first_point = data.data[0]
        self.second_point = data.data[1]
        self.color = data.data[2]
        self.getFirstPoint()
        

    def shutdown(self):
        rospy.loginfo("stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)

    def getFirstPoint(self):
        self.waypoints = list()
        self.header = list()
        
        if self.first_point == 0:
            classify = 'A'
            print "a"
            point = (2.51241350174, -1.13419544697, 0.0, 0.0, 0.0, -0.805028711954, 0.59323584933)
        elif self.first_point == 1:
            classify = 'B'
            print "b"
            point = (-0.0552434921265, 1.04927754402, 0.0, 0.0, 0.0, 0.985979658423, 0.166865554192)
        elif self.first_point == 2:
            classify = 'C'
            print "c"
            point = (-0.476341724396, 0.063718251884, 0.0, 0.0, 0.0, 0.989910246374, 0.141695815479)
        else :
            classify = 'D+'
            point = (2.51241350174, -1.13419544697, 0.0, 0.0, 0.0, -0.805028711954, 0.59323584933)
       
            # point = (position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)
            self.header.append(classify)
            self.waypoints.append(Pose(Point(position_x, position_y, position_z),Quaternion(orientation_x, orientation_y, orientation_z, orientation_w)))
            # print self.waypoints
        
       
        for i in range(len(self.waypoints)):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.waypoints[i]
            self.move_base.send_goal(goal)
            self.move_base.wait_for_result()
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
        self.getSecondPoint()

    def getSecondPoint(self):
        self.waypoints = list()
        self.header = list()
        if self.second_point == 0:
            classify = 'A'
            print "a"
            point = (2.51241350174, -1.13419544697, 0.0, 0.0, 0.0, -0.805028711954, 0.59323584933)
        elif self.second_point == 1:
            classify = 'B'
            print "b"
            point = (-0.0552434921265, 1.04927754402, 0.0, 0.0, 0.0, 0.985979658423, 0.166865554192)
        elif self.second_point == 2:
            classify = 'C'
            print "c"
            point = (-0.476341724396, 0.063718251884, 0.0, 0.0, 0.0, 0.989910246374, 0.141695815479)
        else :
            classify = 'D'
            point = (2.51241350174, -1.13419544697, 0.0, 0.0, 0.0, -0.805028711954, 0.59323584933)
       
            # point = (position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)
            self.header.append(classify)
            self.waypoints.append(Pose(Point(position_x, position_y, position_z),Quaternion(orientation_x, orientation_y, orientation_z, orientation_w)))
            # print self.waypoints
        
       
        for i in range(len(self.waypoints)):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.waypoints[i]
            self.move_base.send_goal(goal)
            self.move_base.wait_for_result()
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
        
    

if __name__ == '__main__':

    try:
        Setpoints()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    
    
    rospy.sleep(5)
