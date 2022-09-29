#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
import os
import actionlib
from Tkinter import *
from tkMessageBox import *
from tkFileDialog import *
from math import radians, pi
from actionlib_msgs import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


class CruiseSystem():
    def __init__(self):
        self.filename = 'goalpoint.txt'
        root = Tk()
        root.title('HG Cruise System')
        root.geometry('1000x200')
        self.seq = 0
        self.waypoints = []

        thread = threading.Thread(target = self.sentpoint)
        HgButton(text='保存目标', command= self.savepoint).grid(row=0, column=0)
        HgButton(text='启动巡航', command= lambda: thread.start()).grid(row=0, column=1)
        HgButton(text='退出', command= self.shutdown).grid(row=1, column=1)
        self.cruise_node()
        root.mainloop()

    def cruise_node(self):
        rospy.init_node("cruise", anonymous=False)
        rospy.loginfo("Node cruise has been created!")
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback, queue_size=1)


    def callback(self,data):
        if askyesno('Verify','Add waypoint?'):
            showinfo('YES','add this point')
            point = [str(self.seq), data.header.frame_id, data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
            self.waypoints.append(point)
            self.seq += 1
            print point
        else:
            showinfo('NO', 'Ignore this point')
            pass
        

    def savepoint(self):
        f = open(self.filename, 'w')
        for i in range(len(self.waypoints)):
            for j in range(len(self.waypoints[i])):
                f.write(str(self.waypoints[i][j]) + ' ')
            f.write('\n')
        f.close()
        print(self.filename + ' ' + 'has been created!')

    def sentpoint(self):
        self.goalpoints = []
        f = open(self.filename, 'r')
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move_base server")
        rospy.loginfo("Starting navigation test")
        
        points = f.readlines()
        for i in range(len(points)):
            goalpoints = points[i].split(' ')[0:-1]
            self.goalpoints.append(Pose(Point(float(goalpoints[2]),float(goalpoints[3]),float(goalpoints[4])),Quaternion(float(goalpoints[5]),float(goalpoints[6]),float(goalpoints[7]),float(goalpoints[8]))))
        print(self.goalpoints)
        f.close()
        while 1:
            for i in range(len(self.goalpoints)):
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = self.goalpoints[i]
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
        rospy.loginfo("Stopping the robot ...")
        try:
            self.move_base.cancel_goal()
        except:
            pass
        rospy.sleep(0.5)
        os._exit(99)



class HgButton(Button):
    def __init__(self, parent=None, **config):
        Button.__init__(self,parent, **config)
        self.grid(padx=20,pady=5)
        self.config(width=20, height=2,font=25)

if __name__ == '__main__':
    filepath = os.path.split(os.path.realpath(__file__))[0] 
    os.chdir(filepath)
    
    try:
        CruiseSystem()
    except rospy.ROSInterruptException:
        pass








