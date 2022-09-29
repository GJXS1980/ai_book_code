#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import MySQLdb
from actionlib_msgs import *
from geometry_msgs.msg import PoseStamped

# 获取/move_base_simple/goal的数据,并自动地设置为目标点
class GetGoalPoint():
    def __init__(self):
        rospy.init_node('get_goalpoint',anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback, queue_size=1)
        self.waypoints = list()
        
        while not rospy.is_shutdown():
            #获取数据\
            rospy.spin()
    def callback(self,data):
        
        

        point = [str(raw_input("set pose:")), data.header.frame_id, data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        self.waypoints.append(point)
        print point


    def shutdown(self,):
        
        ## 将读取的点的数据保存在数据库中
        for i in range(len(self.waypoints)):
            try:
                cursor.execute(
                    "INSERT INTO goalpoints(classify, frame_id, position_x,position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w) values('%s','%s','%f','%f','%f','%f','%f','%f','%f')" % (self.waypoints[i][0],self.waypoints[i][1],self.waypoints[i][2],self.waypoints[i][3],self.waypoints[i][4],self.waypoints[i][5],self.waypoints[i][6],self.waypoints[i][7],self.waypoints[i][8]))

            except Exception as e:
                print "Insert Failed"
        db.commit()
        cursor.close()
        db.close()    
        rospy.loginfo("record waypoints in mysql")
        rospy.sleep(2)
        rospy.loginfo("get_waypoint_node is shutdown")
        

if __name__ == '__main__':
    db = MySQLdb.connect("localhost","root","12345678","mysql", charset = 'utf8')
    cursor = db.cursor()
    cursor.execute("DROP TABLE IF EXISTS goalpoints")
    sql = """CREATE TABLE goalpoints (
         classify  CHAR(20),
         frame_id  CHAR(20),
         position_x FLOAT,
         position_y FLOAT,
         position_z FLOAT,  
         orientation_x FLOAT,
         orientation_y FLOAT,
         orientation_z FLOAT,
         orientation_w FLOAT)"""
    cursor.execute(sql)
    try:
        GetGoalPoint()
    except rospy.ROSInterruptException:
        rospy.loginfo("")

        
