#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int64

from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped, PoseWithCovarianceStamped, PoseArray
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


class Setpoints():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)

        #   初始化ROS节点
        rospy.init_node('nav_setpoints', anonymous = False)

        # 目标点的x,y,w坐标
        self.waypointsx = list([1.69192862511, 2.67467808723, 1.87002670765, 0.0750854089856, -0.366911172867])
        self.waypointsy = list([-1.14016997814, -1.20085799694, -0.173780530691, -0.969016313553, -1.06978178024])
        self.waypointsaw = list([0.00494333483253, 0.0, 0.999868289316, 0.999909724069, 0.998928188835])
        self.waypointsw = list([0.999987781646, 1.0, 0.0162297264389, 0.0134366555461, 0.0462868615377])

        #   订阅语音输入的命令词
        rospy.Subscriber('voice/goal_point', Int32MultiArray, self.getGoalPoint)

        self.setpose_pub = rospy.Publisher("amcl_pose", PoseWithCovarianceStamped, latch=True, queue_size=1)

        #   发布导航成功之后的话题
        self.pub = rospy.Publisher('castlex_pick_up_tag', Int64, queue_size=1)

        self.flag = 0

        #   订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base cation server ...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move_base server")

        while not rospy.is_shutdown():
            # self.move_base.wait_for_result()
            rospy.sleep(1)

    #   获取语音输入的命令词
    def getGoalPoint(self,data):
        self.first_point = data.data[0]
        self.getFirstPoint()

    #   设置导航的目标点
    def getFirstPoint(self):
        # 初始化goal为MoveBaseGoal类型
        goal = MoveBaseGoal()
        # 使用map的frame定义goal的frame id
        goal.target_pose.header.frame_id = 'map'
        # 设置时间戳
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
		#设置厨房点的位姿并导航到厨房点
        if self.first_point == 2:
            i = 0
            while i < 2 and not rospy.is_shutdown():
                goal.target_pose.pose.position.x = self.waypointsx[i]
                goal.target_pose.pose.position.y = self.waypointsy[i]
                goal.target_pose.pose.orientation.z = self.waypointsaw[i]
                goal.target_pose.pose.orientation.w = self.waypointsw[i]
                rospy.loginfo("Sending goal")
                self.move(goal)
                i += 1
                if i == 1:
                    self.flag = 1
                else:
                    pass

		#	设置客厅点的位姿并导航到客厅点
        elif self.first_point == 0:
            i = 5
            while i < 7 and not rospy.is_shutdown():
                goal.target_pose.pose.position.x = self.waypointsx[i]
                goal.target_pose.pose.position.y = self.waypointsy[i]
                goal.target_pose.pose.orientation.z = self.waypointsaw[i]
                goal.target_pose.pose.orientation.w = self.waypointsw[i]
                rospy.loginfo("Sending goal")
                self.move(goal)
                i += 1
                if i == 6:
                    self.flag = 1
                else:
                    pass

        else:
            pass

    #   执行导航动作
    def move(self, goal):
        self.move_base.send_goal(goal)

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
                if self.flag == 1:
                    self.tag = 1
                    self.pub.publish(self.tag)
                else:
                    pass

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
