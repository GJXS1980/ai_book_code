#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
from math import radians, pi
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int64, Int32

from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


class Setpoints():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)

        #	初始化ROS节点
        rospy.init_node('nav_setpoints1', anonymous = False)

        #   订阅语音输入的命令词
        rospy.Subscriber('voice/goal_point', Int32MultiArray, self.getGoalPoint1)

        #   订阅机器人初始化状态的命令词        
        rospy.Subscriber('ultrasonic_back_finish', Int32, self.castlex_tag)

        #   订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #	发布导航成功之后的话题
        self.pub1 = rospy.Publisher('castlex_drop_off_tag', Int64, queue_size=10)

        #	等待服务的接入
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Waiting for move_base cation server ...")


        #	初始化second_point
        self.second_point = None

        while not rospy.is_shutdown():
            # self.move_base.wait_for_result()
            rospy.sleep(1)
        


    #	获取语音输入的命令词
    def getGoalPoint1(self,data):
		self.second_point = data.data[2]
		return self.second_point


	#	获取机械臂初始化成功的信息
    def castlex_tag(self,data):
        self.tag = data.data           
        if self.tag == 3:
            self.getSecondPoint()
        else:
            pass


    #	设置导航的目标点
    def getSecondPoint(self):
    	# 初始化goal为MoveBaseGoal类型
		goal = MoveBaseGoal()

    	# 使用map的frame定义goal的frame id
		goal.target_pose.header.frame_id = 'map'

		# 设置时间戳
		goal.target_pose.header.stamp = rospy.Time.now()

		#设置A点的位姿并导航到A点
		if self.second_point == 0:
            #goal.target_pose.pose.position.x = -0.3353787660598755
            #goal.target_pose.pose.position.y = 0.06666236370801926
            #goal.target_pose.pose.position.z = 0.0
            #goal.target_pose.pose.orientation.x = 0.0
            #goal.target_pose.pose.orientation.y = 0.0
            #goal.target_pose.pose.orientation.z = 0.9908882648706486
            #goal.target_pose.pose.orientation.w = 0.13468647497664876

			goal.target_pose.pose.position.x = -0.0166056174785
			goal.target_pose.pose.position.y = -0.0715344920754
			goal.target_pose.pose.position.z = 0.0
			goal.target_pose.pose.orientation.x = 0.0
			goal.target_pose.pose.orientation.y = 0.0
			goal.target_pose.pose.orientation.z = 0.983609179373
			goal.target_pose.pose.orientation.w = 0.180313566471
			
			rospy.loginfo("Sending goal")
			self.move(goal)


		#	设置B点的位姿并导航到B点
		elif self.second_point == 1:
			goal.target_pose.pose.position.x = 0.33280229568481445
			goal.target_pose.pose.position.y = 0.8912155032157898
			goal.target_pose.pose.position.z = 0.0
			goal.target_pose.pose.orientation.x = 0.0
			goal.target_pose.pose.orientation.y = 0.0
			goal.target_pose.pose.orientation.z = 0.5958412041184254
			goal.target_pose.pose.orientation.w = 0.8031022721140222

			rospy.loginfo("Sending goal")
			self.move(goal)

		#	设置D点的位姿并导航到D点            
		elif self.second_point == 3:
			goal.target_pose.pose.position.x = 2.981112003326416
			goal.target_pose.pose.position.y =  -0.14194917678833008
			goal.target_pose.pose.position.z = 0.0
			goal.target_pose.pose.orientation.x = 0.0
			goal.target_pose.pose.orientation.y = 0.0
			goal.target_pose.pose.orientation.z = -0.16554004272043585
			goal.target_pose.pose.orientation.w = 0.9862030694822017

			rospy.loginfo("Sending goal")
			self.move(goal)

          
		#	设置C点的位姿并导航到C点          
		elif self.second_point == 2:
            #goal.target_pose.pose.position.x = 2.3649470806121826
            #goal.target_pose.pose.position.y =  -0.9243940114974976
            #goal.target_pose.pose.position.z = 0.0
            #goal.target_pose.pose.orientation.x = 0.0
            #goal.target_pose.pose.orientation.y = 0.0
            #goal.target_pose.pose.orientation.z = -0.8144058415580713
            #goal.target_pose.pose.orientation.w = 0.5802957222279773

			goal.target_pose.pose.position.x = 1.80639922619
			goal.target_pose.pose.position.y =  -0.56131964922
			goal.target_pose.pose.position.z = 0.0
			goal.target_pose.pose.orientation.x = 0.0
			goal.target_pose.pose.orientation.y = 0.0
			goal.target_pose.pose.orientation.z = -0.836956215055
			goal.target_pose.pose.orientation.w = 0.547269854899

			rospy.loginfo("Sending goal")
			self.move(goal)

		else:
			pass


	#	执行导航动作
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
				self.tag1 = 4
				self.pub1.publish(self.tag1)


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
