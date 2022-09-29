#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, Point
from math import copysign, sqrt, pow
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, Int32, Int64
import tf



class CalibrateLinear():
    def __init__(self):
        # Give the node a name
        rospy.init_node('ultrasonic_ranging', anonymous=False)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        # How fast will we check the odometry values?
        self.rate = 10
        r = rospy.Rate(self.rate)


	# 订阅导航成功话题
        rospy.Subscriber('castlex_pick_up_tag', Int64, self.start_flag_callback)

	# 订阅抓取物体成功的话题
        rospy.Subscriber('grasp_success', Int64, self.start_flag_callback)
	# 订阅第二个点导航成功的话题
        rospy.Subscriber('castlex_drop_off_tag', Int64, self.start_flag_callback)

	# 订阅放置物体成功的话题
        rospy.Subscriber('ultrasonic_back_finish1', Int32, self.start_flag_callback)
        # Subscribe to Ultrasound Sensor Information
        self.first_flag = True
        self.ultrasound = rospy.Subscriber('/sensor_data', Float32MultiArray , self.sensor_callback)
        #self.ultrasound = rospy.Subscriber('/robot0/sonar_0', Range , self.stdr_sensor_callback)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

	# 分布前进成功的话题
        self.forward_finish = rospy.Publisher('ultrasonic_forward_finish', Int32, queue_size = 1)
        self.forward_finish1 = rospy.Publisher('ultrasonic_forward_finish1', Int32, queue_size = 1)
        
        
	# 分布后退成功的话题
        self.back_finish = rospy.Publisher('ultrasonic_back_finish', Int32, queue_size = 1)
        self.back_finish1 = rospy.Publisher('ultrasonic_back_finish_fin', Int32, queue_size = 1)


        # Give tf some time to fill its buffer
        rospy.sleep(1)

        rospy.loginfo("Bring up rqt_reconfigure to control the test.")


    def sensor_callback(self, req):
        self.ultrasonic_val = req.data[0] / 1000 #castle底盘驱动程序的传感器数据排序，第一个是超声波测量



    def start_flag_callback(self, req):
        Distance = rospy.get_param('Distance')
        self.near, self.far_from = Distance['N'], Distance['F']

        Speed = rospy.get_param('Speed')
        self.forward, self.back_off = Speed['F'], Speed['B']

        self.tag = req.data
        move_cmd = Twist()
        if self.tag == 1:
                while self.ultrasonic_val > self.near:
                        if self.ultrasonic_val > self.near:
                                move_cmd.linear.x = self.forward
                                self.cmd_vel.publish(move_cmd) 
                                print(self.ultrasonic_val)
                                rospy.Subscriber('/sensor_data', Float32MultiArray , self.sensor_callback)

                if self.ultrasonic_val < self.near or self.ultrasonic_val == self.near:
                        move_cmd.linear.x = 0.0
                        self.cmd_vel.publish(move_cmd) 
                        tag = 1
                        self.forward_finish.publish(tag) 


        elif self.tag == 4:
                while self.ultrasonic_val > self.near:
                        if self.ultrasonic_val > self.near:
                                move_cmd.linear.x = self.forward
                                self.cmd_vel.publish(move_cmd) 
                                print(self.ultrasonic_val)
                                rospy.Subscriber('/sensor_data', Float32MultiArray , self.sensor_callback)
                if self.ultrasonic_val < self.near or self.ultrasonic_val == self.near:
                        move_cmd.linear.x = 0.0
                        self.cmd_vel.publish(move_cmd) 
                        tag = 1
                        self.forward_finish1.publish(tag)

        elif self.tag == 3:
                while self.ultrasonic_val < self.far_from:
                        if self.ultrasonic_val < self.far_from:
                                move_cmd.linear.x = -(self.back_off)
                                self.cmd_vel.publish(move_cmd) 
                                rospy.Subscriber('/sensor_data', Float32MultiArray , self.sensor_callback)                     
                if self.ultrasonic_val > self.far_from or self.ultrasonic_val == self.far_from:
                        move_cmd.linear.x = 0.0
                        self.cmd_vel.publish(move_cmd) 
                        tag = 3
                        self.back_finish.publish(tag) 


        elif self.tag == 5:
                while self.ultrasonic_val < self.far_from: 
                        if self.ultrasonic_val < self.far_from:
                                move_cmd.linear.x = -(self.back_off)
                                self.cmd_vel.publish(move_cmd) 
                                rospy.Subscriber('/sensor_data', Float32MultiArray , self.sensor_callback)  
                if self.ultrasonic_val > self.far_from or self.ultrasonic_val == self.far_from:
                        move_cmd.linear.x = 0.0
                        self.cmd_vel.publish(move_cmd) 
                        tag = 1
                        self.back_finish1.publish(tag)

    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the rob1ot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)




if __name__ == '__main__':
    CalibrateLinear()
    rospy.spin()
#    except:
#        rospy.loginfo("Calibration terminated.")
