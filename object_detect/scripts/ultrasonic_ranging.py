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

        #PID parameter
        self.kp = 0.15
        self.ki = 0.006
        self.kd = 0.1
        self.Ek = 0.0
        self.SUM = 0.0
        self.Ek1 = 0.0
        self.Output = 0.0
        self.vel_output = 0.0


        # Set the distance to travel
        self.test_distance = 0.2 # meters 存储目标距离
        self.Forward_distance = 0.10 # 前进并与墙的距离
        self.Back_distance = 0.25 #后退并与墙的距离
        self.ultrasonic_val = 4.0 #目前超声波测量距离
        self.ultrasonic_last = 4.0 #存储上次读取值，用于限速滤波
        self.ultrasonic_last_last = 4.0
        self.tolerance = 0.02 # meters 距离精度容忍度
        self.start_test = False #启动平移功能标志

        # 启动指令
        self.tag = 0 #接受上层程序的tag
        self.Forward_tag = 1 #
        self.Forward_tag1 = 4 #
        self.Back_tag = 3 #
	
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

        move_cmd = Twist()

        while not rospy.is_shutdown():
            # Stop the robot by default
            move_cmd = Twist()

            if self.start_test:
                rospy.loginfo("*************************** ")
                # How close are we?
                distance = self.ultrasonic_val
                rospy.loginfo("distance: " +str(self.ultrasonic_val) +" m")

                error = self.test_distance - self.ultrasonic_val
                rospy.loginfo("error: " +str(error) +" m")

                # Are we close enough?
                if not self.start_test or abs(error) <  self.tolerance:
                    self.cmd_vel.publish(Twist())
                    rospy.sleep(1) #消抖
                    if not self.start_test or abs(error) <  self.tolerance:
                        self.start_test = False
                        params = False
                        if self.tag == 1:
                            finish_tag = Int32(data = 1) #
                            self.forward_finish.publish(finish_tag)
                        elif self.tag == 4:
                            finish_tag = Int32(data = 1) #
                            self.forward_finish1.publish(finish_tag)
                        elif self.tag == 3:
                            finish_tag = Int32(data = 3) #
                            self.back_finish.publish(finish_tag)
                        elif self.tag == 5:
                            finish_tag = Int32(data = 1) #
                            self.back_finish1.publish(finish_tag)
                        self.tag = 0
                    rospy.loginfo(params)
                    #break
                else:
                    # If not, move in the appropriate direction
                    self.vel_output = self.Incremental_PID_Cal(self.test_distance, self.ultrasonic_val)
                    if abs(self.vel_output)<=0.06:
                        self.vel_output = 0.06
                    if abs(self.vel_output) >= 0.2:
                        self.vel_output = 0.2
                    move_cmd.linear.x = copysign(self.vel_output, -1 * error)
                    rospy.loginfo("move_cmd.liner.x: "+str(move_cmd.linear.x))
                    self.cmd_vel.publish(move_cmd) #不能一直发0速度，会与其他节点冲撞
            r.sleep()

        # Stop the robot
        #self.cmd_vel.publish(Twist())

    '''
    位置式PID算法函数
    参数： 
    SetValue:目标值
    ActualValue:反馈值
    返回值：
    PID增量的叠加量
    '''
    def Incremental_PID_Cal(self, SetValue, ActualValue):
        self.Ek = SetValue - ActualValue
        self.SUM += self.Ek
        self.Output = self.kp*self.Ek + self.ki*self.SUM + self.kd*(self.Ek - self.Ek1)
        self.Ek1 = self.Ek
        return self.Output

    def sensor_callback(self, req):
        self.ultrasonic_val = req.data[0] / 1000 #castle底盘驱动程序的传感器数据排序，第一个是超声波测量数据
        
#        self.ultrasonic_val = (self.ultrasonic_val + self.ultrasonic_last + self.ultrasonic_last_last)/3
        if abs(self.ultrasonic_val-self.ultrasonic_last) >= 0.5 and not self.first_flag: #限速滤波
            self.ultrasonic_val = self.ultrasonic_last
        if self.ultrasonic_val >= 4:
            self.ultrasonic_val = 4
        self.first_flag = False
        #self.ultrasonic_val = req.range
        self.ultrasonic_last_last = self.ultrasonic_last
        self.ultrasonic_last = self.ultrasonic_val
        print self.ultrasonic_val

    def start_flag_callback(self, req):
        self.tag = req.data
        if (self.tag == 1) or (self.tag == 4):
            self.test_distance = self.Forward_distance
            # 开启超声波测距平移
            self.start_test = True 
        elif (self.tag == 3) or (self.tag == 5):
            self.test_distance = self.Back_distance
            # 开启超声波测距平移
            self.start_test = True 

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
