#!/usr/bin/env python
#coding=utf-8

# 作者:龙爵健
# 修改日期:2019/02/02
# 
# 
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import os
# header: 
#   seq: 2522
#   stamp: 
#     secs: 1548922541
#     nsecs: 649487368
#   frame_id: ''
# axes: [-0.0, -0.0, 0.0, -0.0, -0.0, 0.0, 0.0, 0.0]
# buttons: [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
# header: 
#   seq: 2523
#   stamp: 
#     secs: 1548922541
#     nsecs: 821595145
#   frame_id: ''
# axes: [-0.0, -0.0, 0.0, -0.0, -0.0, 0.0, 0.0, 0.0]
# buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# TODO 同时按住停止按钮和前进按钮还会继续走
class TeleopJoy:
  def __init__(self):
    self.linear_gears = float(rospy.get_param("~linear_gears",0.1))
    self.angular_gears = float(rospy.get_param("~angular_gears",0.1))

    self.lastlinearX = 0.0
    self.lastlinearY = 0.0
    self.lastangularZ = 0.0
    self.joy_speed_x = 0.0
    self.joy_rev_z = 0.0
    self.i = 2

    rospy.init_node('teleop_joy', anonymous=True)
    self.pub = rospy.Publisher(rospy.get_param("~output_joy_topic","/cmd_vel"), Twist, queue_size=10)
    rospy.Subscriber(rospy.get_param("~input_joy_topic","joy"), Joy, self.callback, queue_size=1)

    # rate = rospy.Rate(10) # 10hz

  def loop(self):
    self.i -= 1
    data_twist = Twist()
    target_speed_x = self.joy_speed_x * self.linear_gears
    target_rev_z = self.joy_rev_z * self.angular_gears

    self.lastlinearX = target_speed_x
    self.lastangularZ = target_rev_z

    target_speed_x = round(target_speed_x * 100) / 100.0
    target_rev_z = round(target_rev_z * 100) / 100.0

    self.lastlinearX = round(self.lastlinearX * 100) / 100.0
    self.lastangularZ = round(self.lastangularZ * 100) / 100.0


    if target_speed_x > self.lastlinearX :
      self.lastlinearX += float(rospy.get_param("~accelerate_linear","0.05"))
    elif target_speed_x < self.lastlinearX:
      self.lastlinearX -= float(rospy.get_param("~accelerate_linear","0.05"))
    data_twist.linear.x = self.lastlinearX
      
    if target_rev_z > self.lastangularZ :
      self.lastangularZ += float(rospy.get_param("~accelerate_augular","0.05"))
    elif target_rev_z < self.lastangularZ:
      self.lastangularZ -= float(rospy.get_param("~accelerate_augular","0.05"))
    data_twist.angular.z = self.lastangularZ
    data_twist.angular.z = target_rev_z

    if self.i > 0 or self.linear_gears or self.angular_gears: 
        self.pub.publish(data_twist)
    else: pass

  def callback(self, data):
    self.i = 2
    data_twist = Twist()
    if data.buttons[0]:
      self.linear_gears += float(rospy.get_param("~linear_gears_step","0.05"))
      if self.linear_gears > rospy.get_param("~linear_gears_max","1.0"):
        self.linear_gears = float(rospy.get_param("~linear_gears_max","1.0"))
      
    if data.buttons[2]:
      self.angular_gears += float(rospy.get_param("~angular_gears_step","0.05"))
      if self.angular_gears > rospy.get_param("~angular_gears_max","1.0"):
        self.angular_gears = float(rospy.get_param("~angular_gears_max","1.0"))

    if data.buttons[3]:
      self.linear_gears -= float(rospy.get_param("~linear_gears_step","0.05"))
      if self.linear_gears < rospy.get_param("~linear_gears_min","1.0"):
        self.linear_gears = float(rospy.get_param("~linear_gears_min","0.1"))

    if data.buttons[1]:
      self.angular_gears -= float(rospy.get_param("~angular_gears_step","0.05"))
      if self.angular_gears < rospy.get_param("~angular_gears_min","1.0"):
        self.angular_gears = float(rospy.get_param("~angular_gears_min","1.0"))

    if data.buttons[4]:
      data_twist.linear.x = 0
      data_twist.linear.y = 0
      data_twist.linear.z = 0
      data_twist.angular.x = 0
      data_twist.angular.y = 0
      data_twist.angular.z = 0
      self.lastlinearX = 0
      self.lastlinearY = 0
      self.lastangularZ = 0
    else:
      self.joy_speed_x = data.axes[1]
      self.joy_rev_z = data.axes[3]

if __name__ == '__main__':
  try:
    teleop_joy = TeleopJoy()
    while not rospy.is_shutdown():
      teleop_joy.loop()
      rospy.sleep(0.1)

  except rospy.ROSInterruptException:
    rospy.logerr("teleop_joy node error!")
    os._exit(0)
  
