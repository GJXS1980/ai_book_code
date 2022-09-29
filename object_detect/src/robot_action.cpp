#include "ros/ros.h"
#include "object_detect/Center_msg.h"
#include <cstdlib>

#include <iostream>
 
using namespace std;


class Listener
{
public:
  int count = 0;
  float x1 = 0, y1 = 0;
public:
  void callback(const object_detect::Center_msg::ConstPtr& msg);
  int print_datax()
  {

  return x1;
  }
  int print_datay()
  {

  return y1;
  }
};


void Listener::callback(const object_detect::Center_msg::ConstPtr& msg)
{
  x1 = msg->data[0];
  y1 = msg->data[1];
  print_datax();
  print_datay();
  ++count;
}



int main(int argc, char **argv)
{
  float sx = 0, sy = 0;

  // 初始化ROS节点
  ros::init(argc, argv, "listener_class");

  // 创建节点句柄
  ros::NodeHandle n;

  // 创建一个Subscriber，订阅名为chatter的topic，注册回调函数chatterCallback
  Listener listener;
  ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::callback, &listener);
  ros::Rate loop_rate(10);

// %EndTag(SUBSCRIBER)%
  while(ros::ok() and listener.count <=2){
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "After spin: \n";
  sx = listener.print_datax();
  sy = listener.print_datay();

  std::cout << sx << "\n";
  std::cout << sy << "\n";

  return 0;
}
