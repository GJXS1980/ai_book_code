#include "ros/ros.h"
#include "castlex_msgs/skeleton.h"
#include "castlex_msgs/joint.h"
#include "geometry_msgs/Twist.h"

#define MIN_DISTANCE 1600 //millimetre
#define MAX_DISTANCE 1700 //millimetre

double threshold = 0.5;
double target_x=0, target_y=0, target_z=0;
bool cb1_flag = false;

void callback(const castlex_msgs::skeleton::ConstPtr& msg)
{
    double x=0, y=0, z=0;
    int counter = 0;
    cb1_flag = true;
    for(int i=0; i< msg->joints.size(); i++) {
        if(msg->joints[i].confidence > threshold) {
            x += msg->joints[i].joint_position.x;
            y += msg->joints[i].joint_position.y;
            z += msg->joints[i].joint_position.z;
            counter += 1;
        }
        }

    x /= counter;
    y /= counter;
    z /= counter;

    target_x = x;
    target_y = y;
    target_z = z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "castlex_follow");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<castlex_msgs::skeleton>("skeleton",10, callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("follow_vel", 10);
    geometry_msgs::Twist twist;
    ros::Rate loop(10);
    while(ros::ok()) 
    {
    if(cb1_flag)
        {
        if(target_z > MAX_DISTANCE)
            {
            twist.linear.x = target_z/10000*0.7;
            } 
        else if(target_z < MIN_DISTANCE)
        	{
        	twist.linear.x = -target_z/10000*0.7;

        	}

        	else
            	{
            	twist.linear.x = 0;
            	}

        if(fabs( atan2(target_x, target_z) ) > 0.01) 
            {
            twist.angular.z = atan2(target_x,target_z)*2.3;
            }
        else 
            { //center
            twist.angular.z = 0;
            }
            cb1_flag = false;
        }

    else
        {
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 0;
            cb1_flag = false;
        }

        pub.publish(twist);
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}
