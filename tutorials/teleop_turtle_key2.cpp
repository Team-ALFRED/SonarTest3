#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/String.h" // added by Rachel 
#include "std_msgs/Int32.h" // added by Rachel 
#include <sstream>


int kfd = 0;
struct termios cooked, raw;
ros::Publisher twist_pub_;
double linear = 0;
double angular = 0;
double l_scale = 2.0; 
double a_scale = 2.0;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


/* added by Rachel */


void chatterCallback2(const std_msgs::Int32::ConstPtr& msg)
{
  char c;
  bool dirty=false;

  ROS_INFO("I heard: [%d]", msg->data);
  if(msg->data < 20)
  {
    linear = 1.0;
    dirty = true; 
  }
  else if(msg->data > 20)
  {
    linear = 0; 
    dirty = true;
  }

  geometry_msgs::Twist twist;
//  twist.angular.z = a_scale*angular;
  twist.linear.x = l_scale*linear;
  dirty = true; 
  if(dirty ==true)
  {
    twist_pub_.publish(twist);    
    dirty=false;
  }
  return;
}


/* end */ 


void mySigintHandler(int sig)
{
  ros::shutdown();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle2");
  ros::NodeHandle n2_; 
  ros::NodeHandle nh_2;
  signal(SIGINT, mySigintHandler);

  nh_2.param("scale_angular", a_scale, a_scale);
  nh_2.param("scale_linear", l_scale, l_scale);
  twist_pub_ = nh_2.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);


  ros::Subscriber sub2 = n2_.subscribe("sensor2", 10, chatterCallback2);
  ros::spin();
  return(0);
}


