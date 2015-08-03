#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/String.h" // added by Rachel 
#include "std_msgs/UInt64.h" // added by Rachel 
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


void chatterCallback(const std_msgs::UInt64::ConstPtr& msg)
{
  char c;
  bool dirty=false;
  unsigned int sensor1 = 0; // left 
  unsigned int sensor2 = 0; // middle
  unsigned int sensor3 = 0; // right

  sensor1 = (msg->data >> 14) & 0x3f; 
  sensor2 = (msg->data >> 7) & 0x3f;
  sensor3 = msg->data & 0x3f;  

  ROS_INFO("I heard: [%d] = [%d] [%d] [%d]", msg->data,sensor1,sensor2,sensor3);
  
/*  if(sensor1 < 40)
  {
    angular = 0.25;
    dirty = true; 
  }
  else if(sensor3 < 40)
  {
    angular = -0.25;
    dirty = true;
  }
  else 
  {
    angular = 0; 
    dirty = true; 
  }
  
  if(sensor2 > 40)
  {
    linear = 0.25; 
    dirty = true; 
  }
  else
  {
    linear = 0;
    dirty = true;
  }
  
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale*angular;
  twist.linear.x = l_scale*linear;
  dirty = true; 
  if(dirty ==true)
  {
    twist_pub_.publish(twist);    
    dirty=false;
  }
 */ return;
}


/* end */ 


void mySigintHandler(int sig)
{
  ros::shutdown();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  ros::NodeHandle n; 
  ros::NodeHandle nh_;
  signal(SIGINT, mySigintHandler);

  nh_.param("scale_angular", a_scale, a_scale);
  nh_.param("scale_linear", l_scale, l_scale);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);


  ros::Subscriber sub = n.subscribe("sensor", 10, chatterCallback);
  ros::spin();
  return(0);
}


