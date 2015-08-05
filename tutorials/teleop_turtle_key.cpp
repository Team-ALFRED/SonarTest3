#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/String.h" // added by Rachel 
#include "std_msgs/Int16MultiArray.h" // added by Rachel 
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


void chatterCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
  char c;
  bool dirty=false;
  double sensor1 = (msg->data[1]) * 0.004;
  double sensor2 = (msg->data[2]) * 0.004;
  double sensor3 = (msg->data[3]) * 0.004;
  int e_stop = msg->data[4]; 
  if(sensor1 > 0.4)
  {
    sensor1 = 0.4;
  }

  if(sensor2 > 0.4)
  {
    sensor2 = 0.4;
  }
 
  if(sensor3 > 0.4)
  {
    sensor3 = 0.4;
  }

  ROS_INFO("I heard: [%d] [%d] [%d] [%d]", msg->data[1],msg->data[2],msg->data[3],msg->data[4]);
  
  linear = sensor2 - 0.18;

  angular = sensor2 - sensor1 ; 
  dirty = true; 
  if(linear > 0.4)
  {
    linear = 0.4;
  }

  else if(linear < -0.4)
  {
    linear = -0.4;
  }

  else if(linear > 0 && linear < 0.05)
  {
    angular = 0.4; 
  }  


  if(angular > 0.4)
  {
    angular = 0.4;
  }

  if(angular < -0.4)
  {
    angular = -0.4;
  }
  /*if(sensor1 < 50)
  {
    angular = 0.25;
    dirty = true; 
  }
  else if(sensor3 < 50)
  {
    angular = -0.25;
    dirty = true;
  }
  else 
  {
    angular = 0; 
    dirty = true; 
  }
  
  if(sensor2 > 50)
  {
    linear = 0.25; 
    dirty = true; 
  }
  else
  {
    linear = 0;
    dirty = true;
  }
  */
  if(e_stop == 1)
  {
    linear = 0;
    angular = 0; 
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
 return;
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


