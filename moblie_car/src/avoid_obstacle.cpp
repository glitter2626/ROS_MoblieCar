#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

ros::Publisher twist_pub;

ros::Subscriber odometry_sub;

ros::Subscriber result_sub;


float e = 0;
float old_e = 0;
float E = 0;
const float Kp = 4;
const float Ki = 0.01;
const float Kd = 0.1; 
const float pi = 3.14159265;

const float GOAL_X = 10.0;
const float GOAL_Y = 10.0;

void init_PID(){

    e = 0;
    old_e = 0;
    E = 0;
}

float PID(float obj_theta, float phi){

    float dt = 0.2;

    e = obj_theta - phi;
    
    // make sure -pi ~ pi 
    e = atan2(sin(e*pi/180.0), cos(e*pi/180.0)) * 180.0 / pi;  
    
    // get delta e
    float e_dot = (e - old_e) / dt;
    
    E = E + e * dt;
    
    float w = Kp * e + Kd * e_dot + Ki * E;
    
    old_e = e;

    return w;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::Twist twist_msg;
    
    // TODO
    float delta_theta = atan2((GOAL_Y - msg->pose.pose.position.y), (GOAL_X - msg->pose.pose.position.x)) * 180.0 / pi;
    float x = msg->pose.pose.orientation.x;
    float y = msg->pose.pose.orientation.y;
    float z = msg->pose.pose.orientation.z;
    float w = msg->pose.pose.orientation.w;
    
    float phi = atan2(2*(w*z+x*y), (1-2*(pow(y,2)+pow(z,2)))) * 180.0 / pi;
    float angular_z = PID(delta_theta, phi);
    float v = 0.2;
    
    twist_msg.linear.x = v * cos(phi*pi/180.0);
    twist_msg.linear.y = v * sin(phi*pi/180.0);
    twist_msg.angular.z = angular_z;
    
    
    twist_pub.publish(twist_msg);
}

void resultCallback(const std_msgs::String::ConstPtr& msg){

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "controller");
  
  ros::NodeHandle n;
  
  twist_pub = n.advertise<geometry_msgs::Twist>("/Twist", 2000);

  odometry_sub = n.subscribe("/Odometry", 2000, odometryCallback);
  
  result_sub = n.subscribe("/result", 2000, resultCallback);

  init_PID();

  ros::Rate loop_rate(5);
  

  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
