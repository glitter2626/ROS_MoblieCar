#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "ros/console.h"
#include <cmath>
#include <string>
#include <utility>

#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)

ros::Publisher twist_pub;

ros::Subscriber odometry_sub;

ros::Subscriber laser_sub;

ros::Subscriber goal_sub;

std::string target_frame = "/base_link";
std::string source_frame = "/map";


float tmp_theta = 0.0;

float e = 0;
float old_e = 0;
float E = 0;
const float Kp = 4;
const float Ki = 0.01;
const float Kd = 0.1; 
const float pi = 3.1415;

float GOAL_X = -2.0;
float GOAL_Y = -0.5;

float x_vector = 0.0;
float y_vector = 0.0;

void init_PID(){

    e = 0;
    old_e = 0;
    E = 0;
}

float PID(float obj_theta, float phi){

    float dt = 0.1;

    e = obj_theta - phi;
    
    // make sure -pi ~ pi 
    e = atan2(sin(e), cos(e));  
    
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
    
    //ROS_INFO("Goal x: %f \t, y: %f\n", GOAL_X, GOAL_Y);
    
    // TODO
    if(fabs(GOAL_Y - msg->pose.pose.position.y) < 0.02  && fabs(GOAL_X - msg->pose.pose.position.x) < 0.02){
        
        twist_msg.linear.x = 0;
        twist_msg.linear.y = 0;
        twist_msg.angular.z = 0;
        ROS_INFO("Stop \n");
        twist_pub.publish(twist_msg);
        
        return;   
    }
    
    float delta_x = GOAL_X - msg->pose.pose.position.x;
    float delta_y = GOAL_Y - msg->pose.pose.position.y;
    // normalize
    float alpha = 0.75;
    delta_x = delta_x / sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    delta_y = delta_y / sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    float delta_theta = atan2(delta_y * alpha + y_vector * (1 - alpha), delta_x * alpha + x_vector * (1 - alpha));
    
    //float delta_theta = atan2((GOAL_Y - msg->pose.pose.position.y), (GOAL_X - msg->pose.pose.position.x));
    float x = msg->pose.pose.orientation.x;
    float y = msg->pose.pose.orientation.y;
    float z = msg->pose.pose.orientation.z;
    float w = msg->pose.pose.orientation.w;
    
    float phi = atan2(2*(w*z+x*y), (1-2*(pow(y,2)+pow(z,2))));
    
    float angular_z = PID(delta_theta, phi);
    float v = 0.2;
    
    //ROS_INFO("delta_theta: %f \t, phi: %f\n", delta_theta, phi);
    //ROS_INFO("angular_z: %f\n", angular_z);
    
    
    twist_msg.linear.x = v;
    //twist_msg.linear.y = v * sin(phi);
    twist_msg.angular.z = angular_z;
    
    
    twist_pub.publish(twist_msg);
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){

    if(msg == NULL)
        return;
    
    GOAL_X = msg->pose.position.x;
    GOAL_Y = msg->pose.position.y; 

}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

    std::pair<float, float> laser_vector[5];
    int size = msg->ranges.size();
    
    x_vector = 0.0;
    y_vector = 0.0;
        
    for(int i = 0, j = 0; i < size; i += int(size/5), j++){
       
            
        float phi = ((float)i / size) * 360;
        phi = DEG2RAD(phi);
        
        if(msg->ranges[i] == std::numeric_limits<float>::infinity()){
            laser_vector[j].first = cos(phi) * msg->range_max;
            laser_vector[j].second = sin(phi) * msg->range_max;
        }else if(j == 0){  
            laser_vector[j].first = cos(phi) * msg->ranges[i] * 0.5;
            laser_vector[j].second = sin(phi) * msg->ranges[i] * 0.5;
            
        }else{
            laser_vector[j].first = cos(phi) * msg->ranges[i];
            laser_vector[j].second = sin(phi) * msg->ranges[i];
        }
        //ROS_INFO("laser No. : %d, value: %f\n", i, msg->ranges[i]);
        x_vector += laser_vector[j].first;
        y_vector += laser_vector[j].second;        
    }
    
    // normalize
    x_vector = x_vector / sqrt(pow(x_vector, 2) + pow(y_vector, 2));
    y_vector = y_vector / sqrt(pow(x_vector, 2) + pow(y_vector, 2));
    
    ROS_INFO("x_vector: %f, y_vector: %f\n", x_vector, y_vector);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "controller");
  
  ros::NodeHandle n;
  
  twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 2000);

  odometry_sub = n.subscribe("/odom", 2000, odometryCallback);
  
  goal_sub = n.subscribe("move_base_simple/goal", 2000, goalCallback);
  
  laser_sub = n.subscribe("/scan", 2000, laserCallback);  // !

  init_PID();

  ros::Rate loop_rate(10);
  

  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
