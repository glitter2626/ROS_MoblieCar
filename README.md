# ROS_MoblieCar
## Simulation Enviroment:  
* Gazebo & turtlebot3: roslaunch turtlebot3_gazebo turtlebot3_world.launch

![image](https://github.com/glitter2626/ROS_MoblieCar/blob/master/simulation_enviroment.png)

## Move From Point A to Point B:
* tf tree(base_link -> laser): roslaunch moblie_car moblie_car.launch 
* rviz: rosrun rviz rviz
* mapping: rosrun gmapping slam_gmapping
* publish goal topic: click "2D Nav Goal" button to implement 

![image](https://github.com/glitter2626/ROS_MoblieCar/blob/master/rviz_goal.png)

## Method:
* First: calculate Point A & Point B vector  
* Then: summing per radian vector by checking laser data values   
* Final: combine two vector 
* * *
Maybe not a good method because mobile car may collide in some situation & paramater relationship  

