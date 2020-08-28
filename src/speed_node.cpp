#include <iostream>
#include "motor_controller/motor_controller.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

MotorController* motor_ctrler;

void Write_speedHandle(geometry_msgs::Twist motorSpeed)
{
    int speed_L=motorSpeed.linear.x;
    int speed_R=motorSpeed.linear.y;
    
    motor_ctrler->setSpeed(speed_L,speed_R);
    
}
int main(int argc, char** argv)
{
    ros::init(argc,argv,"speed_node");
    
    ros::NodeHandle nh;
    
    motor_ctrler=MotorController::CreateInstance();
    
    ROS_INFO("\033[1;32m---->\033[0m Speed Node Started.");
    
    ros::Subscriber subSpeed=nh.subscribe<geometry_msgs::Twist>("/motor_speed",1,&Write_speedHandle);
    
    ros::spin();
    
    return 0;
}
