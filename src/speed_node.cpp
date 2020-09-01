//speed_node.cpp 负责将速度写入到电机&&读取电机速度
#include <iostream>
#include "motor_controller/motor_controller.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

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
    
    ros::Subscriber sub_setSpeed=nh.subscribe<geometry_msgs::Twist>("/motor_set_speed",1,&Write_speedHandle);
    
    ros::Publisher speed_pub=nh.advertise<geometry_msgs::Twist>("/motor_get_speed",50);	//先只用轮式odom，后期融合激光odom
    
    //tf::TransformBroadcaster odom_broadcaster;
    int speed_getL=0;
    int speed_getR=0;
    geometry_msgs::Twist speed_get;
    speed_get.linear.x=0;
    speed_get.linear.y=0;
    speed_get.linear.z=0;
   	speed_get.angular.x=0;
    speed_get.angular.y=0;
    speed_get.angular.z=0;
    ros::Rate rate(200);    //control frequence 200Hz
    while(ros::ok())
    {
    
    	motor_ctrler->getSpeed(speed_getL,speed_getR);

    	speed_get.linear.x=(double)speed_getL*0.0004;//TIRE_LONG/40.0; //0.0004 can be adjust further
    	speed_get.linear.y=(double)speed_getR*0.0004;//TIRE_LONG/40.0;
    	speed_pub.publish(speed_get);

    	ros::spinOnce();
    	rate.sleep();
    }
    
    
    
    return 0;
}

/*---------机器人参数-----------*/
//总长95cm;总宽68cm;高36cm
//行驶速度0-4.5km/h
//轮胎 外径32cm 宽6cm
//-----------机器人模型-------//
//轮胎周长：pi*32cm~=1.0053096m
//速度区间0-1.25m/s
//编码器报告数值与实际转速比值：300/7.5=40？？？
//encoder report value relate velocity 300(RPM?):1.2(m/s)

