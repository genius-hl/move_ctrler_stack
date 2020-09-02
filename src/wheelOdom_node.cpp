#include <iostream>
//#include "motor_controller/motor_controller.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//MotorController* motor_ctrler;

tf::TransformBroadcaster* odom_broadcaster;
ros::Publisher odom_pub;
double v_x=0.0;
double v_y=0.0;
double v_turn=0.0;
double x=0.0;
double y=0.0;
double theta=0.0;
ros::Time current_time,last_time;
void speed_to_odom(geometry_msgs::Twist motorSpeed)
{
	current_time = ros::Time::now();
	double v_mid = (motorSpeed.linear.x+motorSpeed.linear.y) / 2.0;
	//if(v_mid<0.008&&v_mid>-0.008) 
		//v_mid=0;
	v_x = v_mid * cos(theta); //
	v_y = v_mid * sin(theta);
	double v_turnJoin=-motorSpeed.linear.x+motorSpeed.linear.y;
	//if(v_turnJoin<0.008||v_turnJoin>-0.008)
		//v_turnJoin=0;
	v_turn=v_turnJoin/1.5466/0.625;	//左转为正 v_turn=(v_r - v_l)/L; //1.5466 in here is a hardcode coef, to fix 
	
	double dt=(current_time-last_time).toSec();
	double delta_x = v_x * dt;
	double delta_y = v_y * dt;
	//double delta_x = (v_x * cos(theta) - v_y * sin(theta)) * dt;
	//double delta_y = (v_x * sin(theta) + v_y * cos(theta)) * dt;
	double delta_theta = v_turn*dt;
	
	x+=delta_x;
	y+=delta_y;
	theta+=delta_theta;
	//mapping theta to range(0,2*PI)
	//theta=fmod(theta+M_PI,2*M_PI) >= 0 ? (theta) : (theta+3*M_PI) ;
	theta=fmod(theta,2*M_PI);
	if(theta<0)
	    theta+=2*M_PI;
	
	std::cout<<"theta:"<<theta<<std::endl;
	
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
	
	//first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster->sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v_x;
    odom.twist.twist.linear.y = v_y;
    odom.twist.twist.angular.z = v_turn;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
	
}
//TODO
//后期可以写个回调函数，
//接受到更高精度的里程计信息，
//就更新x,y，theta等值,
//消除DR的累计误差。

int main(int argc, char** argv)
{
	ros::init(argc,argv,"wheel_odom");
	
	ros::NodeHandle nh;
	
	ROS_INFO("\033[1;32m---->\033[0m wheel odom Node Started.");
	
	odom_pub=nh.advertise<nav_msgs::Odometry>("odom",50);	//先只用轮式odom，后期融合激光odom
	odom_broadcaster=new tf::TransformBroadcaster(); 
	current_time=ros::Time::now();
	last_time=ros::Time::now();
	ros::Subscriber subGetSpeed=nh.subscribe<geometry_msgs::Twist>("/motor_get_speed",1,&speed_to_odom);
	
	ros::spin();
	
	return 0;
	
}

