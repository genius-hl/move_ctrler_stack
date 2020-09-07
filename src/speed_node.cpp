//speed_node.cpp 负责将速度写入到电机&&读取电机速度
#include <iostream>
#include "motor_controller/motor_controller.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

MotorController* motor_ctrler;
bool Trust_ROSNAV=false;
double MAX_v_x=0.2; double MAX_v_th=0.15;
void Write_speedHandle(geometry_msgs::Twist motorSpeed)
{
    int speed_L=motorSpeed.linear.x;
    int speed_R=motorSpeed.linear.y;
    
    motor_ctrler->setSpeed(speed_L,speed_R);
    
}

/*--------Handle function---------*/
//  recieve message "cmd_vel"
//  velocity command from move_base
/*--------------------------------*/
void move_base_speedHandle(geometry_msgs::Twist cmd_vel)
{
    //ROS_INFO("cmd_vel receieved!");
    int speed_R=0;
    int speed_L=0;
    if(!Trust_ROSNAV)
        return;
    //cmd_vel_to_speed();
    if(cmd_vel.linear.y!=0 || cmd_vel.linear.z!=0 || 
        cmd_vel.angular.x!=0 || cmd_vel.angular.y!=0)
    {
        std::cerr<<"WARN: illegal action"<<std::endl;   //use ROS_ERROR?
        return;
    }
    //if need rotate, rotate first? //TODO
    if(cmd_vel.angular.z<-0.1||cmd_vel.angular.z>0.1)
    //if(std::abs(cmd_vel.angular.z) == MAX_v_th )
    {
         speed_R=cmd_vel.angular.z*1.70*0.62/2*2400/1.0053096;
         speed_L=-speed_R;
         //speed_R=speed_R
         //speed_L=speed_L*2400/1.0053096;
    }/*
    else if(std::abs(cmd_vel.linear.x) == MAX_v_x)    //then heading or stepback
    {
        
        if( (cmd_vel.linear.x < (-0.333333)) || (cmd_vel.linear.x > 0.333333) )
        {
            if(cmd_vel.linear.x>0)
            speed_R=800;    //1/3*2400/1.0053096;    //should limit the max_vel to 1/3 i.e 800RPM for encoder
            else 
            speed_R=-800;   //-1/3*2400/1.0053096; 
            
            speed_L=speed_R;
        }
        else
        {
            speed_R=cmd_vel.linear.x*2400/1.0053096;    //should limit the max_vel to 1/3 i.e 800RPM for encoder
            speed_L=speed_R;
        }
        
    }
    else
    {
        if((std::abs(cmd_vel.linear.x) / MAX_v_x) > std::abs(cmd_vel.angular.z) / MAX_v_th)
        {
            speed_R=cmd_vel.linear.x*2400/1.0053096;    //should limit the max_vel to 1/3 i.e 800RPM for encoder
            speed_L=speed_R;
        }
        else
        {
            speed_R=cmd_vel.angular.z*1.70*0.62/2*2400/1.0053096;
            speed_L=-speed_R;
        }
    }*/
    else if( std::abs(cmd_vel.linear.x) > 0.000001 )
    {
        if( (cmd_vel.linear.x < (-0.333333)) || (cmd_vel.linear.x > 0.333333) )
        {
            if(cmd_vel.linear.x>0)
            speed_R=800;    //1/3*2400/1.0053096;    //should limit the max_vel to 1/3 i.e 800RPM for encoder
            else 
            speed_R=-800;   //-1/3*2400/1.0053096; 
            
            speed_L=speed_R;
        }
        else
        {
            speed_R=cmd_vel.linear.x*2400/1.0053096;    //should limit the max_vel to 1/3 i.e 800RPM for encoder
            speed_L=speed_R;
        }
    }
    else
    {
         speed_R=cmd_vel.angular.z*1.70*0.62/2*2400/1.0053096;
         speed_L=-speed_R;
    }
    //the motor controller set speed format is “!m 0 x” or “!m x 0” 
    //if((speed_R * speed_L)>0)	//heading or backing
    if((speed_R>0&&speed_L>0) || (speed_R<0&&speed_L<0))
    {
       	speed_L=0;
    	speed_R=-speed_R/3;	//!m 0 -100时关系是3,但其他命令好像不是，非线性的 TODO

    }
    else if((speed_R>0&&speed_L<0) || (speed_R<0&&speed_L>0))	//turning
    {
    	speed_L=-speed_L/3;
    	speed_R=0;
    }
    std::cout<<"speed_L: "<<speed_L<<" speed_R: "<<speed_R<<std::endl;
    motor_ctrler->setSpeed(speed_L,speed_R);
    
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"speed_node");
    
    ros::NodeHandle nh;
    
    motor_ctrler=MotorController::CreateInstance();
    
    ROS_INFO("\033[1;32m---->\033[0m Speed Node Started.");
    
    ros::Subscriber sub_setSpeed=nh.subscribe<geometry_msgs::Twist>("/motor_set_speed",1,&Write_speedHandle);
    //import move_base velocity command
    ros::Subscriber sub_cmdVel=nh.subscribe<geometry_msgs::Twist>("/cmd_vel",1,&move_base_speedHandle);
    ros::param::set("Trust_ROSNAV",false);
    ros::param::get("/move_base/TrajectoryPlannerROS/max_vel_x",MAX_v_x);
    ros::param::get("/move_base/TrajectoryPlannerROS/max_vel_x",MAX_v_th);
    
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
    ros::Rate rate(1000);    //control frequence 200Hz
    while(ros::ok())
    {
        if(!ros::param::get("Trust_ROSNAV",Trust_ROSNAV))
            std::cerr<<"cannot get parameter /speed_node/Trust_ROSNAV"<<std::endl;
        //std::cout<<"Trust_ROSNAV: "<<Trust_ROSNAV<<std::endl;
    	motor_ctrler->getSpeed(speed_getL,speed_getR);
		
		//正确的关系应该是：
		//编码器返回值speed_getL和speed_getR为减速器的每分钟转速X|Y RPM
		//减速器减速比为40：1
		//因此轮子的每秒转速X/60/40 | Y/60/40 RPS
		//而轮子1圈（1R）为pi*2*r=pi*2*0.32约=1.0053096米
		//因此线速度：
		speed_get.linear.x=(double)speed_getL / 2400.0 * 1.0053096;
		speed_get.linear.y=(double)speed_getR / 2400.0 * 1.0053096;
		//如果要求角速度，w=v/r就行
    	//speed_get.linear.x=(double)speed_getL*0.0004;//TIRE_LONG/40.0; //0.0004 can be adjust further
    	//speed_get.linear.y=(double)speed_getR*0.0004;//TIRE_LONG/40.0;
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

