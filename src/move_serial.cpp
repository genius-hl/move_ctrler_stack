#include <iostream>
#include "motor_controller/motor_controller.h"
#include <stdio.h>
#include <termios.h>
#define min(a,b) ((a)<(b)? (a):(b))
#define max(a,b) ((a)>(b)? (a):(b))
void RestoreKeyboardBlocking(struct termios *initial_settings)
{
	tcsetattr(0, TCSANOW, initial_settings);
}

void SetKeyboardNonBlock(struct termios *initial_settings)
{
	//https://gist.github.com/whyrusleeping/3983293
    struct termios new_settings;
    tcgetattr(0,initial_settings);

    new_settings = *initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &new_settings);
}

int scanKeyboard()
{ 
    int ch;
    struct termios oldt;
    //struct termios newt;

    /// Store old settings, and copy to new settings
    //tcgetattr(STDIN_FILENO, &oldt);
    //newt = oldt;
/*
    /// Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);
*/
    SetKeyboardNonBlock(&oldt);
    
    /// Get the current character
    ch = getchar();

    /// Reapply old settings
    //tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    RestoreKeyboardBlocking(&oldt);

    return ch;
}
/*
void recycle_pub(geometry_msgs::Twist &speed)
{
    ros::Rate rate(100);
    while(ros::ok())
    {
        pubSpeed.publish(speed);
        rate.sleep();
    }
    
}*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_serial");
    
    ros::NodeHandle nh;
    
    ROS_INFO("\033[1;32m---->\033[0m key scan Started.");
    
    ros::Publisher pubSpeed=nh.advertise<geometry_msgs::Twist>("/motor_speed",1);
    
    
    //MotorController* motor_ctrler;
    //if(argc==2)
        //motor_ctrler=MotorController::CreateInstance(argv[1]);
    //else
	    //motor_ctrler=MotorController::CreateInstance();
	int key;
	int speed_L=0,speed_R=0;
	float quicken=1.0;
	geometry_msgs::Twist speed_twist, zero_twist;
	speed_twist.linear.x=0;
	speed_twist.linear.y=0;
	speed_twist.linear.z=0;
	speed_twist.angular.x=0;
	speed_twist.angular.y=0;
	speed_twist.angular.z=0;
	zero_twist=speed_twist;
    
	while(ros::ok())
	{
	key=scanKeyboard();
	//printf(" ascii=%d",key);
	    switch(key){
	    
	    //case 0xE0: switch(c=scanKeyboard()){
	    
	        case 'i': speed_L=0; speed_R=-100; break; //head
	        case 'k': speed_L=0; speed_R=100; break;    //back
	        case 'j': speed_L=100; speed_R=0; break;    //left
	        case 'l': speed_L=-100; speed_R=0; break;   //right
	        case 'u': 
			    quicken*=0.8;
			    quicken=min(quicken,10);	//宏替换：quicken<10? quick:10
			    quicken=max(0.01,quicken); 
			    break;
	        case 'n': 
			    quicken*=1.25;
			    quicken=min(quicken,10);
			    quicken=max(0.01,quicken);
			    break;
	        default: break;
	        //}
	    //default:break;
	    }
	    if(key=='s')
        {
            //motor_ctrler->ros_serial->flush();
            speed_L=0;
            speed_R=0;
        }
	    if(key=='\x03')
        {
            speed_L=0;
            speed_R=0;
            break;
        }
        speed_twist.linear.x=int(speed_L*quicken);
        speed_twist.linear.y=int(speed_R*quicken);
        pubSpeed.publish(speed_twist);
	    //motor_ctrler->setSpeed(speed_L,speed_R);
	}
	
	pubSpeed.publish(zero_twist);
	return 0;
}
