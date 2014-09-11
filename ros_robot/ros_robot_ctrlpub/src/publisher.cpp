#include <string>
#include <math.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <stdio.h>


#define B_UP 4
#define B_LEFT 7
#define B_RIGHT 5
#define B_DOWN 6

class CRobot_Pub {
public:
    CRobot_Pub();
private:
    void joystick_sub_callback(const sensor_msgs::Joy::ConstPtr& joymsg);
    void robot_pub_callback(const ros::TimerEvent& e);

    ros::NodeHandle n;
    
    ros::Publisher body1_pub;
    ros::Publisher body2_pub;
    ros::Publisher body3_pub;
    ros::Publisher body4_pub;
    
    ros::Publisher wheel1_pub;
    ros::Publisher wheel2_pub;
    ros::Publisher wheel3_pub;
    ros::Publisher wheel4_pub;
    
    
    ros::Subscriber joystick_sub;
    ros::Timer timer;
    ros::Time current_time, last_time;
    
    int DIRECTION;
    // 7 8 9
    // 4 5 6
    // 1 2 3
    std_msgs::Float64 r1, r2, r3, r4, w1, w2, w3, w4;
};

CRobot_Pub::CRobot_Pub()
{
    body1_pub = n.advertise<std_msgs::Float64>("/ros_robot/joint_body1_controller/command",1);
    body2_pub = n.advertise<std_msgs::Float64>("/ros_robot/joint_body2_controller/command",1);
    body3_pub = n.advertise<std_msgs::Float64>("/ros_robot/joint_body3_controller/command",1);
    body4_pub = n.advertise<std_msgs::Float64>("/ros_robot/joint_body4_controller/command",1);
    
    wheel1_pub = n.advertise<std_msgs::Float64>("/ros_robot/joint_wheel1_controller/command",1);
    wheel2_pub = n.advertise<std_msgs::Float64>("/ros_robot/joint_wheel2_controller/command",1);
    wheel3_pub = n.advertise<std_msgs::Float64>("/ros_robot/joint_wheel3_controller/command",1);
    wheel4_pub = n.advertise<std_msgs::Float64>("/ros_robot/joint_wheel4_controller/command",1);
    
    joystick_sub = n.subscribe("/joy0" , 1, &CRobot_Pub::joystick_sub_callback, this);
    timer = n.createTimer(ros::Duration(1/100), &CRobot_Pub::robot_pub_callback, this);
    
    DIRECTION = 5;
}


void CRobot_Pub::joystick_sub_callback(const sensor_msgs::Joy::ConstPtr& joymsg)
{
//buttons : left:7  up:4  right:5  down:6
    DIRECTION = 5;
    
    int up = joymsg->buttons[B_UP];
    int left = joymsg->buttons[B_LEFT];
    int down = joymsg->buttons[B_DOWN];
    int right = joymsg->buttons[B_RIGHT];

    if (up) DIRECTION = 8;
    if (right) DIRECTION = 6;
    if (down) DIRECTION = 2;
    if (left) DIRECTION = 4;
    
    //if (up && right) DIRECTION = 9;
    //if (right && down) DIRECTION = 3;
    //if (down && left) DIRECTION = 1;
    //if (up && left) DIRECTION = 7;
}

void CRobot_Pub::robot_pub_callback(const ros::TimerEvent& e)
{
    switch(DIRECTION)
    {
        case 8:
        {
            r1.data = 0.0; r2.data = 0.0; r3.data = 0.0; r4.data = 0.0;
            w1.data = 5.0; w2.data = 5.0; w3.data = 5.0; w4.data = 5.0;break;
        }
//        case 9:
//        {
//            r1.data = -M_PI/4; r2.data = -M_PI/4; r3.data = 0.0; r4.data = 0.0;
//            w1.data = 5.0; w2.data = 5.0; w3.data = 5.0; w4.data = 5.0;break;
//        }
        case 6:
        {
            r1.data = 0.9828; r2.data = -0.9828; r3.data = 0.9828; r4.data = -0.9828;
            w1.data = -1.0; w2.data = 1.0; w3.data = 1.0; w4.data = -1.0;break;
        }
//        case 3:
//        {
//            r1.data = -M_PI/4; r2.data = -M_PI/4; r3.data = 0.0; r4.data = 0.0;
//            w1.data = -5.0; w2.data = -5.0; w3.data = -5.0; w4.data = -5.0;break;
//        }
        case 2:
        {
            r1.data = 0.0; r2.data = 0.0; r3.data = 0.0; r4.data = 0.0;
            w1.data = -5.0; w2.data = -5.0; w3.data = -5.0; w4.data = -5.0;break;
        }
//        case 1:
//        {
//            r1.data = M_PI/4; r2.data = M_PI/4; r3.data = 0.0; r4.data = 0.0;
//            w1.data = -5.0; w2.data = -5.0; w3.data = -5.0; w4.data = -5.0;break;
//        }
        case 4:
        {
            r1.data = 0.9828; r2.data = -0.9828; r3.data = 0.9828; r4.data = -0.9828;
            w1.data = 1.0; w2.data = -1.0; w3.data = -1.0; w4.data = 1.0;break;
        }
//        case 7:
//        {
//            r1.data = M_PI/4; r2.data = M_PI/4; r3.data = 0.0; r4.data = 0.0;
//            w1.data = 5.0; w2.data = 5.0; w3.data = 5.0; w4.data = 5.0;break;
//        }
        default:
        {
            r1.data = 0.0; r2.data = 0.0; r3.data = 0.0; r4.data = 0.0;
            w1.data = 0.0; w2.data = 0.0; w3.data = 0.0; w4.data = 0.0;break;
        }
    }
    
    body1_pub.publish(r1);
    body2_pub.publish(r2);
    body3_pub.publish(r3);
    body4_pub.publish(r4);
    
    wheel1_pub.publish(w1);
    wheel2_pub.publish(w2);
    wheel3_pub.publish(w3);
    wheel4_pub.publish(w4);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_robot_ctrlpub");
    
    CRobot_Pub robot_pub;
    
    ros::spin();

    return 0;
    }


