#include <string>
#include <math.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include<gazebo_msgs/GetModelState.h>

#define B_UP 4
#define B_LEFT 7
#define B_RIGHT 5
#define B_DOWN 6

class COdom_Pub {
public:
    COdom_Pub();
private:
    void Odom_pub_callback(const ros::TimerEvent& e);

    ros::NodeHandle n;
    
    ros::Publisher odom_pub;
    
    ros::Timer timer;
    
    ros::Time current_time, last_time;
    
    tf::TransformBroadcaster odom_broadcaster;
    
    ros::ServiceClient client_getmodel;
    
    gazebo_msgs::GetModelState modelState;
    
};

COdom_Pub::COdom_Pub()
{
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    
    timer = n.createTimer(ros::Duration(1/10), &COdom_Pub::Odom_pub_callback, this);
    
    client_getmodel = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    
    modelState.request.model_name="ros_robot";
    
}


void COdom_Pub::Odom_pub_callback(const ros::TimerEvent& e)
{
    client_getmodel.call(modelState);
    
    current_time = ros::Time::now();
    //pub the tf of an Odom frame to base link
    //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base";
    
    odom_trans.transform.translation.x = modelState.response.pose.position.x;
    odom_trans.transform.translation.y = modelState.response.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = modelState.response.pose.orientation;
    
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    
    
    //pub the odometry msg
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    
    //set the position
    odom.pose.pose = modelState.response.pose;
    odom.pose.pose.position.z = 0.0;
    
    //set the velocity
    odom.child_frame_id = "base";
    odom.twist.twist = modelState.response.twist;
    //publish the message
    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_robot_Odompub");
    
    COdom_Pub odom_pub;
    
    ros::spin();

    return 0;
    }


