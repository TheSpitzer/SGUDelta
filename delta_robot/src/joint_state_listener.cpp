#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"
#include "vector"
#include "iostream"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

float joint_position1;
float joint_position2;
float joint_position3;
float aux1;
float aux2;
float aux3;

	ros::Publisher pub1_;
	ros::Publisher pub2_;
	ros::Publisher pub3_;

void JointArrayCb(const sensor_msgs::JointState& m)
{
	joint_position1 = m.position[0];
	joint_position2 = m.position[1];
	joint_position3 = m.position[2];
	std_msgs::Float64 aux1;
	aux1.data = joint_position1;
	std_msgs::Float64 aux2;
	aux2.data = joint_position2;
	std_msgs::Float64 aux3;
        aux3.data = joint_position3;
        pub1_.publish(aux1);
        pub2_.publish(aux2);
        pub3_.publish(aux3);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_state_listener");
	ros::NodeHandle nh_;
	ros::Subscriber sub_ = nh_.subscribe("/joint_states", 1, JointArrayCb);
        pub1_ =nh_.advertise<std_msgs::Float64>("/joint1_controller/command", 1);
        pub2_ =nh_.advertise<std_msgs::Float64>("/joint2_controller/command", 1);
        pub3_ =nh_.advertise<std_msgs::Float64>("/joint3_controller/command", 1);


	ros::spin();
	
return 0;
}
