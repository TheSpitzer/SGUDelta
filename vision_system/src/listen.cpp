#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"

float buffer1[2];
float vel;
float velocity;
int x = 0;
ros::Publisher speed_;
geometry_msgs::Twist Twist;

void data_buffer(geometry_msgs::Twist buff)
{
	buffer1[x] = buff.linear.x;
	buffer1[x] = buff.linear.y;
	x++;
	if (x == 2)
	{
		x = 0;
	}

}

//void velCb(geometry_msgs::Vector3Stamped v)
//{
//	vel = v.x;
//	geometry_msgs::Twist velocity;
//	velocity.angular.x = vel;
//	speed_.publish(velocity);
//}

void CoordinateCb(const geometry_msgs::Twist::ConstPtr& m)
{
		if (m->linear.y > 70.0 && m->linear.y < 120.0)
		{

			ROS_INFO("Coordinate is: X: %f Y: %f", m->linear.x, m->linear.y);

		}
		else
		{
		}
		ros::Duration(0.3).sleep();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coordinate_publisher");
	ros::NodeHandle nh_;
	ros::Subscriber sub_ = nh_.subscribe("/coordinate", 1, CoordinateCb);
//	ros::Subscriber rpm_ = nh_.subscribe("/rpm", 100, velCb);
//	speed_ = nh_.advertise<geometry_msgs::Twist>("/coordinate", 1);
	ros::spin();
return 0;
}
