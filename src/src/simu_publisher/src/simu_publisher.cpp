#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

geometry_msgs::PoseWithCovarianceStamped start;
geometry_msgs::PoseStamped goal;
ros::Publisher goal_pub;
bool receive_flag;

void pubgoal(const nav_msgs::Path path)
{
	if(receive_flag == false)
	{
		goal_pub.publish(goal);
		receive_flag = true;
	}
	return;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "simu_publisher");
	ros::NodeHandle n;
    ros::Publisher start_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/autostart", 1);;
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("/autogoal", 1);
	ros::Subscriber path_pub = n.subscribe<nav_msgs::Path>("/pathPoints", 1, pubgoal);
	receive_flag = false;
	
	ros::Duration(2.0).sleep();
    start_pub.publish(start);
	goal_pub.publish(goal);
	ros::spin();

	return 0;
}