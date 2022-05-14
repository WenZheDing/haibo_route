#include "simulation.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "lidar_sim");
	ros::NodeHandle n;
	string frame_id = "base";

	LocalMap LM(n, frame_id);

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		LM.pub_LocalMap();
		LM.pub_LaserMap();
		LM.pub_GlobalMap();
		loop_rate.sleep();
	}

	return 0;
}