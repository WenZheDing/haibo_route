#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include "hybrid_astar/node3d.h"
#include "hybrid_astar/collisiondetection.h"

class ObsDetect
{
public:
    ObsDetect(ros::NodeHandle &_n);
    ~ObsDetect(){}

private:
    void getMap(const nav_msgs::OccupancyGridConstPtr &map_);
    void getPath(const nav_msgs::Path &global_path);

    nav_msgs::OccupancyGridConstPtr map;
    std::vector<HybridAStar::Node3D> globalPath;
    std_msgs::Bool replanFlag;

    HybridAStar::CollisionDetection configurationSpace;

    ros::NodeHandle n;
    ros::Subscriber hybridPathSub;
    ros::Subscriber globalMapSub;

    ros::Publisher replanFlagPub;

};

#endif