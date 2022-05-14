#include <ros/ros.h>
#include <string>
#include <vector>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8MultiArray.h>
#include <visualization_msgs/Marker.h>

#include "hybrid_astar/node3d.h"

using namespace std;
// using namespace HybridAStar;

class PathTarget
{
public:
    PathTarget(ros::NodeHandle &node_handle, bool sim = 1);
    ~PathTarget() {}
    void pubTarget();
    std::vector<HybridAStar::Node3D> nodePath; // 逆序存放整条混合A*路径点；

private:
    void getGlobalPoints(const nav_msgs::Path global_points);
    void chooseTarget();
    void pubGoalArea(HybridAStar::Node3D goal_pose);

    bool sim_;

    nav_msgs::Path chosenPath; // 存放关键点的路径，逆序；
    std_msgs::Int8MultiArray TurningIndexes; // 表示第TurningIndexes个关键点时人字形转弯点；

    ros::NodeHandle nh_;
    ros::Publisher pubPath;
    ros::Publisher pubCuspIndex;
    ros::Publisher goal_area_pub;

    ros::Subscriber global_points_sub_;
};