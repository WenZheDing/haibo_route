#include "obstacle_detection.h"

ObsDetect::ObsDetect(ros::NodeHandle &_n)
{
    n = _n;
    replanFlag.data = false;

    hybridPathSub = n.subscribe("/pathPoints", 1, &ObsDetect::getPath, this);
    globalMapSub = n.subscribe("/simu/global_map", 1, &ObsDetect::getMap, this);

    replanFlagPub = n.advertise<std_msgs::Bool>("/replan_flag", 1, true);
}

void ObsDetect::getMap(const nav_msgs::OccupancyGridConstPtr &map_)
{
    map = map_;
    map_origin_x = map->info.origin.position.x;
    map_origin_y = map->info.origin.position.y;
    map_resolution = map->info.resolution;
    configurationSpace.updateGrid(*map);
    // configurationSpace.viewGrid();
    if (globalPath.size() > 0)
    {
        // ROS_INFO("global path in obstacle detection > 0.");
        for (int i = 0; i < globalPath.size(); i++)
        {
            if (!configurationSpace.isTraversable(&globalPath[i]))
            {
                
                replanFlag.data = false;
                replanFlagPub.publish(replanFlag);
                // ROS_ERROR("REPLANFALG IS %i", replanFlag.data);
                // 在检测到A*路径发生碰撞后，清空之前的路径，防止在A*二次规划时仍在不停发送replanFlag;
                globalPath.clear();
                return;
            }
        }
    }
}

void ObsDetect::getPath(const nav_msgs::Path &global_path)
{
    ROS_INFO("obstacle detection received a new path!");
    int global_points_size = global_path.poses.size();
    globalPath.clear();
    globalPath.resize(global_points_size + 1);
    float t;
    int prim;

    //重新还原node3D对象
    for (int i = 0; i < global_points_size; i++)
    {
        // Node3D node;
        globalPath[i + 1].setX(global_path.poses[i].pose.position.x);
        globalPath[i + 1].setY(global_path.poses[i].pose.position.y);
        t = global_path.poses[i].pose.position.z;
        prim = static_cast<int>(global_path.poses[i].pose.orientation.z);
        // ROS_INFO("t is %.2f, prim is %i", t, prim);
        globalPath[i + 1].setT(t);
        globalPath[i + 1].setPrim(prim);
    }
    // 造点；
    // Node3D new_goal;
    globalPath[0].setX(globalPath[1].getX() - 15 * cos(globalPath[1].getT()));
    globalPath[0].setY(globalPath[1].getY() - 15 * sin(globalPath[1].getT()));
    globalPath[0].setT(globalPath[1].getT());
    globalPath[0].setPrim(4);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "obs_detect");
    ros::NodeHandle _n("obs_detect");
    ObsDetect detector(_n);
    ros::spin();
    return 0;
}