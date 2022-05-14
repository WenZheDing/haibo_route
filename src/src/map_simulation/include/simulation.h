#ifndef LOCAL_MAP_H
#define LOCAL_MAP_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <iostream>
#include <utility>
#include <vector>
#include <math.h>
#include <map>

using namespace std;

class LocalMap
{
public:
    LocalMap(ros::NodeHandle &node_handle, string &frame_id);
    void pub_LocalMap() { localmap_pub.publish(local_map); };
    void pub_LaserMap() { lasermap_pub.publish(laser_map); };
    void pub_GlobalMap() { global_detectedmap_pub.publish(global_detectedmap); };

private:
    string frame_id_;
    float offsetx;
    float offsety;
    float resolution;
    int width;
    int height;
    int deg;
    int groups;
    multimap<int, int> scan_points; // key值表示位于圆周上哪一组（5度为一组），后一个参数表示点的索引；
    vector<int> border_points;      // 存放边界点在local_points中的索引序号；

    void saveMap(const nav_msgs::OccupancyGridConstPtr &map)
    {
        map_ = map;
        if (!detectedmap_initialize_flag)
        {
            global_detectedmap.info.resolution = map_->info.resolution;
            global_detectedmap.info.width = map_->info.width;
            global_detectedmap.info.height = map_->info.height;
            global_detectedmap.info.origin.position.x = map_->info.origin.position.x;
            global_detectedmap.info.origin.position.y = map_->info.origin.position.y;
            global_detectedmap.data.resize((global_detectedmap.info.width / global_detectedmap.info.resolution) * (global_detectedmap.info.height / global_detectedmap.info.resolution));
            detectedmap_initialize_flag = true;
        }
    };
    // 读取全局地图，并进行坐标转换，将感知范围内的点存入局部地图；
    void get_SimPose(const geometry_msgs::PoseStamped sim_pose); // 获取dwa中最新的车辆位姿，并发布与/map的tf；
    void init_map(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial);
    void update_localmap(); // 更新局部地图；
    // void extract_border();			// 提取每行边界点的索引；
    // void update_multimap();			// 更新边界点的索引；
    void update_lasermap(); // 更新激光雷达地图；
    void update_globalmap();

    ros::NodeHandle n;
    ros::Publisher localmap_pub;
    ros::Publisher lasermap_pub;
    ros::Publisher global_detectedmap_pub;
    ros::Subscriber globalmap_sub;
    ros::Subscriber sim_pose_sub;
    ros::Subscriber start_sub;

    tf::TransformListener listener_;

    std::vector<geometry_msgs::PointStamped> local_points;  // 存储局部地图中每个栅格点在/base坐标系下的xy坐标；
    std::vector<geometry_msgs::PointStamped> global_points; // 存储局部地图中每个栅格点在/map坐标系下的xy坐标；

    nav_msgs::OccupancyGridConstPtr map_;       // 上帝视角全局地图；
    nav_msgs::OccupancyGrid global_detectedmap; // 探测到的全局地图；
    nav_msgs::OccupancyGrid local_map;          // 非雷达视角的无遮挡局部地图；
    nav_msgs::OccupancyGrid laser_map;          // 雷达视角的有遮挡局部地图；
    bool detectedmap_initialize_flag;           //是否初始化探测到的全局地图
};
#endif