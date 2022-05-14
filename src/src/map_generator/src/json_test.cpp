/*
 * @Descripttion: 
 * @version: 
 * @Author: sueRimn
 * @Date: 2021-12-25 20:31:21
 * @LastEditors: sueRimn
 * @LastEditTime: 2022-01-19 14:53:21
 */
#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>                 //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>                  //体素滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>         //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>      //半径滤波器头文件
#include <json/json.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "json_test");
  ros::NodeHandle nh;

  ros::Rate loop_rate(1.0);
  ros::Publisher occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
  std::string map_generator_package_path = ros::package::getPath("map_generator");
  std::string json_file_path = map_generator_package_path + "/src/n95global_map_empty.json";
  // std::string json_file_path = map_generator_package_path + "/src/bb95global_map.json";
  nav_msgs::OccupancyGrid global_map;

  Json::Reader json_reader;
  std::ifstream json_file;
  json_file.open(json_file_path);
  Json::Value root;

  if (!json_reader.parse(json_file, root))
  {
    std::cout << "Error opening json file : " << json_file_path << std::endl;
  }

  int width, height, size;
  float resolution;
  global_map.info.width = root["width"].asInt();
  global_map.info.height = root["height"].asInt();
  global_map.info.resolution = root["resolution"].asFloat();
  global_map.header.frame_id = "/map";
  global_map.info.origin.position.x = root["origin.x"].asDouble();
  global_map.info.origin.position.y = root["origin.y"].asDouble();
  global_map.info.origin.position.z = 0.0;
  size = root["data"].size();

  for (int i = 0; i < size; i++)
  {
    global_map.data.push_back(root["data"][i].asInt());
  }

  while (ros::ok())
  {
    occupancy_grid_pub.publish(global_map);

    loop_rate.sleep();

    ros::spinOnce();
  }

  return 0;
}
