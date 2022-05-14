#include "visualize.h"
#include <unistd.h>
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <ros/package.h>

using namespace HybridAStar;
//###################################################
//                                CLEAR VISUALIZATION
//###################################################
namespace HybridAStar
{

  Visualize::Visualize()
  {
    pubNode3D = n.advertise<geometry_msgs::PoseStamped>("/visualizeNodes3DPose", 100);
    pubNodes3D = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPoses", 100);
    pubNodes3Dreverse = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPosesReverse", 100);
    pubNodes3DCosts = n.advertise<visualization_msgs::MarkerArray>("/visualizeNodes3DCosts", 100);
    pubNode2D = n.advertise<geometry_msgs::PoseStamped>("/visualizeNodes2DPose", 100);
    pubNodes2D = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes2DPoses", 100);
    pubNodes2DCosts = n.advertise<visualization_msgs::MarkerArray>("/visualizeNodes2DCosts", 100);
    pubMainRoad = n.advertise<geometry_msgs::PolygonStamped>("/visializeMainRoad", 1, true);
    // CONFIGURE THE CONTAINER
    poses3D.header.frame_id = "path";
    poses3Dreverse.header.frame_id = "path";
    poses2D.header.frame_id = "path";

    std::string decision_package_path = ros::package::getPath("decision");
    Json::Reader json_reader;
    std::ifstream json_file;
    Json::Value root;
    std::string json_file_path;
    std::string strPathName_up, strPathName_down;

    json_file_path = decision_package_path + "/json_files/start_and_goal.json";
    json_file.open(json_file_path);
    if (!json_reader.parse(json_file, root))
    {
      std::cout << "Error opening json file  : " << json_file_path << std::endl;
    }
    strPathName_up = root["strPathName_up"].asString();
    strPathName_down = root["strPathName_down"].asString();

    setMainRoad_visual(decision_package_path + "/json_files/" + strPathName_up +"_mviz.csv", decision_package_path + "/json_files/" + strPathName_down +"_mviz.csv");
    // setMainRoad_visual(decision_package_path + "/json_files/uB_BB95_mviz.csv", decision_package_path + "/json_files/dB_BB95_mviz.csv");
  }

  void Visualize::clear()
  {
    poses3D.poses.clear();
    poses3Dreverse.poses.clear();
    poses2D.poses.clear();

    // 3D COSTS
    visualization_msgs::MarkerArray costCubes3D;
    visualization_msgs::Marker costCube3D;
    // CLEAR THE COST HEATMAP
    costCube3D.header.frame_id = "path";
    costCube3D.header.stamp = ros::Time::now();
    costCube3D.id = 0;
    costCube3D.action = 3;
    costCubes3D.markers.push_back(costCube3D);
    pubNodes3DCosts.publish(costCubes3D);

    // 2D COSTS
    visualization_msgs::MarkerArray costCubes2D;
    visualization_msgs::Marker costCube2D;
    // CLEAR THE COST HEATMAP
    costCube2D.header.frame_id = "path";
    costCube2D.header.stamp = ros::Time::now();
    costCube2D.id = 0;
    costCube2D.action = 3;
    costCubes2D.markers.push_back(costCube2D);
    pubNodes2DCosts.publish(costCubes2D);
  }

  //###################################################
  //                                    CURRENT 3D NODE
  //###################################################
  void Visualize::publishNode3DPose(Node3D &node)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "path";
    pose.header.stamp = ros::Time::now();
    pose.header.seq = 0;
    pose.pose.position.x = node.getX() * Constants::cellSize;
    pose.pose.position.y = node.getY() * Constants::cellSize;

    //FORWARD
    if (node.getPrim() < 3)
    {
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
    }
    //REVERSE
    else
    {
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT() + M_PI);
    }

    // PUBLISH THE POSE
    pubNode3D.publish(pose);
  }

  //###################################################
  //                              ALL EXPANDED 3D NODES
  //###################################################
  void Visualize::publishNode3DPoses(Node3D &node)
  {
    geometry_msgs::Pose pose;
    pose.position.x = node.getX() * Constants::cellSize;
    pose.position.y = node.getY() * Constants::cellSize;

    //FORWARD
    if (node.getPrim() < 3)
    {
      pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
      poses3D.poses.push_back(pose);
      poses3D.header.stamp = ros::Time::now();
      // PUBLISH THE POSEARRAY
      pubNodes3D.publish(poses3D);
    }
    //REVERSE
    else
    {
      pose.orientation = tf::createQuaternionMsgFromYaw(node.getT() + M_PI);
      poses3Dreverse.poses.push_back(pose);
      poses3Dreverse.header.stamp = ros::Time::now();
      // PUBLISH THE POSEARRAY
      pubNodes3Dreverse.publish(poses3Dreverse);
    }
  }

  //###################################################
  //                                    CURRENT 2D NODE
  //###################################################
  void Visualize::publishNode2DPose(Node2D &node)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "path";
    pose.header.stamp = ros::Time::now();
    pose.header.seq = 0;
    pose.pose.position.x = (node.getX() + 0.5) * Constants::cellSize;
    pose.pose.position.y = (node.getY() + 0.5) * Constants::cellSize;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    // PUBLISH THE POSE
    pubNode2D.publish(pose);
  }

  //###################################################
  //                              ALL EXPANDED 2D NODES
  //###################################################
  void Visualize::publishNode2DPoses(Node2D &node)
  {
    if (node.isDiscovered())
    {
      geometry_msgs::Pose pose;
      pose.position.x = (node.getX() + 0.5) * Constants::cellSize;
      pose.position.y = (node.getY() + 0.5) * Constants::cellSize;
      pose.orientation = tf::createQuaternionMsgFromYaw(0);

      poses2D.poses.push_back(pose);
      poses2D.header.stamp = ros::Time::now();
      // PUBLISH THE POSEARRAY
      pubNodes2D.publish(poses2D);
    }
  }

  //###################################################
  //                                    COST HEATMAP 3D
  //###################################################
  void Visualize::publishNode3DCosts(Node3D *nodes, int width, int height, int depth)
  {
    visualization_msgs::MarkerArray costCubes;
    visualization_msgs::Marker costCube;

    float min = 1000;
    float max = 0;
    int idx;
    bool once = true;
    float red = 0;
    float green = 0;
    float blue = 0;
    int count = 0;

    ColorGradient heatMapGradient;
    heatMapGradient.createDefaultHeatMapGradient();

    float values[width * height];

    // ________________________________
    // DETERMINE THE MAX AND MIN VALUES
    for (int i = 0; i < width * height; ++i)
    {
      values[i] = 1000;

      // iterate over all headings
      for (int k = 0; k < depth; ++k)
      {
        idx = k * width * height + i;

        // set the minimum for the cell
        if (nodes[idx].isClosed() || nodes[idx].isOpen())
        {
          values[i] = nodes[idx].getC();
        }
      }

      // set a new minimum
      if (values[i] > 0 && values[i] < min)
      {
        min = values[i];
      }

      // set a new maximum
      if (values[i] > 0 && values[i] > max && values[i] != 1000)
      {
        max = values[i];
      }
    }

    // _______________
    // PAINT THE CUBES
    for (int i = 0; i < width * height; ++i)
    {
      // if a value exists continue
      if (values[i] != 1000)
      {
        count++;

        // delete all previous markers
        if (once)
        {
          costCube.action = 3;
          once = false;
        }
        else
        {
          costCube.action = 0;
        }

        costCube.header.frame_id = "path";
        costCube.header.stamp = ros::Time::now();
        costCube.id = i;
        costCube.type = visualization_msgs::Marker::CUBE;
        values[i] = (values[i] - min) / (max - min);
        costCube.scale.x = Constants::cellSize;
        costCube.scale.y = Constants::cellSize;
        costCube.scale.z = 0.1;
        costCube.color.a = 0.5;
        heatMapGradient.getColorAtValue(values[i], red, green, blue);
        costCube.color.r = red;
        costCube.color.g = green;
        costCube.color.b = blue;
        // center in cell +0.5
        costCube.pose.position.x = (i % width + 0.5) * Constants::cellSize;
        costCube.pose.position.y = ((i / width) % height + 0.5) * Constants::cellSize;
        costCubes.markers.push_back(costCube);
      }
    }

    if (Constants::coutDEBUG)
    {
      std::cout << "3D min cost: " << min << " | max cost: " << max << std::endl;
      std::cout << count << " 3D nodes expanded " << std::endl;
    }

    // PUBLISH THE COSTCUBES
    pubNodes3DCosts.publish(costCubes);
  }

  //###################################################
  //                                    COST HEATMAP 2D
  //###################################################
  void Visualize::publishNode2DCosts(Node2D *nodes, int width, int height)
  {
    visualization_msgs::MarkerArray costCubes;
    visualization_msgs::Marker costCube;

    float min = 1000;
    float max = 0;
    bool once = true;
    float red = 0;
    float green = 0;
    float blue = 0;
    int count = 0;

    ColorGradient heatMapGradient;
    heatMapGradient.createDefaultHeatMapGradient();

    float values[width * height];

    // ________________________________
    // DETERMINE THE MAX AND MIN VALUES
    for (int i = 0; i < width * height; ++i)
    {
      values[i] = 1000;

      // set the minimum for the cell
      if (nodes[i].isDiscovered())
      {
        values[i] = nodes[i].getG();

        // set a new minimum
        if (values[i] > 0 && values[i] < min)
        {
          min = values[i];
        }

        // set a new maximum
        if (values[i] > 0 && values[i] > max)
        {
          max = values[i];
        }
      }
    }

    // _______________
    // PAINT THE CUBES
    for (int i = 0; i < width * height; ++i)
    {
      // if a value exists continue
      if (nodes[i].isDiscovered())
      {
        count++;

        // delete all previous markers
        if (once)
        {
          costCube.action = 3;
          once = false;
        }
        else
        {
          costCube.action = 0;
        }

        costCube.header.frame_id = "path";
        costCube.header.stamp = ros::Time::now();
        costCube.id = i;
        costCube.type = visualization_msgs::Marker::CUBE;
        values[i] = (values[i] - min) / (max - min);
        costCube.scale.x = Constants::cellSize;
        costCube.scale.y = Constants::cellSize;
        costCube.scale.z = 0.1;
        costCube.color.a = 0.5;
        heatMapGradient.getColorAtValue(values[i], red, green, blue);
        costCube.color.r = red;
        costCube.color.g = green;
        costCube.color.b = blue;
        // center in cell +0.5
        costCube.pose.position.x = (i % width + 0.5) * Constants::cellSize;
        costCube.pose.position.y = ((i / width) % height + 0.5) * Constants::cellSize;
        costCubes.markers.push_back(costCube);
      }
    }

    if (Constants::coutDEBUG)
    {
      std::cout << "2D min cost: " << min << " | max cost: " << max << std::endl;
      std::cout << count << " 2D nodes expanded " << std::endl;
    }

    // PUBLISH THE COSTCUBES
    pubNodes2DCosts.publish(costCubes);
  }

  void Visualize::publishMainRoad()
  {
    mainRoad.header.frame_id = "map";
    mainRoad.header.stamp = ros::Time::now();
    geometry_msgs::Point32 uphill;
    geometry_msgs::Point32 downhill;

    for (int i = 0; i < up_road_.size(); i++)
    {
      uphill.x = up_road_.at(i).x;
      uphill.y = up_road_.at(i).y;
      mainRoad.polygon.points.push_back(uphill);
      // std::cout << "up_road_" << downhill.x << std::endl;
    }

    for (int j = 0; j < down_road_.size(); j++)
    {
      downhill.x = down_road_.at(j).x;
      downhill.y = down_road_.at(j).y;
      mainRoad.polygon.points.push_back(downhill);
      // std::cout << "down_road_" << downhill.x << std::endl;
    }

    pubMainRoad.publish(mainRoad);
    up_road_.clear();
    down_road_.clear();
  }

  void Visualize::setMainRoad_visual(std::string up_path, std::string down_path)
  {
    std::ifstream up_file(up_path);
    std::ifstream down_file(down_path);
    std::string one_line;
    RoutePoint line;
    while (getline(up_file, one_line))
    {
      if (!one_line.size())
      {
        break;
      }
      std::istringstream sin(one_line);
      std::vector<std::string> fields;
      std::string field;
      while (getline(sin, field, ','))
      { // 将字符串流sin中的字符读入到field字符串中，以逗号为分隔符

        fields.push_back(field); // 将刚刚读取的字符串添加到向量fields中
      }
      line.x = std::stof(fields[0]);
      line.y = std::stof(fields[1]);
      line.heading = std::stof(fields[2]);

      up_road_.push_back(line);
      // std::cout << "line.heading" <<line.heading <<  std::endl;
      fields.clear();
    }

    while (getline(down_file, one_line))
    {
      if (!one_line.size())
      {
        break;
      }
      std::istringstream sin(one_line);
      std::vector<std::string> fields;
      std::string field;
      while (getline(sin, field, ','))
      { // 将字符串流sin中的字符读入到field字符串中，以逗号为分隔符

        fields.push_back(field); // 将刚刚读取的字符串添加到向量fields中
      }
      line.x = std::stof(fields[0]);
      line.y = std::stof(fields[1]);
      line.heading = std::stof(fields[2]);

      down_road_.push_back(line);
      fields.clear();
    }
    if (!up_road_.size() || !down_road_.size())
    {
      ROS_ERROR("main road infomation fail to get");
      std::cout << " visualize up road size:" << up_road_.size() << std::endl;
      std::cout << " visualize down road size:" << down_road_.size() << std::endl;
    }
  }
}