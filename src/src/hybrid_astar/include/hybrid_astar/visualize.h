#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

#include "gradient.h"
#include <iostream>
#include <fstream>
#include "node3d.h"
#include "node2d.h"
#include "data_type.h"
namespace HybridAStar
{
  class Node3D;
  class Node2D;
  /*!
   \brief A class for visualizing the hybrid A* search.

  Depending on the settings in constants.h the visualization will send different amounts of detail.
  It can show the 3D search as well as the underlying 2D search used for the holonomic with obstacles heuristic.
*/
  class Visualize
  {
  public:
    // ___________
    // CONSTRUCTOR
    /// The default constructor initializing the visualization object and setting publishers for the same.
    Visualize();

    // CLEAR VISUALIZATION
    /// Clears the entire visualization
    void clear();
    /// Clears the 2D visualization
    void clear2D() { poses2D.poses.clear(); }

    // PUBLISH A SINGLE/ARRAY 3D NODE TO RViz
    /// Publishes a single node to RViz, usually the one currently being expanded
    void publishNode3DPose(Node3D &node);
    /// Publishes all expanded nodes to RViz
    void publishNode3DPoses(Node3D &node);
    // PUBLISH THE COST FOR A 3D NODE TO RViz
    /// Publishes the minimum of the cost of all nodes in a 2D grid cell
    void publishNode3DCosts(Node3D *nodes, int width, int height, int depth);

    // PUBLISH A SINGEL/ARRAY 2D NODE TO RViz
    /// Publishes a single node to RViz, usually the one currently being expanded
    void publishNode2DPose(Node2D &node);
    /// Publishes all expanded nodes to RViz
    void publishNode2DPoses(Node2D &node);
    // PUBLISH THE COST FOR A 2D NODE TO RViz
    /// Publishes the minimum of the cost of all nodes in a 2D grid cell
    void publishNode2DCosts(Node2D *nodes, int width, int height);

    void publishMainRoad();

    void setMainRoad_visual(std::string up_path, std::string down_path);
    std::vector<RoutePoint> up_road_;
    std::vector<RoutePoint> down_road_;

  private:
    /// A handle to the ROS node
    ros::NodeHandle n;
    /// Publisher for a single 3D node
    ros::Publisher pubNode3D;
    /// Publisher for an array of 3D forward nodes
    ros::Publisher pubNodes3D;
    /// Publisher for an array of 3D reaward nodes
    ros::Publisher pubNodes3Dreverse;
    /// Publisher for an array of 3D cost with color gradient
    ros::Publisher pubNodes3DCosts;
    /// Publisher for a single 2D node
    ros::Publisher pubNode2D;
    /// Publisher for an array of 2D nodes
    ros::Publisher pubNodes2D;
    /// Publisher for an array of 2D cost with color gradient
    ros::Publisher pubNodes2DCosts;

    ros::Publisher pubMainRoad;
    /// Array of poses describing forward nodes
    geometry_msgs::PoseArray poses3D;
    /// Array of poses describing reaward nodes
    geometry_msgs::PoseArray poses3Dreverse;
    /// Array of poses describing 2D heuristic nodes
    geometry_msgs::PoseArray poses2D;
    geometry_msgs::PolygonStamped mainRoad;
  };
}
#endif // VISUALIZE_H
