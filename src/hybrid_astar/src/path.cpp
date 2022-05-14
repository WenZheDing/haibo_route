#include "path.h"

using namespace HybridAStar;

//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear()
{
    Node3D node;
    path.poses.clear();
    pathNodes.markers.clear();
    pathVehicles.markers.clear();
    addNode(node, 0);
    addVehicle(node, 1);
    publishPath(); //发布A*规划的路径
    publishPathNodes();
    publishPathVehicles();
}

////###################################################
////                                         TRACE PATH
////###################################################
//// __________
//// TRACE PATH
//void Path::tracePath(const Node3D* node, int i) {
//  if (i == 0) {
//    path.header.stamp = ros::Time::now();
//  }

//  if (node == nullptr) { return; }

//  addSegment(node);
//  addNode(node, i);
//  i++;
//  addVehicle(node, i);
//  i++;

//  tracePath(node->getPred(), i);
//}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::updatePath(std::vector<Node3D> nodePath)
{
    path.header.stamp = ros::Time::now();
    int k = 0;

    for (size_t i = 0; i < nodePath.size(); ++i)
    {
        addSegment(nodePath[i]);
        addNode(nodePath[i], k);
        k++;
        addVehicle(nodePath[i], k);
        k++;
    }

    return;
}

void Path::publishPathPoints(std::vector<Node3D> nodePath) //此处nodepath节点先后顺序是反的
{
    nav_msgs::Path pathMsg;
    pathMsg.header.frame_id = "path";
    pathMsg.header.stamp = ros::Time::now();
    for (size_t i = 0; i < nodePath.size(); ++i) //从后往前找
    {
        geometry_msgs::PoseStamped pathPoint;
        pathPoint.pose.position.x = nodePath[i].getX();
        pathPoint.pose.position.y = nodePath[i].getY();
        //   pathPoint.pose.position.z = nodePath[i].getT() + 10* nodePath[i].getPrim();
        pathPoint.pose.position.z = nodePath[i].getT();
        pathPoint.pose.orientation.z = static_cast<float>(nodePath[i].getPrim());
        pathMsg.poses.push_back(pathPoint);
        //std::cout << "In path.cpp pubpathpoints: " << "pathPoint.pose.position.x" << pathPoint.pose.position.x << "pathPoint.pose.position.y" << pathPoint.pose.position.y <<std::endl;
    }
    pubPathPoints.publish(pathMsg);
    lastPath = pathMsg;
}

// ___________
// ADD SEGMENT
void Path::addSegment(const Node3D &node)
{
    geometry_msgs::PoseStamped vertex;
    vertex.pose.position.x = node.getX() * Constants::cellSize;
    vertex.pose.position.y = node.getY() * Constants::cellSize;
    //vertex.pose.position.z = 0;
    vertex.pose.position.z = node.getPrim() < 3 ? 1 : -1;
    //--Mapping debug--
    vertex.header.frame_id = "path";
    // 这里rsshot的prim可能有问题

    if (node.getPrim() < 3)
    {
        vertex.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
        //std::cout<<"Yaw:"<<node.getT()<<std::endl;
    }
    else
    {
        vertex.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
        //std::cout<<"Yaw: reverse W"<<node.getT()<<std::endl;
    }

    path.poses.push_back(vertex);
}

// ________
// ADD NODE
void Path::addNode(const Node3D &node, int i)
{
    visualization_msgs::Marker pathNode;

    // delete all previous markers
    if (i == 0)
    {
        pathNode.action = 3;
    }

    pathNode.header.frame_id = "path";
    pathNode.header.stamp = ros::Time(0);
    pathNode.id = i;
    pathNode.type = visualization_msgs::Marker::SPHERE;
    pathNode.scale.x = 0.1;
    pathNode.scale.y = 0.1;
    pathNode.scale.z = 0.1;
    pathNode.color.a = 1.0;

    if (smoothed)
    {
        pathNode.color.r = Constants::pink.red;
        pathNode.color.g = Constants::pink.green;
        pathNode.color.b = Constants::pink.blue;
    }
    else
    {
        pathNode.color.r = Constants::purple.red;
        pathNode.color.g = Constants::purple.green;
        pathNode.color.b = Constants::purple.blue;
    }

    pathNode.pose.position.x = node.getX() * Constants::cellSize;
    pathNode.pose.position.y = node.getY() * Constants::cellSize;
    pathNodes.markers.push_back(pathNode);
}

void Path::addVehicle(const Node3D &node, int i)
{
    visualization_msgs::Marker pathVehicle;

    // delete all previous markersg
    if (i == 1)
    {
        pathVehicle.action = 3;
    }

    pathVehicle.header.frame_id = "path";
    pathVehicle.header.stamp = ros::Time(0);
    pathVehicle.id = i;
    pathVehicle.type = visualization_msgs::Marker::CUBE;
    pathVehicle.scale.x = vehicle_length - Constants::bloating * 2;
    pathVehicle.scale.y = vehicle_width - Constants::bloating * 2;
    pathVehicle.scale.z = 1;
    pathVehicle.color.a = 0.1;

    if (smoothed)
    {
        pathVehicle.color.r = Constants::orange.red;
        pathVehicle.color.g = Constants::orange.green;
        pathVehicle.color.b = Constants::orange.blue;
    }
    else
    {
        pathVehicle.color.r = Constants::teal.red;
        pathVehicle.color.g = Constants::teal.green;
        pathVehicle.color.b = Constants::teal.blue;
    }

    pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
    pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
    pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
    pathVehicles.markers.push_back(pathVehicle);
}

void Path::savePathPointsTocsv(std::vector<Node3D> nodePath) //此处nodepath节点先后顺序是反的
{
    std::ofstream HybridCsv;
    time_t rawtime;
    char name_buffer[80];
    std::time(&rawtime);
    std::tm time_tm;
    localtime_r(&rawtime, &time_tm);
    strftime(name_buffer, 80, "/csv/hybrid_path_in_log__%F_%H%M%S.csv", &time_tm);
    std::string save_path = ros::package::getPath("decision") + name_buffer;
    HybridCsv.open(save_path, std::ios::out);
    for (size_t i = 0; i < nodePath.size(); ++i) //从后往前找
    {
        geometry_msgs::PoseStamped pathPoint;
        pathPoint.pose.position.x = nodePath[i].getX();
        pathPoint.pose.position.y = nodePath[i].getY();
        //   pathPoint.pose.position.z = nodePath[i].getT() + 10* nodePath[i].getPrim();
        pathPoint.pose.position.z = nodePath[i].getT();
        pathPoint.pose.orientation.z = static_cast<float>(nodePath[i].getPrim()) < 3 ? 1 : 2;
        //std::cout << "In path.cpp pubpathpoints: " << "pathPoint.pose.position.x" << pathPoint.pose.position.x << "pathPoint.pose.position.y" << pathPoint.pose.position.y <<std::endl;
        HybridCsv << pathPoint.pose.position.x << ',' << pathPoint.pose.position.y <<
        ',' << pathPoint.pose.position.z/M_PI * 180.0 << ","<< pathPoint.pose.orientation.z + 0 <<'\r';
    }
    HybridCsv.close();
}
