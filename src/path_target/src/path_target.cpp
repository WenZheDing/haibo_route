#include "path_target.h"

PathTarget::PathTarget(ros::NodeHandle &node_handle, bool sim)
{
    sim_ = sim;

    nh_ = node_handle;

    pubPath = nh_.advertise<nav_msgs::Path>("/path_target/keypoints_map", 1, true);
    pubCuspIndex = nh_.advertise<std_msgs::Int8MultiArray>("/path_target/cusp_index", 1, true);
    goal_area_pub = nh_.advertise<visualization_msgs::Marker>("/path_target/goal_area", 1, true);

    global_points_sub_ = nh_.subscribe("/pathPoints", 1, &PathTarget::getGlobalPoints, this);

    chosenPath.poses.clear();
    TurningIndexes.data.clear();
}

void PathTarget::getGlobalPoints(const nav_msgs::Path global_points)
{
    ROS_INFO("Get pathpoints!");
    int global_points_size = global_points.poses.size();
    if (global_points_size == 0)
    {
        // ROS_ERROR("GLOBAL POINTS SIZE = 0!");
        ROS_ERROR("start or goal will collide! Please give a new valid goal or start point!!!");
        chosenPath.poses.resize(1);
        pubPath.publish(chosenPath);
        return;
    }

    TurningIndexes.data.clear();
    chosenPath.poses.clear();
    nodePath.clear();
    TurningIndexes.data.push_back(-1);
    
    int prim, count = 2;
    float t;
    int i = 0;
    bool flag = true;
    float Yaw_thresh = 0.9; // 1.0；

    nodePath.resize(global_points_size + 1);
    //重新还原node3D对象
    for (int i = 0; i < global_points_size; i++)
    {
        // Node3D node;
        nodePath[i + 1].setX(global_points.poses[i].pose.position.x);
        nodePath[i + 1].setY(global_points.poses[i].pose.position.y);
        t = global_points.poses[i].pose.position.z;
        prim = static_cast<int>(global_points.poses[i].pose.orientation.z);
        nodePath[i + 1].setT(t);
        nodePath[i + 1].setPrim(prim);
    }
    // 造点；
    // Node3D new_goal;
    nodePath[0].setX(nodePath[1].getX() - 15 * cos(nodePath[1].getT()));
    nodePath[0].setY(nodePath[1].getY() - 15 * sin(nodePath[1].getT()));
    nodePath[0].setT(nodePath[1].getT());
    nodePath[0].setPrim(4);
    nodePath.erase(nodePath.begin());
    // ROS_INFO("converse 144 !");
    std::vector<HybridAStar::Node3D> nodePath2;         // 存放混合A*路径上的关键点；
    
    //nodePath2.push_back(nodePath[0]);
    float lastT = nodePath[1].getT() > 0 ? nodePath[1].getT() : 2 * M_PI + nodePath[1].getT();
    nodePath2.push_back(nodePath[1]); // 将终点作为关键点；

    //筛选关键点
    for (size_t i = 2; i < nodePath.size() - 1; ++i)
    {
        // 判断路径点是否发生方向变化（前进、后退）；
        if ((nodePath[i - 1].getPrim() < 3) ^ (nodePath[i].getPrim() < 3))//^XOR if the bit is same=0,else1;
        {
            std::cout << "前后变化 Last Yaw angle is " << lastT << ", current angle is " << nodePath[i].getT() << std::endl;
            // lastT = -nodePath[i].getT() > 0 ? -nodePath[i].getT() : 6.28 + (-nodePath[i].getT());
            lastT = nodePath[i].getT() > 0 ? nodePath[i].getT() : 2 * M_PI + nodePath[i].getT();
            nodePath2.push_back(nodePath[i]);
            TurningIndexes.data.push_back(count);
            cout << "TurningIndex is: " << count << endl;
            count++;
            continue;
        }

        // 判断车辆转角是否发生较大偏转，若是，则作为关键点；
        if ((nodePath[i].getT() < 0 && min(fabs(2 * M_PI + nodePath[i].getT() - lastT), 2 * M_PI - fabs(2 * M_PI + nodePath[i].getT() - lastT)) > Yaw_thresh) ||
            (nodePath[i].getT() > 0 && min(double(fabs(nodePath[i].getT() - lastT)), 2 * M_PI - fabs(nodePath[i].getT() - lastT)) > Yaw_thresh))
        {
            std::cout << "偏转 Last Yaw angle is " << lastT << ", current angle is " << nodePath[i].getT() << std::endl;
            lastT = nodePath[i].getT() > 0 ? nodePath[i].getT() : 2 * M_PI + nodePath[i].getT();
            nodePath2.push_back(nodePath[i]);
            count++;
        }
    }
    nodePath2.push_back(nodePath[nodePath.size() - 1]); // 将起点作为名义上关键点(逆序最后一个)，实际上target_index_(正序)从1开始；

    pubGoalArea(nodePath[0]);

    // 将关键点从Node3d中存放到nav_msgs::Path中；
    for (size_t i = 0; i < nodePath2.size(); ++i)
    {
        geometry_msgs::PoseStamped vertex;
        vertex.pose.position.x = nodePath2[i].getX() * HybridAStar::Constants::cellSize;
        vertex.pose.position.y = nodePath2[i].getY() * HybridAStar::Constants::cellSize;
        //vertex.pose.position.z = 0;
        vertex.pose.position.z = nodePath2[i].getPrim() < 3 ? 1 : -1;
        std::cout << "nodePath2[i].getX() :  " <<nodePath2[i].getX() << "       " <<"nodePath2[i].getY():" << nodePath2[i].getY() << std::endl;
        //--Mapping debug--
        vertex.header.frame_id = "path";
        // 这里rsshot的prim可能有问题
        if (nodePath2[i].getPrim() < 3)
        {
            vertex.pose.orientation = tf::createQuaternionMsgFromYaw(nodePath2[i].getT());
            //std::cout<<"Yaw:"<<node.getT()<<std::endl;
        }
        else
        {
            vertex.pose.orientation = tf::createQuaternionMsgFromYaw(nodePath2[i].getT() + M_PI);
            //std::cout<<"Yaw: reverse W"<<node.getT()<<std::endl;
        }
        chosenPath.poses.push_back(vertex);
    }
    pubPath.publish(chosenPath);
    pubCuspIndex.publish(TurningIndexes);

    return;
}

void PathTarget::pubTarget()
{
    if (chosenPath.poses.empty())
    {
        // ROS_INFO("Waiting for valid local path");
        return;
    }
    /**
     * 发布map中base的位姿
     * 仿真模式下，在接收到init pose之前，发布默认tf.
     * 接收到init_pose之后，由dwa_planner的仿真部分开始发pose get_sim_pose 发位置
     */
    return;
}

// 可视化期望的停车范围；
void PathTarget::pubGoalArea(HybridAStar::Node3D goal_pose)
{
    visualization_msgs::Marker ladderShape;
    float back_hori = 2.575 + 1;
    float back_ver = 3.1 + 0.6;
    float forward_hori = 2.575 + 0.5;
    float forward_ver = 7.535 + 0.6;
    ladderShape.header.frame_id = "map";
    ladderShape.header.stamp = ros::Time::now();

    ladderShape.type = visualization_msgs::Marker::LINE_LIST;
    ladderShape.color.r = 0.0 / 255.0;
    ladderShape.color.g = 0.0 / 255.0;
    ladderShape.color.b = 0.0 / 255.0;
    ladderShape.color.a = 1.5;
    ladderShape.scale.x = 0.2;

    int lineComb[4][2] = {{0, 1}, {0, 2}, {2, 3}, {1, 3}};

    float theta = goal_pose.getT() > 0 ? goal_pose.getT() : 6.28 + goal_pose.getT();
    float x = goal_pose.getX() * HybridAStar::Constants::cellSize;
    float y = goal_pose.getY() * HybridAStar::Constants::cellSize;
    geometry_msgs::Point points[4];
    points[0].x = x + (back_hori)*sin(-theta) - (back_ver)*cos(-theta);
    points[0].y = y + (back_hori)*cos(-theta) + (back_ver)*sin(-theta);
    points[0].z = 1;
    points[1].x = x - (back_hori)*sin(-theta) - (back_ver)*cos(-theta);
    points[1].y = y - (back_hori)*cos(-theta) + (back_ver)*sin(-theta);
    points[1].z = 1;
    points[2].x = x + (forward_hori)*sin(-theta) + (forward_ver)*cos(-theta);
    points[2].y = y + (forward_hori)*cos(-theta) - (forward_ver)*sin(-theta);
    points[2].z = 1;
    points[3].x = x - (forward_hori)*sin(-theta) + (forward_ver)*cos(-theta);
    points[3].y = y - (forward_hori)*cos(-theta) - (forward_ver)*sin(-theta);
    points[3].z = 1;

    for (int i = 0; i < 4; i++)
    {
        ladderShape.points.push_back(points[lineComb[i][0]]);
        ladderShape.points.push_back(points[lineComb[i][1]]);
    }
    goal_area_pub.publish(ladderShape);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "llllll");
    ros::NodeHandle nh("path_target");

    PathTarget path_target(nh, 1);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        path_target.pubTarget();
        loop_rate.sleep();
    }
    return 0;
}