#ifndef PARAM_H
#define PARAM_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vehicle_msgs/adm_lat.h>

namespace Decision
{
    enum STATE_TYPE
    {
        INITIAL_STATE,
        HYBRID_ASTAR_STATE,
        GENERAL_KEYPOINT_STATE,
        CUSP_KEYPOINT_STATE,
        TERMINAL_KEYPOINT_STATE,
        MISSION_COMPLETE_STATE,
        WAITING_STATE
    };
    
    struct param
    {
        geometry_msgs::PoseStamped start;     // 起点；
        geometry_msgs::PoseStamped goal;      // 终点；
        nav_msgs::Path globalKeypoints;       // 存放逆序的关键点，在map坐标系下；
        std_msgs::Int8MultiArray cuspIndexes; // 人字形关键点在globalKeypoints中的索引，逆序；
        bool firstPlanFlag;                   // 是否是初次规划；
        bool replanFlag;                      // 检查A*路径是否发生碰撞、规划时间过长的标志；
        bool sendFlag;                        // 确保初次规划和二次规划时只发送一次起点和终点；
        bool waitingFlag;
        //--void same ptah test--
        bool alpOut;

        STATE_TYPE lastState;
        std::string stringName[7] = {"INITIAL_STATE",
                                     "HYBRID_ASTAR_STATE",
                                     "GENERAL_KEYPOINT_STATE",
                                     "CUSP_KEYPOINT_STATE",
                                     "TERMINAL_KEYPOINT_STATE",
                                     "MISSION_COMPLETE_STATE",
                                     "WAITING_STATE"};
        // bool terminalFlag;                    // 下个关键点是否是终点的标志；
        std_msgs::Bool turningFlag; // 下个关键点是否是人字形关键点的标志；

        tf::TransformListener tfListener;
        tf::StampedTransform transform;
        std_msgs::Float32 predict_time;  // DWA中动态变化的时间；
        nav_msgs::OccupancyGrid gridMap; // 需实时更新的全局地图，map坐标系下；
        bool validStart;                 // 验证起点有效性；
        bool validGoal;                  // 验证终点有效性；
        int targetIndex;                 // 当前跟踪的关键点是第几个关键点；
        float targetDist;
        float tarDisThresh; // 判断是否到达关键点的距离阈值；

        vehicle_msgs::adm_lat finalCmdMsg; // 最终的控制命令由决策系统决定；

        // to do
        std_msgs::Float32 HAStartTime; // 混合A*路径规划所用时间，可选参数；
        bool emergencyStop;            // 感知中的AEB制动；
        bool only_plan_flag;  //是否只用于路径规划；
        // std_msgs::Float32 w_distance;              // 距离代价权重；
        // std_msgs::Float32 w_heading;               // 朝向代价权重；
        // std_msgs::Float32 w_direction;             // 方向代价权重；
    };

    // struct peception_flag
    // {
    //     bool validStart;
    //     bool validGoal;
    // };

}

#endif