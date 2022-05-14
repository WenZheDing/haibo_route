#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <math.h>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vehicle_msgs/adm_lat.h>
#include <map>
#include "weightChange.h"

// using namespace std;

class DwaPlanner
{
public:
    struct position
    {
        float fx;
        float fy;
    };

    struct map_point
    {
        int nx;
        int ny;
    };

    struct state
    {
        float fx;
        float fy;
        float fyaw;
    };

    //匀速运动后的轨迹与代价
    struct trajectory
    {
        float fspeed;
        float fsteer;
        std::vector<state> vsStates;//包含三自由度的向量
        float fDirectionCost;
        float fOscillationCost;
        float fDistanceCost;
        float fHeadingCost;
        bool bCollisionFlag; //会产生碰撞时为1
        float fTotalCost;
        //定义拷贝构造函数
        trajectory operator=(trajectory &tmp)
        {
            fspeed = tmp.fspeed;
            fsteer = tmp.fsteer;
            vsStates.assign(tmp.vsStates.begin(), tmp.vsStates.end());
            fDirectionCost = tmp.fDirectionCost;
            fDistanceCost = tmp.fDistanceCost;
            fOscillationCost = tmp.fOscillationCost;
            fHeadingCost = tmp.fHeadingCost;
            bCollisionFlag = tmp.bCollisionFlag;
            fTotalCost = tmp.fTotalCost;

            return *this;
        }
    };


public:
    DwaPlanner(ros::NodeHandle &node_handle, string &frame_id, bool reverse = 0, bool simulation = 1);
    ~DwaPlanner();
    void StartLocalPlanner();

private:
    /**
    * @brief 定义车辆运动参数
    */

    float PI;
    float MAX_SPEED; // m/s 向前
    float MIN_SPEED; // m/s 向后
    float MAX_ACC;   // m/s*s
    float MIN_ACC;   //  m/s*s

    float MAX_STEER;       // rad 向右
    float MIN_STEER;       // rad 向左
    float MAX_STEER_SPEED; // rad/s
    float MIN_STEER_SPEED; // rad/s
    float MAX_STEER_ACC;   // rad/s*s
    float MIN_STEER_ACC;   // rad/s*s

    float SPEED_RESOLUTION; // m/s
    float STEER_RESOLUTION; // rad

    float dt;           // s
    float PREDICT_TIME; // s

    float VEHICLE_H;       // m 车辆横向长度 5150
    float DWA_VEHICLE_H;       // m 车辆横向长度 8150
    float VEHICLE_V_AXIS;  // m 前后轴轴距 4570
    float VEHICLE_V_FRONT; // m 后轴距离车辆最前方的距离 2965+4570=7535
    float VEHICLE_V_BACK;  // m 后轴距离车辆最后方的距离 3100

    //震荡代价不能超过15%
    float mfDirectionWeight;
    float mfOscillationWeight;
    float mfDistanceWeight;
    float mfHeadingWeight;

    bool turningFlag;
private:

    /**
     * @brief 根据速度输入 计算轨迹以及轨迹的代价
     * @param trial 被计算的采样轨迹 引用传递
     */
    void CalculateTraj(trajectory &tTrial);

    /**
     * @brief 分别计算方向代价 震荡代价 障碍物代价
     * @param trial 被计算的采样轨迹 引用传递
     */
    void getStatesInTraj(trajectory &tTrial);
    void getDirectionCost(trajectory &tTrial);
    void getOscillationCost(trajectory &tTrial);
    void getDistanceCost(trajectory &tTrial);
    void getHeadingCost(trajectory &tTrial);
    void CollisionCheck(trajectory &tTrial);
    bool DistCheck(trajectory &tTrial);

    /**
     * @brief 订阅地图 获取地图基本参数 以及占据情况
     * @brief 订阅终点
     */
    void getMap(const nav_msgs::OccupancyGridConstPtr &map);
    void getTarget(const geometry_msgs::PoseStamped &target);
    void getHeading(const geometry_msgs::PointStamped &heading);
    void resetTime(const std_msgs::Float32 &time);
    void getTurningFlag(const std_msgs::Bool &flag);
    /**
     * @brief 获取坐标(x,y)处的数据
     */
    int getMapData(int x, int y) { return int(map_->data[x + y * map_->info.width]); }

    /**
     * @brief 更新车辆当前速度和采样速度范围 收到速度和方向盘转角数据时调用
     */
    void refreshSampleRange(float fCurSpeed, float fCurSteer);

    /**
     * @brief 将最好的路径转成path发出去
     */
    void pubBestTrajAsPath();
    void pubTrajAsPath(trajectory &tTrial);
    void pubBestTrajCollisionMap();
    void pubBestPolygon(position &A, position &B, position &C, position &D);
    void pubBoundBox(float &up, float &low, float &left, float &right);
    void pubControlCmd();
    void pubSimPosition(float fSimTime);
    void pubVisTraj();

    void prepareForBackward();

    /**
     * @brief ROS相关
     */
    ros::NodeHandle nh;
    string msFrame_id;
    string name_of_node;
    ros::Publisher best_path_pub; //发布最后的最优路径
    ros::Publisher path_pub;
    ros::Publisher collision_map_pub;
    ros::Publisher best_polygon_pub;
    ros::Publisher bound_box_pub;
    ros::Publisher control_cmd_pub;
    ros::Publisher sim_pose_pub;
    ros::Publisher traj_pub;

    ros::Subscriber map_sub;
    ros::Subscriber target_sub;
    ros::Subscriber target_heading_sub;
    ros::Subscriber predict_time_sub;
    ros::Subscriber turning_flag_sub;

    /**
     * @brief 地图信息相关
     */
    nav_msgs::OccupancyGridConstPtr map_;

    float mfResolution;       //地图分辨率
    float mfWidth;            //地图宽度
    float mfHeight;           // 地图高度
    position mpOrigin;        // 地图原点
    position mpTarget;        //目标点
    position mpTargetHeading; //目标方向
    position mpTar_Head_vec;  //目标方向-目标点 得到的向量；
    int mnTargetDirection;    // 行进方向
    int mnLastTargetDirection; // 记录上一次的行进方向，需要为prepare for backward做准备

    /**
     * @brief 车辆信息相关
     */
    geometry_msgs::Polygon mVehiclePolygon;
    float mfCurSpeed;
    float mfCurSteer;
    int mCurControlState;
    int mLastControlState;
    //采样范围
    float mfSampleSpeedMax = MAX_SPEED;
    float mfSampleSpeedMin = MIN_SPEED;
    float mfSampleSteerMax = MAX_STEER;
    float mfSampleSteerMin = MIN_STEER;

    /**
     * @brief 数据结构相关
     */
    vector<trajectory> mvtTrials;
    trajectory mtBestTrajectory;
    trajectory mtLastTrajectory;
    state msInitState;
    float fMaxCost = 0;

    bool mbReverse;
    bool mbSimulation;
    // 用dwa采样路径中的某一个state来当做在当前时刻车辆坐标系下的位，由SimPosePub发出去转换到世界坐标系后作为下一时刻车辆相对全局坐标系的tf
    geometry_msgs::PoseStamped sim_pose;
    float distThresh = 7.5;
    float frontDist = 9.0;
    float backDist = -3.1;
};

#endif
