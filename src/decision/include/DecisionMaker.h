#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H

#include <algorithm>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include "state.h"
#include "../../hybrid_astar/include/hybrid_astar/constants.h"
#include "map"
#include "data_type.h"
// #include "hybrid_astar/helper.h"

namespace Decision
{
    class DecisionMaker
    {
    public:
        DecisionMaker(param *initParam);
        ~DecisionMaker();

        void process();
        // void updateParam();

    public:
        void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &startPoint);
        void setGoal(const geometry_msgs::PoseStamped::ConstPtr &goalPoint);
        void setMap(const nav_msgs::OccupancyGrid _map);
        void setStartTime(const std_msgs::Float32 time);
        void setReplanFlag(const std_msgs::Bool flag);
        void setGlobalKeypoints(const nav_msgs::Path globalKeypoints_);
        void setCuspIndex(const std_msgs::Int8MultiArray cuspIndex);
        void setControlCmd(const vehicle_msgs::adm_lat controlCmd);
        float normalizeHeadingRad(float t);
        int iFindNearestPoint(RP_tstDestinationStatus & destination_position , std::vector<RoutePoint> waypoint);
        int iFindEntryIndexNew(std::string strPathName , std::vector<RoutePoint> waypoint);
        void vFindPlanHandover(std::vector<RoutePoint>waypoint);
        void vGNSSToGlobal(double B0, double L0, double latCurrent, double lonCurrent);
        int iFindAlptoRtkHandover(std::vector<RoutePoint> waypoint);

    private:
        param *DM_Param;
        State *DM_state;
        InitState *init_state;
        HA_PlanningState *HA_state;
        GeneralKeypointState *general_kp_state;
        CuspKeypointState *cusp_kp_state;
        TerminalKeypointState *term_kp_state;
        MissionCompleteState *complete_state;
        WaitingState *waiting_state;
        
        std::vector<RoutePoint> up_road_;
        std::vector<RoutePoint> down_road_;
        std::string strPathName_up, strPathName_down;
        std::string u2_BB95, uA_BB95, uB_BB95, d2_BB95, dA_BB95, dB_BB95;
        int i_up_entry_index_, i_down_entry_index_;
        float last_x, last_y, last_yaw;
        bool only_for_routeplanning;
        int dist_thresh;

        RP_tstCalDistance st_cal_distance , st_gnss_to_global_;
        RP_tstDestinationStatus st_destination_status_;
        RP_tstPlanToAlp st_plan_to_alp_;
        RTKData st_rtk_data_;
        StopPoint st_stop_point_;
        RoutePoint st_route_point_alpin_, st_route_point_alpout_;
        double B0 = 31.1367846;
        double L0 = 118.1789369;

        ros::NodeHandle n;
        ros::Subscriber subStart;
        ros::Subscriber subGoal;
        ros::Subscriber subMap;
        ros::Subscriber subPath;
        ros::Subscriber subKeypointIndex;
        ros::Subscriber subStartTime;
        ros::Subscriber subReplanFlag;
        ros::Subscriber subDwaConCmd;

        ros::Publisher pubProjectionPoint;
        ros::Publisher pubHandoverInPoint;
        ros::Publisher pubHandoverOutPoint;

        void setMainRoad(std::string, std::string);
        RoutePoint getHandoverPoint(float, float, float, bool);
    };
}
#endif