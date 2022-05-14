#include "DecisionMaker.h"
#include <jsoncpp/json/json.h>
#include <ros/package.h>

namespace Decision
{
    Json::Reader json_reader;
    std::ifstream json_file;
    Json::Value root;
    std::string json_file_path;
    DecisionMaker::DecisionMaker(param *initParam)
    {
        DM_Param = initParam;
        DM_Param->lastState = INITIAL_STATE;

        init_state = new InitState(DM_Param);
        DM_state = init_state;
        HA_state = new HA_PlanningState(DM_Param);
        general_kp_state = new GeneralKeypointState(DM_Param);
        cusp_kp_state = new CuspKeypointState(DM_Param);
        term_kp_state = new TerminalKeypointState(DM_Param);
        complete_state = new MissionCompleteState(DM_Param);
        waiting_state = new WaitingState(DM_Param);

        init_state->NextStates.push_back(HA_state);

        HA_state->NextStates.push_back(general_kp_state);
        HA_state->NextStates.push_back(cusp_kp_state);
        HA_state->NextStates.push_back(waiting_state);
        // HA_state->NextStates.push_back(term_kp_state);

        general_kp_state->NextStates.push_back(cusp_kp_state);
        general_kp_state->NextStates.push_back(term_kp_state);
        general_kp_state->NextStates.push_back(HA_state);
        general_kp_state->NextStates.push_back(waiting_state);

        // cusp_kp_state->NextStates.push_back(term_kp_state);
        cusp_kp_state->NextStates.push_back(HA_state);
        cusp_kp_state->NextStates.push_back(general_kp_state);
        cusp_kp_state->NextStates.push_back(waiting_state);

        term_kp_state->NextStates.push_back(complete_state);
        term_kp_state->NextStates.push_back(HA_state);
        term_kp_state->NextStates.push_back(waiting_state);

        waiting_state->NextStates.push_back(HA_state);
        waiting_state->NextStates.push_back(general_kp_state);
        waiting_state->NextStates.push_back(cusp_kp_state);
        waiting_state->NextStates.push_back(term_kp_state);
        waiting_state->NextStates.push_back(complete_state);

        complete_state->NextStates.push_back(HA_state);

        // to do
        // waiting state;

        // subStart = n.subscribe("/initialpose", 1, &DecisionMaker::setStart, this);
        // subGoal = n.subscribe("/move_base_simple/goal", 1, &DecisionMaker::setGoal, this);
        subStart = n.subscribe("/autostart", 1, &DecisionMaker::setStart, this);
        subGoal = n.subscribe("/autogoal", 1, &DecisionMaker::setGoal, this);
        subMap = n.subscribe("/simu/global_map", 1, &DecisionMaker::setMap, this);
        subPath = n.subscribe("/path_target/keypoints_map", 1, &DecisionMaker::setGlobalKeypoints, this);
        subKeypointIndex = n.subscribe("/path_target/cusp_index", 1, &DecisionMaker::setCuspIndex, this);
        subStartTime = n.subscribe("/hybrid_start_time", 1, &DecisionMaker::setStartTime, this);
        subReplanFlag = n.subscribe("/replan_flag", 1, &DecisionMaker::setReplanFlag, this);
        subDwaConCmd = n.subscribe("/dwa_planner/control_cmd", 1, &DecisionMaker::setControlCmd, this);

        pubProjectionPoint = n.advertise<geometry_msgs::PoseStamped>("/conch/project_point", 1, true);
        pubHandoverInPoint = n.advertise<geometry_msgs::PoseStamped>("/conch/handover_in_point", 1, true);
        pubHandoverOutPoint = n.advertise<geometry_msgs::PoseStamped>("/conch/handover_out_point", 1, true);

        std::string decision_package_path = ros::package::getPath("decision");
        json_file_path = decision_package_path + "/json_files/start_and_goal.json";
        json_file.open(json_file_path);
        if (!json_reader.parse(json_file, root))
        {
            std::cout << "Error opening json file  : " << json_file_path << std::endl;
        }
        strPathName_up = root["strPathName_up"].asString();
        strPathName_down = root["strPathName_down"].asString();
        // setMainRoad(decision_package_path + "/json_files/u2_BB95_xy.csv", decision_package_path + "/json_files/d2_BB95_xy.csv");
        setMainRoad(decision_package_path + "/json_files/" + strPathName_up + "_xy.csv", decision_package_path + "/json_files/" + strPathName_down + "_xy.csv");
        st_gnss_to_global_.fX = 0;
        st_gnss_to_global_.fY = 0;
        st_destination_status_.fx = 0;
        st_destination_status_.fy = 0;
        st_destination_status_.fAngleHeading = 0;
        double latCurrent = root["excavtor_lat"].asFloat();
        double lonCurrent = root["excavtor_lon"].asFloat();
        vGNSSToGlobal(B0, L0, latCurrent, lonCurrent);
        std::cout << "GNSSToGlobal====================:" << st_gnss_to_global_.fX << "," << st_gnss_to_global_.fY << std::endl;
        u2_BB95 = root["u2_BB95"].asString();
        uA_BB95 = root["uA_BB95"].asString();
        uB_BB95 = root["uB_BB95"].asString();
        d2_BB95 = root["d2_BB95"].asString();
        dA_BB95 = root["dA_BB95"].asString();
        dB_BB95 = root["dB_BB95"].asString();

        i_up_entry_index_ = 0;
        i_down_entry_index_ = 0;
        st_rtk_data_.fp_nPointMax = root["Joint_Point_Max"].asInt();
        st_rtk_data_.fp_nHandover_backward_dis = root["handover_backward_dis"].asInt();
        st_rtk_data_.fp_nHandover_backward_diff = root["fp_nHandover_backward_diff"].asInt();
        st_destination_status_.fx = st_gnss_to_global_.fX;
        st_destination_status_.fy = st_gnss_to_global_.fY;
        st_destination_status_.fAngleHeading = root["excavtor_heading"].asFloat();
        only_for_routeplanning = root["only_for_routeplanning"].asBool();
        dist_thresh = root["dist_thresh"].asInt();
        DM_state->setOnlyForRouteplanning(only_for_routeplanning);
        st_stop_point_.x = 0;
        st_stop_point_.y = 0;
        st_stop_point_.heading = 0;
        st_cal_distance.fX = 0;
        st_cal_distance.fY = 0;
        st_plan_to_alp_.i_nearest_index = 0;
        last_x = 0;
        last_y = 0;
        last_yaw = 0;
    }

    DecisionMaker::~DecisionMaker()
    {
        delete init_state;
        delete HA_state;
        delete general_kp_state;
        delete cusp_kp_state;
        delete term_kp_state;
        delete complete_state;
    }

    void DecisionMaker::process()
    {
        // std::cout << "present state is " << DM_state->StateName << std::endl;
        DM_state->pubControlCmd();
        DM_state->pubPerceptionFlag();
        DM_state = DM_state->getNextState();
        DM_state->updateParam();
    }

    void DecisionMaker::setMap(const nav_msgs::OccupancyGrid _map)
    {
        DM_state->setMapParam(_map);
        // ROS_INFO("GOT A NEW MAP!");
    }

    void DecisionMaker::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &startPoint)
    {
        static bool testflag_out = 0;
        geometry_msgs::PoseStamped tmp_point;
        float x = startPoint->pose.pose.position.x / HybridAStar::Constants::cellSize;
        float y = startPoint->pose.pose.position.y / HybridAStar::Constants::cellSize;
        float yaw = tf::getYaw(startPoint->pose.pose.orientation);

        // 泊入起点 --sjtu
        if (!testflag_out && root["json_switch_handle"].asBool())
        {
            ROS_WARN("sjtu --sjtu---sjtu--The starting point calculate");
            float excavtor_x = st_destination_status_.fx;
            float excavtor_y = st_destination_status_.fy;
            float excavtor_yaw = (90 - st_destination_status_.fAngleHeading) * M_PI / 180.0;
            i_up_entry_index_ = iFindEntryIndexNew(strPathName_up, up_road_);
            st_route_point_alpin_ = getHandoverPoint(excavtor_x, excavtor_y, excavtor_yaw, testflag_out);
            x = st_route_point_alpin_.x;
            y = st_route_point_alpin_.y;
            yaw = (90 - st_route_point_alpin_.heading) * M_PI / 180.0;
            ROS_WARN("Got a new start at(stju): (%.2f, %.2f, %.2f)", x, y, st_route_point_alpin_.heading);

            // visualize
            geometry_msgs::PoseStamped st_handover_in_pose;
            st_handover_in_pose.header.frame_id = "/map";
            st_handover_in_pose.header.stamp = ros::Time::now();

            st_handover_in_pose.pose.position.x = x;
            st_handover_in_pose.pose.position.y = y;
            st_handover_in_pose.pose.position.z = 0;
            st_handover_in_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

            pubHandoverInPoint.publish(st_handover_in_pose);
        }

        // 计算泊入起点--haibo
        if (!testflag_out && root["origin_handover_switch"].asBool())
        {
            // if (!root["json_switch_handle"].asBool()) {
            ROS_WARN("haibo ---haibo ---haibo--start point ");
            i_up_entry_index_ = iFindEntryIndexNew(strPathName_up, up_road_);
            vFindPlanHandover(up_road_);
            // x = root["origin_start_x"].asFloat();
            // y = root["origin_start_y"].asFloat();
            // yaw = (90.0 - root["origin_start_heading"].asFloat())*3.1415926/180.0;
            x = st_stop_point_.x;
            y = st_stop_point_.y;
            yaw = (90.0 - st_stop_point_.heading) * 3.1415926 / 180.0;
            ROS_WARN("Got a new start at(stoppoint): (%.2f, %.2f, %.2f)", x, y, st_stop_point_.heading);
            // }
        }

        if (testflag_out && root["origin_handover_switch"].asBool())
        {
            if (root["json_switch_handle"].asBool())
            {
                ROS_ERROR("WARN! if origin_handover_switch AND json_switch_handle are true  belive origin_handover_switch!");
                x = st_gnss_to_global_.fX;
                y = st_gnss_to_global_.fY;
                yaw = root["excavtor_heading"].asFloat();
            }
        }

        ROS_WARN("Got a new start at (%.2f, %.2f, %.2f)", x, y, yaw);
        int map_o_x = DM_Param->gridMap.info.origin.position.x;
        int map_o_y = DM_Param->gridMap.info.origin.position.y;
        float map_res = DM_Param->gridMap.info.resolution;
        int height = DM_Param->gridMap.info.height;
        int width = DM_Param->gridMap.info.width;

        if ((y - map_o_y) / map_res >= 0 && (y - map_o_y) / map_res <= height && (x - map_o_x) / map_res >= 0 && (x - map_o_x) / map_res <= width)
        {
            ROS_INFO("Start point is valid!");
            tmp_point.header.frame_id = "map";
            tmp_point.header.stamp = startPoint->header.stamp;
            // tmp_point.pose.position = startPoint->pose.pose.position;
            // tmp_point.pose.orientation = startPoint->pose.pose.orientation;
            tmp_point.pose.position.x = x * HybridAStar::Constants::cellSize;
            tmp_point.pose.position.y = y * HybridAStar::Constants::cellSize;
            tmp_point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            // std::cout << "start push forward 10 m" <<  std::endl;
            DM_state->setStartParam(tmp_point);
            DM_state->setStartValid(true);
            testflag_out = true;
        }
        else
        {
            ROS_ERROR("Start point is invalid(out of the border)!!!!!");
            DM_state->setStartValid(false);
        }
    }

    void DecisionMaker::setGoal(const geometry_msgs::PoseStamped::ConstPtr &goalPoint)
    {
        static bool testflag_out = 0;
        geometry_msgs::PoseStamped tmp_point;
        float x = goalPoint->pose.position.x / HybridAStar::Constants::cellSize;
        float y = goalPoint->pose.position.y / HybridAStar::Constants::cellSize;
        float yaw = tf::getYaw(goalPoint->pose.orientation);

        //* 判断泊入还是泊出
        if (!testflag_out)
        {
            // 泊入;
            // 判断SJTU还是Conch算法
            if (root["json_switch_handle"].asBool())
            {
                // SJTU算法;
                ROS_WARN("#### SJTU In #### Calculating end point...");
                float dis = root["alp_in_move_distance"].asFloat();
                yaw = (90.0 - root["excavtor_heading"].asFloat()) * M_PI / 180.0;
                x = st_gnss_to_global_.fX + dis * cos(yaw);
                y = st_gnss_to_global_.fY + dis * sin(yaw);
                // std::cout << "alp_in_move_distance =" << dis << " m" << std::endl;
                ROS_WARN("excavtor move ahead 10m position: (%.2f, %.2f, %.2f)", x, y, root["excavtor_heading"].asFloat());
                std_msgs::Bool Alpmsgs;
                Alpmsgs.data = false;
                DM_Param->alpOut = false;
                DM_state->pubAlpMsg(Alpmsgs);
            }
            else{
                // Conch算法;
                ROS_WARN("#### Conch In #### Calculating end point...");
                float dis = root["alp_in_move_distance"].asFloat();
                yaw = (90.0 - root["excavtor_heading"].asFloat()) * 3.1415926 / 180.0;
                x = st_gnss_to_global_.fX + dis * cos(yaw);
                y = st_gnss_to_global_.fY + dis * sin(yaw);
                std::cout << "alp_in_move_distance =" << dis << " m" << std::endl;
                ROS_WARN("excavtor move ahead 10m position: (%.2f, %.2f, %.2f)", x, y, root["excavtor_heading"].asFloat());
                std_msgs::Bool Alpmsgs;
                Alpmsgs.data = false;
                DM_Param->alpOut = false;
                DM_state->pubAlpMsg(Alpmsgs);
            }
        }
        else{
            // 泊出
            // 判断SJTU还是Conch算法
            if (root["json_switch_handle"].asBool())
            {
                //  SJTU算法
                ROS_WARN("#### SJTU OUT #### Calculating end point...");
                float excavtor_x = st_destination_status_.fx;
                float excavtor_y = st_destination_status_.fy;
                float excavtor_yaw = (90 - st_destination_status_.fAngleHeading) * 3.1415926 / 180.0;
                i_down_entry_index_ = iFindEntryIndexNew(strPathName_down, down_road_);
                st_route_point_alpout_ = getHandoverPoint(excavtor_x, excavtor_y, excavtor_yaw, testflag_out);
                x = st_route_point_alpout_.x;
                y = st_route_point_alpout_.y;
                yaw = (90 - st_route_point_alpout_.heading) * 3.1415926 / 180.0;
                ROS_WARN("sjtu alp out: (%.2f, %.2f, %.2f)", x, y, st_route_point_alpout_.heading);
                std_msgs::Bool Alpmsgs;
                Alpmsgs.data = false;
                DM_Param->alpOut = false;
                DM_state->pubAlpMsg(Alpmsgs);

                // visualize
                geometry_msgs::PoseStamped st_handover_out_pose;
                st_handover_out_pose.header.frame_id = "/map";
                st_handover_out_pose.header.stamp = ros::Time::now();

                st_handover_out_pose.pose.position.x = x;
                st_handover_out_pose.pose.position.y = y;
                st_handover_out_pose.pose.position.z = 0;
                st_handover_out_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

                pubHandoverOutPoint.publish(st_handover_out_pose);
            }
            else{
                // Conch算法;
                for (int i = 0; i < 5; i++)
                    ROS_WARN("#### Conch OUT #### Calculating end point...");
                i_down_entry_index_ = iFindEntryIndexNew(strPathName_down, down_road_);
                int alp_to_plan_index = iFindAlptoRtkHandover(down_road_);
                x = down_road_.at(alp_to_plan_index).x;
                y = down_road_.at(alp_to_plan_index).y;
                yaw = (90 - down_road_.at(alp_to_plan_index).heading) * 3.1415926 / 180.0;
                std_msgs::Bool Alpmsgs;
                Alpmsgs.data = false;
                DM_Param->alpOut = false;
                DM_state->pubAlpMsg(Alpmsgs);
            }

            if (only_for_routeplanning)
            {
                std_msgs::Bool Alpmsgs;
                Alpmsgs.data = true;
                DM_Param->alpOut = true;
                DM_state->pubAlpMsg(Alpmsgs);
                ROS_INFO("alpout start point is alpin goal!");
                tmp_point.header.frame_id = "map";
                tmp_point.header.stamp = goalPoint->header.stamp;
                tmp_point.pose.position.x = last_x * HybridAStar::Constants::cellSize;
                tmp_point.pose.position.y = last_y * HybridAStar::Constants::cellSize;
                tmp_point.pose.orientation = tf::createQuaternionMsgFromYaw(last_yaw);
                DM_state->setStartParam(tmp_point);
                DM_state->setStartValid(true);
            }
        }

        ROS_WARN("Got a new goal at (%.2f, %.2f, %.2f)", x, y, yaw);
        int map_o_x = DM_Param->gridMap.info.origin.position.x;
        int map_o_y = DM_Param->gridMap.info.origin.position.y;
        float map_res = DM_Param->gridMap.info.resolution;
        int height = DM_Param->gridMap.info.height;
        int width = DM_Param->gridMap.info.width;
        std::cout << "map_o_x:  " << map_o_x << "   map_o_y:   " << map_o_y << "   map_res:   " << map_res << std::endl;
        std::cout << "y: " << (y - map_o_y) / map_res << std::endl;
        std::cout << "height: " << height << std::endl;
        std::cout << "x: " << (x - map_o_x) / map_res << std::endl;
        std::cout << "width: " << width << std::endl;
        //  --real-map-test--
        if ((y - map_o_y) / map_res >= 0 && (y - map_o_y) / map_res <= height && (x - map_o_x) / map_res >= 0 && (x - map_o_x) / map_res <= width)
        {
            ROS_INFO("Goal point is valid!");
            tmp_point.header.frame_id = "map";
            tmp_point.header.stamp = goalPoint->header.stamp;
            // tmp_point.pose.position = goalPoint->pose.position;
            // tmp_point.pose.orientation = goalPoint->pose.orientation;
            tmp_point.pose.position.x = x * HybridAStar::Constants::cellSize;
            tmp_point.pose.position.y = y * HybridAStar::Constants::cellSize;
            // rcx_edit
            tmp_point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            tmp_point.pose.position.z = 0.f;
            if (!DM_Param->firstPlanFlag)
            {
                // 不是初次规划；
                ROS_INFO("Goal point has changed!");
                DM_state->setReplanParam(true);
                testflag_out = true;
                // DM_state->resetParam();
            }
            else
            {
                // 是初次规划；
                ROS_INFO("This is first planning!");
                DM_Param->firstPlanFlag = false;
                testflag_out = true;
            }
            last_x = x;
            last_y = y;
            last_yaw = yaw;
            DM_state->setSendFlag(true);
            DM_state->setGoalParam(tmp_point);
            DM_state->setGoalValid(true);
        }
        else
        {
            ROS_ERROR("Goal point is invalid(out of the border)!!!!!");
            DM_state->setGoalValid(false);
        }
    }

    void DecisionMaker::setGlobalKeypoints(const nav_msgs::Path globalKeypoints_)
    {
        DM_state->setKeypointsParam(globalKeypoints_);

        // 收到路径的第一时间就判断下个关键点是否是人字形关键点；
        int length1 = DM_Param->globalKeypoints.poses.size();
        int length2 = DM_Param->cuspIndexes.data.size();
        if (length1 >= 3 && length2 >= 1)
        {
            for (int i = 1; i < length2; i++)
            {
                if (DM_Param->cuspIndexes.data[i] == length1 - 1 - 1)
                {
                    DM_Param->turningFlag.data = true;
                    return;
                }
            }
        }
        //
        else if (length1 == 1)
        {
            // 没考虑交接点发生碰撞的情形，请确保交接点不撞，否则是交接点选取的问题；
            DM_Param->waitingFlag = true;
        }
    }

    void DecisionMaker::setCuspIndex(const std_msgs::Int8MultiArray cuspIndex)
    {
        DM_state->setCuspParam(cuspIndex);

        int length1 = DM_Param->globalKeypoints.poses.size();
        int length2 = DM_Param->cuspIndexes.data.size();
        if (length1 >= 3 && length2 >= 1)
        {
            for (int i = 1; i < length2; i++)
            {
                if (DM_Param->cuspIndexes.data[i] == length1 - 1 - 1)
                {
                    DM_Param->turningFlag.data = true;
                    return;
                }
            }
        }
    }

    void DecisionMaker::setStartTime(const std_msgs::Float32 time)
    {
        DM_state->setStartTimeParam(time);
    }

    void DecisionMaker::setReplanFlag(const std_msgs::Bool flag)
    {
        DM_state->setReplanParam(flag.data);
    }

    void DecisionMaker::setControlCmd(const vehicle_msgs::adm_lat controlCmd)
    {
        DM_state->setFinalCmdMsg(controlCmd);
    }

    void DecisionMaker::setMainRoad(std::string up_path, std::string down_path)
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
            // std::cout<< " line.heading before   " <<  line.heading << std::endl;
            /*
            if (line.heading > 90 && line.heading < 270) {
                line.heading = -(line.heading-90)/180*3.14;
                // std::cout<< " line.heading after   " <<  line.heading << std::endl;
            } else if (line.heading <=90) {
                line.heading = (90 - line.heading)/180*3.14;
            } else if (line.heading >= 270) {
                line.heading = (360-line.heading+90)/180*3.14;
            }
            */
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
            /*
          if (line.heading > 90 && line.heading < 270) {
              line.heading = -(line.heading-90)/180*3.14;
              // std::cout<< " line.heading after   " <<  line.heading << std::endl;
          } else if (line.heading <=90) {
              line.heading = (90 - line.heading)/180*3.14;
          } else if (line.heading >= 270) {
              line.heading = (360-line.heading+90)/180*3.14;
          }
          */
            down_road_.push_back(line);
            fields.clear();
        }
        if (!up_road_.size() || !down_road_.size())
        {
            ROS_ERROR("main road infomation fail to get");
            std::cout << "up road size:" << up_road_.size() << std::endl;
            std::cout << "down road size:" << down_road_.size() << std::endl;
        }
    }

    RoutePoint DecisionMaker::getHandoverPoint(float x, float y, float yaw, bool alp_out)
    {
        // std::map<int, float> Hashmap;
        static RoutePoint handover_in;
        // find handoverOUT
        float dist_min = INFINITY;
        std::vector<Decision::RoutePoint>::iterator pos;

        // find projection_point
        if (alp_out)
        {
            // 下山主路最近点
            pos = down_road_.begin();
            std::cout << "i_down_entry_index_: " << i_down_entry_index_ << std::endl;

            for (auto iter = down_road_.begin(); iter != down_road_.begin() + i_down_entry_index_; iter++)
            { 
                float dist = sqrt((iter->x - x) * (iter->x - x) + (iter->y - y) * (iter->y - y));
                if (dist < dist_min)
                {
                    dist_min = dist;
                    pos = iter;
                }
            }
        }
        else{
            // 上山主路最近点
            pos = up_road_.begin();
            std::cout << "i_up_entry_index_::" << i_up_entry_index_ << std::endl;

            for (auto iter = up_road_.begin() + i_up_entry_index_; iter != up_road_.end(); iter++)
            {
                float dist = sqrt((iter->x - x) * (iter->x - x) + (iter->y - y) * (iter->y - y));
                // Hashmap.insert(std::make_pair(i,dist));
                if (dist < dist_min)
                {
                    dist_min = dist;
                    pos = iter;
                }
            }
        }
        std::cout << "dist_min: " << dist_min << std::endl;
        RoutePoint projection_point = {pos->x, pos->y, (90 - pos->heading) * M_PI / 180.0}; //! 最近点的坐标,注意heading的变化
        std::cout << "projection_point: " << projection_point.x << ", " << projection_point.y << ", " << projection_point.heading << std::endl;
        
        // Visualize
        geometry_msgs::PoseStamped st_project_pose;
        st_project_pose.header.frame_id = "/map";
        st_project_pose.header.stamp = ros::Time::now();

        st_project_pose.pose.position.x = projection_point.x; 
        st_project_pose.pose.position.y = projection_point.y; 
        st_project_pose.pose.position.z = 0; 
        st_project_pose.pose.orientation = tf::createQuaternionMsgFromYaw(projection_point.heading);

        pubProjectionPoint.publish(st_project_pose);

        float f_move_distance;
        float f_angle_diff = projection_point.heading - yaw;
        float f_project_dist = dist_min;
        if (alp_out)
        {
            f_move_distance = root["out_base_distance"].asInt() + root["out_distance_radius"].asInt() * (1 + root["out_heading_weight"].asFloat() * cos(f_angle_diff)) * 
                            (root["out_distance_weight"].asInt() * 4.0 / M_PI * atan2(dist_min / 200, 1)); // atan2(y, x) return (-pi, pi]
            std::cout << "[Down road]: f_move_distance: " << f_move_distance << std::endl;

            float dist = 0;
            float f_angle_change = 0;
            for (auto iter = pos; (iter + 1) < down_road_.begin() + i_down_entry_index_; iter++)
            {
                dist += sqrt(((iter + 1)->x - iter->x) * ((iter + 1)->x - iter->x) + ((iter + 1)->y - iter->y) * ((iter + 1)->y - iter->y));
                f_angle_change = fabs(pos->heading - iter->heading);
                if (dist > f_move_distance || f_angle_change >= 25)
                {
                    if ((iter + 1) > (down_road_.begin() + i_down_entry_index_))
                    {
                        break;
                    }
                    std::cout << "alp out handover point: " << iter->x << ", " << iter->y << ", " << iter->heading << std::endl;
                    return *(iter + 1);
                }
            }
            ROS_INFO("<---------------reach the end of the down main-road--------------->");

            return *(down_road_.begin() + i_down_entry_index_);
        }
        else{
            f_move_distance = root["in_base_distance"].asInt() + root["in_distance_radius"].asInt() * (1 + root["in_heading_weight"].asFloat() * cos(f_angle_diff)) * 
                            (root["in_distance_weight"].asInt() * 4.0 / M_PI * atan2(dist_min / 200, 1)); // atan2(y, x) return (-pi, pi]
            std::cout << "[Up road]: f_move_distance: " << f_move_distance << std::endl;

            float dist = 0;
            float f_angle_change = 0;
            int i_iter_step = cos(f_angle_diff) >= 0 ? 1 : -1;
            for (auto iter = pos; iter  != up_road_.begin() + i_up_entry_index_; )
            {
                dist += sqrt(((iter - 1)->x - iter->x) * ((iter - 1)->x - iter->x) + ((iter - 1)->y - iter->y) * ((iter - 1)->y - iter->y));
                f_angle_change = fabs(pos->heading - iter->heading);
                if (dist > f_move_distance || f_angle_change >= 15)
                {
                    pos = iter - 1;
                    return *pos;
                }
                iter = iter + i_iter_step;
            }
            if (pos == up_road_.begin() + i_up_entry_index_) return *pos;

            ROS_INFO("<---------------cant find handover point in down road--------------->");

            RoutePoint back_point = {pos->x, pos->y, pos->heading};
            std::cout << "back_point: " << back_point.x << ", " << back_point.y << ", " << back_point.heading << std::endl;

            RoutePoint back_point_vector = {projection_point.x - back_point.x, projection_point.y - back_point.y, back_point.heading};
            std::pair<float, float> unit_vector(cos(yaw), sin(yaw));
            float vector_weight = root["vector_weight"].asFloat();
            float offset = back_point_vector.x * vector_weight * unit_vector.first + back_point_vector.y * vector_weight * unit_vector.second;
            std::cout << "offset:" << offset << std::endl;
            // find handover_point
            dist = 0;
            for (auto iter = pos;; offset > 0 ? iter++ : iter--)
            {
                //dist = sqrt((iter->x -  back_point.x)*(iter->x -  back_point.x) + (iter->y -  back_point.y) * (iter->y -  back_point.y));
                if (offset >= 0)
                {
                    dist += sqrt(((iter + 1)->x - iter->x) * ((iter + 1)->x - iter->x) + ((iter + 1)->y - iter->y) * ((iter + 1)->y - iter->y));
                    if (dist > abs(offset))
                    {
                        pos = iter;
                        break;
                    }
                }
                else
                {
                    dist += sqrt(((iter - 1)->x - iter->x) * ((iter - 1)->x - iter->x) + ((iter - 1)->y - iter->y) * ((iter - 1)->y - iter->y));
                    if (dist > abs(offset))
                    {
                        pos = iter;
                        break;
                    }
                }
            }
            handover_in = *pos;
            std::cout << "handover_in     " << handover_in.x << "," << handover_in.y << "," << handover_in.heading << std::endl;
            return handover_in;
        }
    }

    float DecisionMaker::normalizeHeadingRad(float t)
    {
        if (t < 0)
        {
            t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
            return 2.f * M_PI + t;
        }

        return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    }

    int DecisionMaker::iFindNearestPoint(RP_tstDestinationStatus &destination_position, std::vector<RoutePoint> waypoint)
    {
        float delta_x = 0;
        float delta_y = 0;
        float dis_to_excavtor_min = 10000;
        int nearest_index = 0;
        if (waypoint.size() != 0)
        {
            for (int i = 0; i < waypoint.size(); i++)
            {
                delta_x = destination_position.fx - waypoint.at(i).x;
                delta_y = destination_position.fy - waypoint.at(i).y;

                float dist_to_excavtor_temp = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));

                if (dist_to_excavtor_temp < dis_to_excavtor_min)
                {
                    dis_to_excavtor_min = dist_to_excavtor_temp;
                    nearest_index = i; // equal nearest_ids
                }
            }
        }
        else
        {
            std::cout << "输入路线为空" << std::endl;
        }
        return nearest_index;
    }

    int DecisionMaker::iFindEntryIndexNew(std::string strPathName, std::vector<RoutePoint> waypoint)
    {
        int cur_entry_index = 0;
        RP_tstDestinationStatus st_up_entry_point_;
        // fileName = strPathName + "_xy.csv";
        std::cout << "current_PathName=" << strPathName << std::endl;
        std::cout << "waypoint.size=" << waypoint.size() << std::endl;

        if (strPathName.compare(u2_BB95) == 0)
        {
            st_up_entry_point_.fx = 60;
            st_up_entry_point_.fy = 200;
            cur_entry_index = iFindNearestPoint(st_up_entry_point_, waypoint);
            std::cout << "u2_BB95.cur_entry_index=" << cur_entry_index << std::endl;
        }
        else if (strPathName.compare(uA_BB95) == 0)
        {
            st_up_entry_point_.fx = 60;
            st_up_entry_point_.fy = 200;
            cur_entry_index = iFindNearestPoint(st_up_entry_point_, waypoint);
            std::cout << "uA_BB95.cur_entry_index=" << cur_entry_index << std::endl;
        }
        else if (strPathName.compare(uB_BB95) == 0)
        {
            st_up_entry_point_.fx = 60;
            st_up_entry_point_.fy = 200;
            cur_entry_index = iFindNearestPoint(st_up_entry_point_, waypoint);
            std::cout << "uB_BB95.cur_entry_index=" << cur_entry_index << std::endl;
            // } else if (strPathName.compare(d2_BB95) == 0) {
        }
        else if (strPathName == "d2_BB95")
        {
            st_up_entry_point_.fx = 60;
            st_up_entry_point_.fy = 200;
            cur_entry_index = iFindNearestPoint(st_up_entry_point_, waypoint);
            std::cout << "d2_BB95.cur_entry_index=" << cur_entry_index << std::endl;
        }
        else if (strPathName.compare(dA_BB95) == 0)
        {
            st_up_entry_point_.fx = 60;
            st_up_entry_point_.fy = 200;
            cur_entry_index = iFindNearestPoint(st_up_entry_point_, waypoint);
            std::cout << "dA_BB95.cur_entry_index=" << cur_entry_index << std::endl;
        }
        else if (strPathName.compare(dB_BB95) == 0)
        {
            // st_up_entry_point_.fx = 60;
            // st_up_entry_point_.fy = 200;
            st_up_entry_point_.fx = 80;
            st_up_entry_point_.fy = 206;
            cur_entry_index = iFindNearestPoint(st_up_entry_point_, waypoint);
            std::cout << "dB_BB95.cur_entry_index=" << cur_entry_index << std::endl;
        }
        else
        {
            std::cout << "其他平台" << std::endl;
        }
        std::cout << "平台入口点=" << cur_entry_index << std::endl;
        return cur_entry_index;
    }

    void DecisionMaker::vFindPlanHandover(std::vector<RoutePoint> waypoint)
    {

        std::cout << "挖机位置update: " << st_destination_status_.fx << " ;" << st_destination_status_.fy << ";" << st_destination_status_.fAngleHeading << std::endl;
        std::cout << "路线长度waypoint[0].size(): " << waypoint.size() << std::endl;
        bool bo_TransPoint = false; // 是否找到交接点
        int tohandover_index_temp = 0;
        float dist_to_ex_temp = 0;
        float dist_to_exh_temp = 0;
        float dist_to_ex_min = 10000;   // 挖机与路径点最短距离初始值
        float dist_to_exh_min = 1000.0; // 挖机指向与路径点的距离初始值
        float dist_handover = 0.0;
        /*找到路径上与挖机位置最近的点*/
        // std::cout << "i_up_entry_index_:" << i_up_entry_index_ << std::endl;
        if (i_up_entry_index_ > waypoint.size())
        {
            ROS_ERROR("平台入口点计算错误");
            return;
        }
        for (int i = i_up_entry_index_; i < waypoint.size(); i++)
        {
            st_cal_distance.fX = st_destination_status_.fx - waypoint.at(i).x;
            st_cal_distance.fY = st_destination_status_.fy - waypoint.at(i).y;

            dist_to_ex_temp = std::sqrt(std::pow(st_cal_distance.fX, 2) + std::pow(st_cal_distance.fY, 2));
            if (dist_to_ex_temp < dist_to_ex_min)
            {
                dist_to_ex_min = dist_to_ex_temp;
                st_plan_to_alp_.i_nearest_index = i; // equal nearest_ids
                // std::cout << "i= " << i << "; dist_to_ex_min=" << dist_to_ex_min <<std::endl;
            }
        }
        std::cout << "最近点Index(planning to dwa): " << st_plan_to_alp_.i_nearest_index << std::endl;
        std::cout << "挖机定位点到主路的距离:" << dist_to_ex_min << std::endl;
        /*根据最短距离，选择DWA模式*/
        // if (dist_to_ex_min <= 41.0) {
        //     i8_alp_mode_ = DWA_ParkingMode_A;
        // } else {
        //     i8_alp_mode_ = DWA_ParkingMode_B;
        // }

        /*寻找交接点*/
        if ((dist_to_ex_min >= 20) && (dist_to_ex_min < st_rtk_data_.fp_nPointMax))
        {
            for (int d = std::max(20, static_cast<int>(dist_to_ex_min)); d < st_rtk_data_.fp_nPointMax; d++)
            {
                float x_hat = d * sin(st_destination_status_.fAngleHeading / 180 * M_PI);
                float y_hat = d * cos(st_destination_status_.fAngleHeading / 180 * M_PI);
                int min_nearest = std::max(i_up_entry_index_, st_plan_to_alp_.i_nearest_index - 1000);
                int max_nearest = std::min(st_plan_to_alp_.i_nearest_index + 1000, static_cast<int>(waypoint.size()));
                // log_logger_.LogInfo() << "min_nearest=991?:max_nearest=1982=" << min_nearest << " ," << max_nearest;
                for (int j = min_nearest; j < max_nearest; j++)
                {
                    // vGNSSToGlobal(st_destination_status_.dPosLat ,
                    //  st_destination_status_.dPosLon , waypoint[1][j] , waypoint[2][j]);
                    st_cal_distance.fX = waypoint.at(j).x - st_destination_status_.fx;
                    st_cal_distance.fY = waypoint.at(j).y - st_destination_status_.fy;
                    dist_to_exh_temp = sqrt(pow((st_cal_distance.fX - x_hat), 2) + pow((st_cal_distance.fY - y_hat), 2));

                    if (dist_to_exh_temp < dist_to_exh_min)
                    {
                        dist_to_exh_min = dist_to_exh_temp;
                        st_plan_to_alp_.i_plan_to_alp_index = j;
                    }
                }
                // log_logger_.LogInfo() << "dist_to_exh_min:" << dist_to_exh_min;
                if (dist_to_exh_min <= 2.5)
                {
                    bo_TransPoint = true;
                    break;
                }
            }
        }
        else
        {
            bo_TransPoint = false;
            st_plan_to_alp_.i_plan_to_alp_index = st_plan_to_alp_.i_nearest_index;
        }
        if (bo_TransPoint == true)
        {
            std::cout << "挖机朝向上存在与主路的交点P,交点的Index: " << st_plan_to_alp_.i_plan_to_alp_index
                      << " 挖机的朝向角为:" << st_destination_status_.fAngleHeading << std::endl;
            if (st_plan_to_alp_.i_plan_to_alp_index < st_plan_to_alp_.i_nearest_index)
            {
                tohandover_index_temp = st_plan_to_alp_.i_plan_to_alp_index;
                std::cout << "交点P在最近点E的左侧,基于交点P后退50m" << std::endl;
            }
            else
            {
                tohandover_index_temp = st_plan_to_alp_.i_nearest_index;
                std::cout << "交点P在最近点E的右侧,基于最近点E后退50m" << std::endl;
            }
        }
        else
        {
            tohandover_index_temp = st_plan_to_alp_.i_nearest_index; // 使用距离最近点计算终点
            std::cout << "找不到交接点,基于最近点E后退50m" << std::endl;
        }
        for (int i = tohandover_index_temp; i > i_up_entry_index_; i--)
        {
            if (i >= 1)
            {
                // vGNSSToGlobal(waypoint[1][i-1], waypoint[2][i-1], waypoint[1][i], waypoint[2][i]);
                // 循环计算实际交接点dao挖机朝向交点的距离
                st_cal_distance.fX = waypoint.at(i - 1).x - waypoint.at(i).x;
                st_cal_distance.fY = waypoint.at(i - 1).y - waypoint.at(i).y;
                dist_handover += sqrt(pow(st_cal_distance.fX, 2) + pow(st_cal_distance.fY, 2));
                if (dist_handover >= st_rtk_data_.fp_nHandover_backward_dis)
                {
                    st_plan_to_alp_.i_last_handover_index = i - 1; // 计算钝角的虚拟点
                    break;
                }
            }
            else
            {
                /* code */
            }
        }
        // 未找找到50m的点，取最近点
        if (dist_handover < st_rtk_data_.fp_nHandover_backward_dis)
        {
            st_plan_to_alp_.i_last_handover_index = i_up_entry_index_;
        }

        std::cout << "最终的交接点为" << st_plan_to_alp_.i_last_handover_index << std::endl;
        st_plan_to_alp_.i_plan_to_alp_index = st_plan_to_alp_.i_last_handover_index;
        st_plan_to_alp_.f_plan_to_alp_position_x = waypoint.at(st_plan_to_alp_.i_plan_to_alp_index).x;
        st_plan_to_alp_.f_plan_to_alp_position_y = waypoint.at(st_plan_to_alp_.i_plan_to_alp_index).y;
        st_plan_to_alp_.f_nearest_position_x = waypoint.at(st_plan_to_alp_.i_nearest_index).x;
        st_plan_to_alp_.f_nearest_position_y = waypoint.at(st_plan_to_alp_.i_nearest_index).y;
        st_plan_to_alp_.f_last_handover_position_x = waypoint.at(st_plan_to_alp_.i_last_handover_index).x;
        st_plan_to_alp_.f_last_handover_position_y = waypoint.at(st_plan_to_alp_.i_last_handover_index).y;
        st_stop_point_.x = st_plan_to_alp_.f_last_handover_position_x;
        st_stop_point_.y = st_plan_to_alp_.f_last_handover_position_y;
        st_stop_point_.heading = waypoint.at(st_plan_to_alp_.i_last_handover_index).heading;
        // alp_in_start_point_.x = st_plan_to_alp_.f_last_handover_position_x;
        // alp_in_start_point_.y = st_plan_to_alp_.f_last_handover_position_y;
        // alp_in_start_point_.heading = waypoint.at(st_plan_to_alp_.i_last_handover_index).heading;
        std::cout << "st_stop_point_.x" << st_stop_point_.x << " ;" << st_stop_point_.y << ";" << st_stop_point_.heading << std::endl;
    }

    void DecisionMaker::vGNSSToGlobal(double B0, double L0, double latCurrent, double lonCurrent)
    {

        double B = latCurrent * M_PI / 180, L = lonCurrent * M_PI / 180;
        double B00 = B0 * M_PI / 180, L00 = L0 * M_PI / 180;
        double a = 6378137, b = 6356752.3142, e = sqrt(1 - pow(b / a, 2)), ee = sqrt(pow(a / b, 2) - 1);
        double K = (pow(a, 2) * cos(B00) / b) / sqrt(1 + pow(ee, 2) * pow(cos(B00), 2));
        st_gnss_to_global_.fX = K * (L - L00);
        double Y0 = K * log(tan(M_PI / 4 + B00 / 2) * pow((1 - e * sin(B00)) / (1 + e * sin(B00)), e / 2));
        st_gnss_to_global_.fY = K * log(tan(M_PI / 4 + B / 2) * pow((1 - e * sin(B)) / (1 + e * sin(B)), e / 2)) - Y0;
        return;
    }

    int DecisionMaker::iFindAlptoRtkHandover(std::vector<RoutePoint> waypoint)
    {
        // st_rtk_data_ = *local_view.rtk_data;
        float f_dist_alp_to_plan_temp = 0;
        float f_dist_alp_to_plan_min = 10000.0;
        float f_dist_handover = 0.0;
        float f_dist_to_excavator_temp = 0;
        int i_handover_index_temp = 0;
        float f_dist_to_excavator_min = 1000.0; // 挖机指向与路径点的距离初始值
        bool bo_trans_point = false;            // 是否找到交接点
        int i_nearest_index_down = 0;
        int i_alp_to_plan_index = 0;
        float ex_heading = 0;
        float destination_point_x = waypoint.front().x;
        float destination_point_y = waypoint.front().y;
        float destination_point_heading = waypoint.front().heading;
        //计算与主路最近的点
        //临时解决最近点会找到主路上面，只遍历一半的路径
        // std::string v = vector_roadlist[0];
        // log_logger_.LogInfo() << " v=:" << v << ",waypoint:" << waypoint.size();
        // int i_down_entry_index_ = iFindEntryIndexNew(v , waypoint);
        std::cout << " 下山平台入口点:" << i_down_entry_index_ << std::endl;
        if ((i_down_entry_index_ >= waypoint.size() - 1) || (i_down_entry_index_ == 0))
        {
            i_down_entry_index_ = (waypoint.size() / 2);
            std::cout << " 平台入口的index超过路径最大index或未找到平台入口，选择路径=index/2" << std::endl;
            //设置故障状态
            // vSetFault(4 , "can't find platform entry point,use half of hold size as entry point index");
            // st_data_overflow_.bo_find_entrypoint_fail = true;
        }
        else
        {
            //清除故障状态
            // st_data_overflow_.bo_find_entrypoint_fail = false;
        }
        // destination_point_x = st_rtk_data_.excavator_position.x;
        // destination_point_y = st_rtk_data_.excavator_position.y;
        // destination_point_heading = st_rtk_data_.excavator_position.Heading;
        destination_point_x = st_destination_status_.fx;
        destination_point_y = st_destination_status_.fy;
        destination_point_heading = st_destination_status_.fAngleHeading;

        std::cout << "iFindAlptoRtkHandover--挖机位置update: " << destination_point_x << ", " << destination_point_y << "," << destination_point_heading << std::endl;
        if (destination_point_x > 5000 || destination_point_y > 5000)
        {
            std::cout << "iFindAlptoRtkHandover--挖机位置update: " << destination_point_x << ", " << destination_point_y << std::endl;
        }
        for (int i = 0; i < (i_down_entry_index_); i++)
        {
            // vGNSSToGlobal(st_destination_status_.dPosLat,
            //  st_destination_status_.dPosLon, waypoint[1][i], waypoint[2][i]);
            float delat_fX1 = waypoint.at(i).x - destination_point_x;
            float delat_fy1 = waypoint.at(i).y - destination_point_y;
            float f_dist_alp_to_plan_temp = sqrt(pow(delat_fX1, 2) + pow(delat_fy1, 2));
            if (f_dist_alp_to_plan_temp < f_dist_alp_to_plan_min)
            {
                f_dist_alp_to_plan_min = f_dist_alp_to_plan_temp;
                i_nearest_index_down = i;
            }
        }
        std::cout << "the nearest point of dwa to plan:" << i_nearest_index_down << std::endl;
        // if ((st_rtk_data_.excavator_type == 3) || (st_rtk_data_.excavator_type == 7)) {

        //   ex_heading = destination_point_heading  - 90;

        // } else {
        //   ex_heading = destination_point_heading;
        // }
        //按照航向角寻找交接点
        if ((f_dist_alp_to_plan_min >= 20) && (f_dist_alp_to_plan_min < st_rtk_data_.fp_nPointMax))
        {
            for (int d = std::max(20, static_cast<int>(f_dist_alp_to_plan_min)); d < st_rtk_data_.fp_nPointMax; d++)
            {
                float x_hat = d * sin(destination_point_heading / 180 * M_PI); // 挖机为坐标原点，行向角指向的坐标X
                float y_hat = d * cos(destination_point_heading / 180 * M_PI); // 挖机为坐标原点，行向角指向的坐标Y

                for (int j = 0; j < (i_down_entry_index_); j++)
                {
                    // vGNSSToGlobal(st_destination_status_.dPosLat,
                    //  st_destination_status_.dPosLon, waypoint[1][j], waypoint[2][j]);
                    float delat_fX2 = waypoint.at(j).x - destination_point_x;
                    float delat_fy2 = waypoint.at(j).y - destination_point_y;
                    f_dist_to_excavator_temp = std::sqrt(std::pow((delat_fX2 - x_hat), 2) + std::pow((delat_fy2 - y_hat), 2));
                    if (f_dist_to_excavator_temp < f_dist_to_excavator_min)
                    {
                        f_dist_to_excavator_min = f_dist_to_excavator_temp;
                        i_alp_to_plan_index = j;
                    }
                }
                if (f_dist_to_excavator_min <= 2.5)
                {
                    bo_trans_point = true;
                    break;
                }
            }
        }
        else
        {
            bo_trans_point = false;
            i_alp_to_plan_index = i_nearest_index_down;
        }
        if (bo_trans_point == true)
        {
            if (i_alp_to_plan_index >= i_nearest_index_down)
            { // 交点的Index>最近点的Index
                i_handover_index_temp = i_alp_to_plan_index;
                std::cout << "下山交点的Index > 最近点的Index" << std::endl;
            }
            else
            { // 交点的Index<最近点的Index,交点取最近点
                i_handover_index_temp = i_nearest_index_down;
                std::cout << "下山交点的Index < 最近点的Index" << std::endl;
            }
            std::cout << "挖机朝向上存在与主路的交点P,交点的Index: " << i_alp_to_plan_index << " ,挖机的朝向角为:" << destination_point_heading << std::endl;
        }
        else
        {
            i_handover_index_temp = i_nearest_index_down; // 使用距离最近点计算终点
            std::cout << "找不到交接点,基于最近点E前进80m作为交接点" << std::endl;
        }
        if (i_handover_index_temp < i_down_entry_index_)
        {
            for (int i = (i_handover_index_temp); i < (i_down_entry_index_); i++)
            {
                //累计从交接点至当前点的距离
                // vGNSSToGlobal(waypoint[1][i+1], waypoint[2][i+1], waypoint[1][i], waypoint[2][i]);
                float delat_fX3 = waypoint.at(i + 1).x - waypoint.at(i).x;
                float delat_fy3 = waypoint.at(i + 1).y - waypoint.at(i).y;
                f_dist_handover += std::sqrt(std::pow((delat_fX3), 2) + std::pow((delat_fy3), 2));
                if (f_dist_handover > (st_rtk_data_.fp_nHandover_backward_dis + st_rtk_data_.fp_nHandover_backward_diff))
                {
                    i_alp_to_plan_index = i + 1; // 计算下山交接点
                    std::cout << "a向前推进距离:" << f_dist_handover << std::endl;
                    break;
                }
            }
        }
        else
        {
            // 下山前进距离小于80，交接点为入口点
            i_alp_to_plan_index = i_down_entry_index_;
            std::cout << "交接点Index大于平台入口点Index,交接点选为入口点:" << i_alp_to_plan_index << std::endl;
        }
        //  如果交接点到平台出口的距离小于80m，选择平台出口为交接点
        if (f_dist_handover <= (st_rtk_data_.fp_nHandover_backward_dis + st_rtk_data_.fp_nHandover_backward_diff))
        {
            i_alp_to_plan_index = i_down_entry_index_;
            std::cout << "b向前推进距离:" << f_dist_handover << std::endl;
            std::cout << "如果推进距离超过平台入口点，选平台入口点作为交接点：:" << i_alp_to_plan_index << std::endl;
        }
        std::cout << "下山最终的交接点(基于交点前进80m)为Index=" << i_alp_to_plan_index << std::endl;
        std::cout << "下山最终交接点坐标:" << waypoint.at(i_alp_to_plan_index).x << " ," << waypoint.at(i_alp_to_plan_index).y << " ," << waypoint.at(i_alp_to_plan_index).heading << std::endl;
        // i_handover_index_temp = i_alp_to_plan_index;

        return i_alp_to_plan_index;
    }
}