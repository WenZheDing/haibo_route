#include "state.h"

namespace Decision
{
    State::State(param *initParam)
    {
        if (!initParam)
        {
            ROS_ERROR("initParam is empty!");
            mParam = new param;
        }
        else
            mParam = initParam;

        // 每个状态下都能保持当前的状态；
        NextStates.push_back(this);

        mParam->sendFlag = false;
        mParam->firstPlanFlag = true;
        mParam->replanFlag = false;
        mParam->validStart = false;
        mParam->validGoal = false;
        mParam->tarDisThresh = 3.0;
        mParam->targetIndex = 1;
        mParam->turningFlag.data = false;
        mParam->waitingFlag = false;
        // mParam->terminalFlag = false;

        mParam->alpOut = false;

        mParam->cuspIndexes.data.clear();
        mParam->only_plan_flag = false;

        startPub = nh.advertise<geometry_msgs::PoseStamped>("/hybrid_astar_start", 1, true);
        goalPub = nh.advertise<geometry_msgs::PoseStamped>("/hybrid_astar_goal", 1, true);
        predictTimePub = nh.advertise<std_msgs::Float32>("/dwa_predict_time", 1, true);
        turningFlagPub = nh.advertise<std_msgs::Bool>("/dwa_turning_flag", 1, true);
        controlCmdPub = nh.advertise<vehicle_msgs::adm_lat>("/final_cmd_msg", 1, true);
        keypointPub = nh.advertise<geometry_msgs::PoseStamped>("/dwa_keypoint", 1, true);
        targetHeadingPub = nh.advertise<geometry_msgs::PointStamped>("/dwa_target_heading", 1, true);
        finalCmdMsgPub = nh.advertise<vehicle_msgs::adm_lat>("/final_control_cmd", 1, true);
        AlpPub = nh.advertise<std_msgs::Bool>("/ALPOut", 1, true);
    }

    void State::resetParam()
    {
        // 更新起点；
        try
        {
            getParam()->tfListener.lookupTransform("/map", "/base", ros::Time(0), getParam()->transform);
        }
        catch (tf::TransformException &ex)
        {
            /* code for Catch */
            ROS_INFO("state 44!!!");
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        geometry_msgs::PoseStamped new_start;
        new_start.header.frame_id = "map";
        new_start.header.stamp = ros::Time::now();
        new_start.pose.position.x = getParam()->transform.getOrigin().x();
        new_start.pose.position.y = getParam()->transform.getOrigin().y();
        float yaw;
        // yaw = (tf::getYaw(getParam()->transform.getRotation()) + M_PI_2) * 180.f / M_PI;
        yaw = tf::getYaw(getParam()->transform.getRotation()) + M_PI_2;
        new_start.pose.orientation = tf::createQuaternionMsgFromYaw(yaw); // ?单位？？
        setStartParam(new_start);
        setStartValid(true);

        getParam()->globalKeypoints.poses.clear();
        getParam()->cuspIndexes.data.clear();
        setReplanParam(false);
        setSendFlag(true);
        // setTerminalFlag(false);
        getParam()->turningFlag.data = false;
        getParam()->waitingFlag = false;
        setTargetIndex(1);
        setTarDisThresh(3.0);
    }

    void State::dwa_static()
    {
        geometry_msgs::PoseStamped localPose;
        localPose.header.frame_id = "base";
        localPose.header.stamp = ros::Time::now();
        localPose.pose.position.x = 0;
        localPose.pose.position.y = 0;
        localPose.pose.position.z = 0;
        localPose.pose.orientation = getParam()->goal.pose.orientation;
        pubKeypoint(localPose);

        geometry_msgs::PointStamped targetHeading;
        targetHeading.header.frame_id = "base";
        targetHeading.header.stamp = ros::Time::now();
        targetHeading.point.x = 0 + 1 * cos(tf::getYaw(localPose.pose.orientation));
        targetHeading.point.y = 0 + 1 * sin(tf::getYaw(localPose.pose.orientation));
        pubTargeHeading(targetHeading);
    }

    /* -------------------------------------------------------------------------- */
    /*                                  InitState                                 */
    /* -------------------------------------------------------------------------- */
    State *InitState::getNextState()
    {
        // std::cout << "present state is " << this->StateName << std::endl;
        if (getParam()->validStart && getParam()->validGoal)
        {
            std::cout << "InitState -----> HybridAstarState" << std::endl;
            setSendFlag(true);
            return findState(HYBRID_ASTAR_STATE);
        }
        else
        {
            // std::cout << "InitState -----> InitState" << std::endl;
            return this;
        }
        // std::cout << "present state2 is " << this->StateName << std::endl;
    }

    void InitState::pubControlCmd()
    {
        // 初始时发布静止的控制指令；
    }

    void InitState::pubPerceptionFlag()
    {
        //
    }

    void InitState::updateParam()
    {
        // std::cout << "present state1 is " << this->StateName << std::endl;
    }

    /* -------------------------------------------------------------------------- */
    /*                              HA_PlanningState                              */
    /* -------------------------------------------------------------------------- */
    State *HA_PlanningState::getNextState()
    {
        if (getParam()->only_plan_flag)
        {
            // std::cout << "HybridAstarState -----> HybridAstarState" << std::endl;
            return this;
        }
        else if (getParam()->waitingFlag)
        {
            std::cout << "HybridAstarState -----> WaitingState" << std::endl;
            return findState(WAITING_STATE);
        }
        else if (getParam()->globalKeypoints.poses.size() >= 3)
        {
            if (getParam()->turningFlag.data)
            {
                std::cout << "HybridAstarState -----> CuspKeypointState" << std::endl;
                return findState(CUSP_KEYPOINT_STATE);
            }
            else
            {
                // std::cout << "HybridAstarState -----> GeneralKeypointState" << std::endl;
                return findState(GENERAL_KEYPOINT_STATE);
            }
        }
        else
        {
            // std::cout << "HybridAstarState -----> HybridAstarState" << std::endl;
            return this;
        }
        // else if ()
    }

    void HA_PlanningState::pubPerceptionFlag()
    {
        // 发布有效的起点和终点；
        // ROS_INFO("validStart is %i, validGoal is %i, sendFlag is %i", getParam()->validStart, getParam()->validGoal, getParam()->sendFlag);
        if (getParam()->validStart && getParam()->validGoal && getParam()->sendFlag)
        {
            ROS_INFO("Sending valid start and goal point......");
            pubStart();
            pubGoal();
            // pubAstarPlanFlag(true); // true ,进入dwa状态时，须改为false;
            setSendFlag(false);
        }

        dwa_static();
    }

    void HA_PlanningState::pubControlCmd()
    {
        // 初始时发布静止的控制指令；
    }

    void HA_PlanningState::updateParam()
    {
        // setReplanParam(false);
        /* // 超出60s预测时间则准备进入重规划状态；
        float duration = ros::Time::now().sec - getParam()->HAStartTime.data.sec;
        // 记得replan时重置开始时间；
        if (duration > 60)
        {
            // dwa做出一些行为；
            setReplanParam(true);
            resetParam();
        } */
    }

    /* -------------------------------------------------------------------------- */
    /*                            GeneralKeypointState                            */
    /* -------------------------------------------------------------------------- */
    State *GeneralKeypointState::getNextState()
    {
        if (getParam()->waitingFlag)
        {
            std::cout << "GeneralKeypointState -----> WaitingState" << std::endl;
            return findState(WAITING_STATE);
        }
        else if (getParam()->replanFlag) // !!!
        {
            std::cout << "GeneralKeypointState -----> HybridAstarState" << std::endl;
            resetParam();
            return findState(HYBRID_ASTAR_STATE);
        }
        else if (getParam()->targetDist <= getParam()->tarDisThresh)
        {
            // ROS_INFO("ARRIVING AT TARGET POINT! TURNING FLAG IS %i", getParam()->turningFlag.data);
            if (getParam()->turningFlag.data)
            {
                std::cout << "GeneralKeypointState -----> CuspKeypointState" << std::endl;
                return findState(CUSP_KEYPOINT_STATE);
            }
            else if (getParam()->targetIndex == getParam()->globalKeypoints.poses.size() - 1)
            {
                std::cout << "GeneralKeypointState -----> TerminalKeypointState" << std::endl;
                return findState(TERMINAL_KEYPOINT_STATE);
            }
            else
            {
                // std::cout << "GeneralKeypointState -----> GeneralKeypointState" << std::endl;
                return this;
            }
        }
        else
        {
            // ROS_INFO("targetDist is %.2f, tarDisThresh is %.2f", getParam()->targetDist, getParam()->tarDisThresh);
            // std::cout << "GeneralKeypointState -----> GeneralKeypointState" << std::endl;
            return this;
        }
    }

    void GeneralKeypointState::pubPerceptionFlag()
    {
        if (true)
        {
            pubPredictTime();
        }
    }

    void GeneralKeypointState::pubControlCmd()
    {
    }

    void GeneralKeypointState::updateParam()
    {
        getParam()->tarDisThresh = 2.0;
        int length = getParam()->globalKeypoints.poses.size();
        int index = getParam()->targetIndex;
        geometry_msgs::PoseStamped nextKeypoint, lastKeypoint;
        geometry_msgs::PointStamped targetHeading;
        // 验证路径有效性；
        if (length >= 3)
        {
            try
            {
                getParam()->tfListener.lookupTransform("/map", "/base", ros::Time(0), getParam()->transform);
            }
            catch (tf::TransformException &ex)
            {
                /* code for Catch */
                ROS_INFO("state 124!!!");
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            float base_x_map = getParam()->transform.getOrigin().x();
            float base_y_map = getParam()->transform.getOrigin().y();
            // float base_t_map = (tf::getYaw(getParam()->transform.getRotation()) + M_PI_2) * 180.f / M_PI;
            nextKeypoint = getParam()->globalKeypoints.poses[length - index - 1];
            nextKeypoint.header.frame_id = "map";
            nextKeypoint.header.stamp = ros::Time(0);

            geometry_msgs::PoseStamped localPose;
            try
            {
                getParam()->tfListener.transformPose("base", ros::Time(0), nextKeypoint, "map", localPose);
            }
            catch (tf::TransformException &ex)
            {
                ROS_INFO("state 167!!!");
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            pubKeypoint(localPose);

            targetHeading.header.frame_id = "base";
            targetHeading.header.stamp = ros::Time::now();
            targetHeading.point.x = localPose.pose.position.x + 1 * cos(tf::getYaw(localPose.pose.orientation));
            targetHeading.point.y = localPose.pose.position.y + 1 * sin(tf::getYaw(localPose.pose.orientation));
            pubTargeHeading(targetHeading);

            lastKeypoint = getParam()->globalKeypoints.poses[length - index];
            float segDist = sqrt(pow(nextKeypoint.pose.position.x - lastKeypoint.pose.position.x, 2) + pow(nextKeypoint.pose.position.y - lastKeypoint.pose.position.y, 2)); //map
            getParam()->targetDist = sqrt(pow(nextKeypoint.pose.position.x - base_x_map, 2) + pow(nextKeypoint.pose.position.y - base_y_map, 2));
            // 根据与下个关键点的距离确定DWA的预测时间；
            getParam()->predict_time.data = 5.0 * (1 - (1 - 1 / 5.0) * pow(std::min(abs(1 - getParam()->targetDist / segDist), 1.0f), 1));

            if (getParam()->targetDist <= getParam()->tarDisThresh)
            {
                getParam()->targetIndex += 1;
                index += 1;

                // 实时判断下个关键点是否是人字形关键点；
                for (int i = 1; i < getParam()->cuspIndexes.data.size(); i++)
                {
                    ROS_INFO("CUSP INDEX IS %i, length - index -1 is %i, length is %i, index is %i",
                             getParam()->cuspIndexes.data[i], length - index - 1, length, index);
                    if (getParam()->cuspIndexes.data[i] == length - index - 1)
                    {
                        getParam()->turningFlag.data = true;
                        // getParam()->tarDisThresh = 1.5;
                        return;
                    }
                }
                getParam()->turningFlag.data = false;
            }
        }
        else
        {
            ROS_ERROR("Keypoints is valid!!!");
            return;
        }
    }

    /* -------------------------------------------------------------------------- */
    /*                              CuspKeypointState                             */
    /* -------------------------------------------------------------------------- */
    State *CuspKeypointState::getNextState()
    {
        if (getParam()->waitingFlag)
        {
            std::cout << "CuspKeypointState -----> WaitingState" << std::endl;
            return findState(WAITING_STATE);
        }
        else if (getParam()->replanFlag) // !!!
        {
            std::cout << "CuspKeypointState -----> HybridAstarState" << std::endl;
            resetParam();
            return findState(HYBRID_ASTAR_STATE);
        }
        else if (getParam()->targetDist <= getParam()->tarDisThresh)
        {
            if (getParam()->turningFlag.data)
            {
                // std::cout << "CuspKeypointState -----> CuspKeypointState" << std::endl;
                return this;
            }
            else
            {
                std::cout << "CuspKeypointState -----> GeneralKeypointState" << std::endl;
                return findState(GENERAL_KEYPOINT_STATE);
            }
        }
        else
        {
            // std::cout << "CuspKeypointState -----> CuspKeypointState" << std::endl;
            return this;
        }
    }

    void CuspKeypointState::pubPerceptionFlag()
    {
        pubTurningFlag();
    }

    void CuspKeypointState::pubControlCmd()
    {
    }

    void CuspKeypointState::updateParam()
    {
        getParam()->tarDisThresh = 1.5;
        int length = getParam()->globalKeypoints.poses.size();
        int index = getParam()->targetIndex;
        geometry_msgs::PoseStamped nextKeypoint, lastKeypoint;
        geometry_msgs::PointStamped targetHeading;
        // 验证路径有效性；
        if (length >= 3)
        {
            try
            {
                getParam()->tfListener.lookupTransform("/map", "/base", ros::Time(0), getParam()->transform);
            }
            catch (tf::TransformException &ex)
            {
                /* code for Catch */
                ROS_INFO("state 275!!!");
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            float base_x_map = getParam()->transform.getOrigin().x();
            float base_y_map = getParam()->transform.getOrigin().y();
            // float base_t_map = (tf::getYaw(getParam()->transform.getRotation()) + M_PI_2) * 180.f / M_PI;
            nextKeypoint = getParam()->globalKeypoints.poses[length - index - 1];
            nextKeypoint.header.frame_id = "map";
            nextKeypoint.header.stamp = ros::Time(0);

            geometry_msgs::PoseStamped localPose;
            try
            {
                getParam()->tfListener.transformPose("base", ros::Time(0), nextKeypoint, "map", localPose);
            }
            catch (tf::TransformException &ex)
            {
                ROS_INFO("state 293!!!");
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            pubKeypoint(localPose);

            targetHeading.header.frame_id = "base";
            targetHeading.header.stamp = ros::Time::now();
            targetHeading.point.x = localPose.pose.position.x + 1 * cos(tf::getYaw(localPose.pose.orientation));
            targetHeading.point.y = localPose.pose.position.y + 1 * sin(tf::getYaw(localPose.pose.orientation));
            pubTargeHeading(targetHeading);

            lastKeypoint = getParam()->globalKeypoints.poses[length - index];
            float segDist = sqrt(pow(nextKeypoint.pose.position.x - lastKeypoint.pose.position.x, 2) + pow(nextKeypoint.pose.position.y - lastKeypoint.pose.position.y, 2));
            getParam()->targetDist = sqrt(pow(nextKeypoint.pose.position.x - base_x_map, 2) + pow(nextKeypoint.pose.position.y - base_y_map, 2));
            // 根据与下个关键点的距离确定DWA的预测时间；
            getParam()->predict_time.data = 3.5 * (1 - pow(std::min(abs(1 - getParam()->targetDist / segDist), 1.0f), 2));

            if (getParam()->targetDist <= getParam()->tarDisThresh)
            {
                getParam()->targetIndex += 1;
                index += 1;

                // 实时判断下个关键点是否是人字形关键点；
                for (int i = 1; i < getParam()->cuspIndexes.data.size(); i++)
                {
                    if (getParam()->cuspIndexes.data[i] == length - index - 1)
                    {
                        getParam()->turningFlag.data = true;
                        return;
                    }
                }
                getParam()->turningFlag.data = false;
            }
        }
        else
        {
            ROS_ERROR("Keypoints is valid!!!");
            return;
        }
    }

    /* -------------------------------------------------------------------------- */
    /*                            TerminalKeypointState                           */
    /* -------------------------------------------------------------------------- */
    State *TerminalKeypointState::getNextState()
    {
        if (getParam()->waitingFlag)
        {
            std::cout << "TerminalKeypointState -----> WaitingState" << std::endl;
            return findState(WAITING_STATE);
        }
        else if (getParam()->replanFlag) // !!!
        {
            std::cout << "TerminalKeypointState -----> HybridAstarState" << std::endl;
            resetParam();
            return findState(HYBRID_ASTAR_STATE);
        }
        else if (getParam()->targetDist <= getParam()->tarDisThresh)
        {
            std::cout << "TerminalKeypointState -----> MissionCompleteState" << std::endl;
            return findState(MISSION_COMPLETE_STATE);
        }
        else
        {
            // std::cout << "TerminalKeypointState -----> TerminalKeypointState" << std::endl;
            return this;
        }
    }

    void TerminalKeypointState::pubPerceptionFlag()
    {
    }

    void TerminalKeypointState::pubControlCmd()
    {
    }

    void TerminalKeypointState::updateParam()
    {
        getParam()->tarDisThresh = 1;
        int length = getParam()->globalKeypoints.poses.size();
        int index = getParam()->targetIndex;
        geometry_msgs::PoseStamped nextKeypoint, lastKeypoint;
        geometry_msgs::PointStamped targetHeading;
        // 验证路径有效性；
        if (length >= 3)
        {
            try
            {
                getParam()->tfListener.lookupTransform("/map", "/base", ros::Time(0), getParam()->transform);
            }
            catch (tf::TransformException &ex)
            {
                /* code for Catch */
                ROS_INFO("state 275!!!");
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            float base_x_map = getParam()->transform.getOrigin().x();
            float base_y_map = getParam()->transform.getOrigin().y();
            // float base_t_map = (tf::getYaw(getParam()->transform.getRotation()) + M_PI_2) * 180.f / M_PI;
            nextKeypoint = getParam()->globalKeypoints.poses[length - index - 1];
            nextKeypoint.header.frame_id = "map";
            nextKeypoint.header.stamp = ros::Time(0);

            geometry_msgs::PoseStamped localPose;
            try
            {
                getParam()->tfListener.transformPose("base", ros::Time(0), nextKeypoint, "map", localPose);
            }
            catch (tf::TransformException &ex)
            {
                ROS_INFO("state 293!!!");
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            pubKeypoint(localPose);

            targetHeading.header.frame_id = "base";
            targetHeading.header.stamp = ros::Time::now();
            targetHeading.point.x = localPose.pose.position.x + 1 * cos(tf::getYaw(localPose.pose.orientation));
            targetHeading.point.y = localPose.pose.position.y + 1 * sin(tf::getYaw(localPose.pose.orientation));
            pubTargeHeading(targetHeading);

            lastKeypoint = getParam()->globalKeypoints.poses[length - index];
            float segDist = sqrt(pow(nextKeypoint.pose.position.x - lastKeypoint.pose.position.x, 2) + pow(nextKeypoint.pose.position.y - lastKeypoint.pose.position.y, 2));
            getParam()->targetDist = sqrt(pow(nextKeypoint.pose.position.x - base_x_map, 2) + pow(nextKeypoint.pose.position.y - base_y_map, 2));
            // 根据与下个关键点的距离确定DWA的预测时间；
            getParam()->predict_time.data = 5.0 * (1 - 1 / 3.5) * (1 - pow(std::min(abs(1 - getParam()->targetDist / segDist), 1.0f), 2));

            if (getParam()->targetDist <= getParam()->tarDisThresh)
            {
                getParam()->targetIndex += 1;
                index += 1;

                getParam()->turningFlag.data = false;
            }
            // getParam()->tarDisThresh = 0.5;
        }
        else
        {
            ROS_ERROR("Keypoints is valid!!!");
            return;
        }
    }

    /* -------------------------------------------------------------------------- */
    /*                            MissionCompleteState                            */
    /* -------------------------------------------------------------------------- */
    State *MissionCompleteState::getNextState()
    {
        if (getParam()->replanFlag) // !!!
        {
            // std::cout << "MissionCompleteState -----> HybridAstarState" << std::endl;
            resetParam();

            std_msgs::Bool Alpmsgs;
            Alpmsgs.data = true;
            getParam()->alpOut = true;
            State::pubAlpMsg(Alpmsgs);

            return findState(HYBRID_ASTAR_STATE);
        }
        else
        {
            // std::cout << "MissionCompleteState -----> MissionCompleteState" << std::endl;
            return this;
        }
    }

    void MissionCompleteState::pubPerceptionFlag()
    {
        dwa_static();
    }

    void MissionCompleteState::pubControlCmd()
    {
    }

    void MissionCompleteState::updateParam()
    {
        // 包装成一个函数 dwa_static();

        // ROS_INFO("Mission Complete!!!!!");
    }

    /* -------------------------------------------------------------------------- */
    /*                                WaitingState                                */
    /* -------------------------------------------------------------------------- */
    State *WaitingState::getNextState()
    {
        if (getParam()->replanFlag)
        {
            // std::cout << "WaitingState -----> HybridAstarState" << std::endl;
            std::cout << "WaitingState -----> " << getParam()->stringName[getParam()->lastState] << std::endl;
            resetParam();
            return findState(getParam()->lastState);
        }
        else if (!getParam()->waitingFlag)
        {
            std::cout << "WaitingState -----> " << getParam()->stringName[getParam()->lastState] << std::endl;
            return findState(getParam()->lastState);
        }
        else
        {
            std::cout << "WaitingState -----> WaitingState" << std::endl;
            return this;
        }
    }

    void WaitingState::pubPerceptionFlag()
    {
        dwa_static();
    }

    void WaitingState::pubControlCmd()
    {
    }

    void WaitingState::updateParam()
    {
    }
}