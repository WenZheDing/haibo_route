#ifndef STATE_H
#define STATE_H

#include "param.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <ros/ros.h>

namespace Decision
{
    class State
    {
    public:
        State(param *initParam);
        // State(std::shared_ptr<param> initParam);
        virtual ~State(){ delete mParam; }

        virtual State *getNextState() = 0;
        virtual void pubControlCmd() = 0;
        virtual void pubPerceptionFlag() = 0;
        virtual void updateParam() = 0;

        // 在可到达的状态中根据名字返回目标状态指针;
        State *findState(STATE_TYPE name)
        {
            mParam->lastState = this->StateName;
            for (int i = 0; i < NextStates.size(); i++)
            {
                State *tmp_state = NextStates[i];
                if (tmp_state && name == tmp_state->StateName)
                {
                    if (tmp_state == 0)
                    {
                        return this;
                    }    

                    return tmp_state;
                }
            }
            ROS_INFO("Can not find this STATE!!!");
            return nullptr;
        }

        std::vector<State *> NextStates;
        STATE_TYPE StateName;

    public:
        param *getParam() {return mParam;}
        void resetParam();
        void dwa_static();

        void setStartParam(geometry_msgs::PoseStamped startPoint) { mParam->start = startPoint; }
        void setGoalParam(geometry_msgs::PoseStamped goalPoint) { mParam->goal = goalPoint; }
        void setStartValid(bool validFlag) { mParam->validStart = validFlag; }
        void setGoalValid(bool validFlag) { mParam->validGoal = validFlag; }
        void setMapParam(nav_msgs::OccupancyGrid map) { mParam->gridMap = map; }
        void setSendFlag(bool flag) { mParam->sendFlag = flag; }
        void setStartTimeParam(std_msgs::Float32 time) { mParam->HAStartTime = time; }
        void setReplanParam(bool flag) { mParam->replanFlag = flag; }
        void setKeypointsParam(nav_msgs::Path Keypoints) { mParam->globalKeypoints = Keypoints; }
        void setCuspParam(std_msgs::Int8MultiArray KeypointIndex_) { mParam->cuspIndexes = KeypointIndex_; }
        void setTargetIndex(int index) { mParam->targetIndex = index; }
        void setTarDisThresh(float distance) { mParam->tarDisThresh = distance; }
        void setFinalCmdMsg(vehicle_msgs::adm_lat controlCmd) { mParam->finalCmdMsg; }
        void setOnlyForRouteplanning(bool flag) { mParam->only_plan_flag = flag; }

        //void setALPOutFlag(bool flag) { mParam->alpOut = flag; }
        
        // void setTerminalFlag(bool flag) { mParam->terminalFlag = flag; }
        // void setPathParam(nav_msgs::Path globalPath_) { mParam->global_path_ = globalPath_; }

        void pubStart(){ startPub.publish(mParam->start);}
        void pubGoal(){ goalPub.publish(mParam->goal);}
        void pubPredictTime(){ predictTimePub.publish(mParam->predict_time); }
        void pubTurningFlag(){ turningFlagPub.publish(mParam->turningFlag); }
        void pubFinalControl(){ controlCmdPub.publish(mParam->finalCmdMsg); }
        void pubKeypoint(const geometry_msgs::PoseStamped &keypoint){ keypointPub.publish(keypoint); }
        void pubTargeHeading(const geometry_msgs::PointStamped &heading) { targetHeadingPub.publish(heading); }
        void pubFinalCmdMsg(const vehicle_msgs::adm_lat &cmdMsg) { finalCmdMsgPub.publish(cmdMsg); }
        void pubAlpMsg(const std_msgs::Bool ALPOut) { AlpPub.publish(ALPOut); }
        
    private:
        param *mParam;
        ros::NodeHandle nh;
        ros::Publisher startPub;
        ros::Publisher goalPub;
        ros::Publisher predictTimePub;
        ros::Publisher turningFlagPub;
        ros::Publisher controlCmdPub;
        ros::Publisher keypointPub;
        ros::Publisher targetHeadingPub;
        ros::Publisher finalCmdMsgPub;
        ros::Publisher AlpPub;
    };

    // 交接点待命状态；
    class InitState : public State
    {
    public:
        InitState(param *initParam)
            : State(initParam) { StateName = INITIAL_STATE; }
        virtual ~InitState() {};
        virtual State *getNextState();
        virtual void pubControlCmd();
        virtual void pubPerceptionFlag();
        virtual void updateParam();
    };

    // 混合A*路径规划状态；
    class HA_PlanningState : public State
    {
    public:
        HA_PlanningState(param *initParam)
            : State(initParam) { StateName = HYBRID_ASTAR_STATE; }
        virtual ~HA_PlanningState() {};
        virtual State *getNextState(); 
        virtual void pubControlCmd();
        virtual void pubPerceptionFlag();
        virtual void updateParam();
    };

    // DWA跟踪普通关键点状态;
    class GeneralKeypointState : public State
    {
    public:
        GeneralKeypointState(param *initParam)
            : State(initParam) { StateName = GENERAL_KEYPOINT_STATE; }
        virtual ~GeneralKeypointState() {};
        virtual State *getNextState();
        virtual void pubControlCmd();
        virtual void pubPerceptionFlag();
        virtual void updateParam();
    };

    // DWA跟踪人字形关键点状态；
    class CuspKeypointState : public State
    {
    public:
        CuspKeypointState(param *initParam)
            : State(initParam) { StateName = CUSP_KEYPOINT_STATE; }
        virtual ~CuspKeypointState() {};
        virtual State *getNextState();
        virtual void pubControlCmd();
        virtual void pubPerceptionFlag();
        virtual void updateParam();
    };

    // DWA跟踪终点关键点状态；
    class TerminalKeypointState : public State
    {
    public:
        TerminalKeypointState(param *initParam)
            : State(initParam) { StateName = TERMINAL_KEYPOINT_STATE; }
        virtual ~TerminalKeypointState() {};
        virtual State *getNextState();
        virtual void pubControlCmd();
        virtual void pubPerceptionFlag();
        virtual void updateParam();
    };

    // 到达终点状态；
    class MissionCompleteState : public State
    {
    public:
        MissionCompleteState(param *initParam)
            : State(initParam) { StateName = MISSION_COMPLETE_STATE; }
        virtual ~MissionCompleteState() {};
        virtual State *getNextState();
        virtual void pubControlCmd();
        virtual void pubPerceptionFlag();
        virtual void updateParam();
    };

    // 中途待命状态；
    class WaitingState : public State
    {
    public:
        WaitingState(param *initParam)
            : State(initParam) { StateName = WAITING_STATE;}
        virtual ~WaitingState() {};
        virtual State *getNextState();
        virtual void pubControlCmd();
        virtual void pubPerceptionFlag();
        virtual void updateParam();
    };
}
#endif