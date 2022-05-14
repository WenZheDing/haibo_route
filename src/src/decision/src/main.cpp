#include "DecisionMaker.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "decision_maker");

    ros::Time::init();
    ros::Rate loop_rate(10);
    ROS_INFO("111111111111111111111111111");
    Decision::param initParam;
    // initParam->sendFlag = false;
    // ROS_INFO("111111111111111111111111112");
    // initParam->firstPlanFlag = true;
    // initParam->replanFlag = false;
    // initParam->validStart = false;
    // initParam->validGoal = false;
    // initParam->tarDisThresh = 3.0;
    // initParam->targetIndex = 1;
    // initParam->turningFlag.data = false;
    // initParam->cuspIndexes.data.clear();
    // initParam = new Decision::param;
    Decision::DecisionMaker D_Maker(&initParam);
    while (ros::ok())
    {
        ros::spinOnce();
        // ROS_INFO("111111111111111111111111111");
        D_Maker.process();
        loop_rate.sleep();
    }
    return 0;
}