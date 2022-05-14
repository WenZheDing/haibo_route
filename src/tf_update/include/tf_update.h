#ifndef TF_UPDATE_H
#define TF_UPDATE_H

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class TfUpdater
{
public:
    TfUpdater(ros::NodeHandle &nh_, std::string &frame_id_);
    ~TfUpdater(){}

private:
    void initTf(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initPoint);
    void updateTf(const geometry_msgs::PoseStamped &simPose);
    void updateVtdTf(const geometry_msgs::PoseStamped &vtdPose);

    ros::NodeHandle nh;
    ros::Subscriber initSub;
    ros::Subscriber dwaSimSub;
    ros::Subscriber vtdSub;

    std::string frameId; 
    tf::TransformBroadcaster tfBroadcaster;
    tf::TransformListener tfListener;
    tf::StampedTransform tfInitPose;
    tf::Quaternion tfCounterclockwiseRot;
};
#endif