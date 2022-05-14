#include "tf_update.h"

TfUpdater::TfUpdater(ros::NodeHandle &nh_, std::string &frame_id_)
{
    nh = nh_;
    frameId = frame_id_;

    tfCounterclockwiseRot.setRotation(tf::Vector3(0, 0, 1), -M_PI_2);

    initSub = nh.subscribe("/initialpose", 1, &TfUpdater::initTf, this);
    dwaSimSub = nh.subscribe("/dwa_planner/sim_position", 1, &TfUpdater::updateTf, this);
    vtdSub = nh.subscribe("/vtd_global_pose", 1, &TfUpdater::updateVtdTf, this);
}

void TfUpdater::initTf(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initPoint)
{
    tf::Quaternion initQuad;
    tf::quaternionMsgToTF(initPoint->pose.pose.orientation, initQuad);

    tf::Transform trans = tf::Transform(initQuad * tfCounterclockwiseRot,
                                        tf::Vector3(initPoint->pose.pose.position.x,
                                                    initPoint->pose.pose.position.y,
                                                    initPoint->pose.pose.position.z));
    tfInitPose = tf::StampedTransform(trans, ros::Time::now(), "/map", frameId); // frame id: /base
    tfBroadcaster.sendTransform(tfInitPose);
}

void TfUpdater::updateTf(const geometry_msgs::PoseStamped &simPose)
{
    geometry_msgs::PoseStamped globalSimPose;
    try
    {
        tfListener.transformPose("map", ros::Time(0), simPose, frameId, globalSimPose);
    }
    catch (tf::TransformException &ex)
    {
        /* code for Catch */
        ROS_INFO("tf_update 37!!!");
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    tf::Quaternion globalQuad;

    tf::quaternionMsgToTF(globalSimPose.pose.orientation, globalQuad);
    tf::Transform trans = tf::Transform(globalQuad * tfCounterclockwiseRot,
                                        tf::Vector3(globalSimPose.pose.position.x,
                                                    globalSimPose.pose.position.y,
                                                    globalSimPose.pose.position.z));
    tf::StampedTransform stampedTrans = tf::StampedTransform(trans, ros::Time::now(), "/map", frameId); // frame id: /base
    tfBroadcaster.sendTransform(stampedTrans);
}

void TfUpdater::updateVtdTf(const geometry_msgs::PoseStamped &vtdPose)
{

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tf_init");
    ros::NodeHandle _nh("tf_update");

    std::string _frame_id = "base";
    TfUpdater tfUpdater(_nh, _frame_id);
    ros::spin();
    
    return 1;
}