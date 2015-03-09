#ifndef EBAND_CONTROL_H
#define EBAND_CONTROL_H

#include <ros/ros.h>
#include <eband_local_planner/eband_local_planner_ros.h>

class EbandControl
{
public:
    EbandControl();
    void initialize(costmap_2d::Costmap2DROS* costmap, eband_local_planner::EBandPlannerROS* eband_planner);

    // Getters //
    geometry_msgs::Pose getLastHumanPose();
    geometry_msgs::Pose getLastRobotPose();
    costmap_2d::Costmap2DROS* getCostmapROS();

private:
    ros::AsyncSpinner spinner_;
    tf::TransformListener tf_listener_;
    geometry_msgs::Pose human_pose_, robot_pose_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    eband_local_planner::EBandPlannerROS* eband_planner_;
    ros::Subscriber robot_pose_sub_, human_pose_sub_;
    std::vector<geometry_msgs::Point> polygon_;

    void humanPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
};

geometry_msgs::Pose EbandControl::getLastHumanPose(){
    return this->human_pose_;
}

geometry_msgs::Pose EbandControl::getLastRobotPose(){
    return this->robot_pose_;
}

costmap_2d::Costmap2DROS* EbandControl::getCostmapROS(){
    return costmap_ros_;
}

#endif // EBAND_CONTROL_H
