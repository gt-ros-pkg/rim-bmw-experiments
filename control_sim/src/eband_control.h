#ifndef EBAND_CONTROL_H
#define EBAND_CONTROL_H

#include <ros/ros.h>
#include <eband_local_planner/eband_local_planner_ros.h>
#include <tf/transform_broadcaster.h>

class EbandControl
{
public:
    EbandControl();
    void initialize(costmap_2d::Costmap2DROS* costmap, eband_local_planner::EBandPlannerROS* eband_planner);

    // Getters //
    geometry_msgs::Pose getLastHumanPose();
    costmap_2d::Costmap2DROS* getCostmapROS();

private:
    ros::AsyncSpinner spinner_;
    tf::TransformListener tf_listener_;
    geometry_msgs::Pose human_pose_, robot_pose_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    eband_local_planner::EBandPlannerROS* eband_planner_;
    ros::Subscriber human_pose_sub_;
    std::vector<geometry_msgs::Point> polygon_;

    void humanPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
};

geometry_msgs::Pose EbandControl::getLastHumanPose(){
    return this->human_pose_;
}

costmap_2d::Costmap2DROS* EbandControl::getCostmapROS(){
    return costmap_ros_;
}

#endif // EBAND_CONTROL_H
