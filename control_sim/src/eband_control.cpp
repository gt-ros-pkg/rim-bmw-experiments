#include "eband_control.h"

EbandControl::EbandControl() : spinner_(1)//, costmap_ros_("ecostmap", new tf::TransformListener())//, eband_planner_("eband_planner", &tf_listener_, &costmap_ros_)
{
    spinner_.start();
    ros::NodeHandle nh_(""), nh_param_("~");

    std::string costmap_name, eband_planner_name, robot_pose_topic, human_pose_topic;
    nh_param_.getParam("costmap_name", costmap_name);
    nh_param_.getParam("eband_planner_name", eband_planner_name);
    nh_param_.getParam("robot_pose_topic", robot_pose_topic);
    nh_param_.getParam("human_pose_topic", human_pose_topic);

    std::cout << costmap_name << std::endl;
    std::cout << eband_planner_name << std::endl;

    // TODO : load the polygon from the param server //
    geometry_msgs::Point human_point;
    human_point.x = 2;
    human_point.y = 0;
    polygon_.push_back(human_point);
    human_point.x = 0;
    human_point.y = 2;
    polygon_.push_back(human_point);
    human_point.x = 0;
    human_point.y = 1;
    polygon_.push_back(human_point);
    human_point.x = 0.2;
    human_point.y = 0.2;
    polygon_.push_back(human_point);
    human_point.x = 1;
    human_point.y = 0;
    polygon_.push_back(human_point);
/*
    costmap_2d::Costmap2DROS costmap_ros(costmap_name, tf_listener_);
    costmap_ros_ = &costmap_ros;

    eband_local_planner::EBandPlannerROS eband_planner(eband_planner_name, &tf_listener_, costmap_ros_);
    eband_planner_ = &eband_planner;
*/
    robot_pose_sub_ = nh_.subscribe<geometry_msgs::Pose>(robot_pose_topic, 1, boost::bind(&EbandControl::robotPoseCallback, this, _1));
    human_pose_sub_ = nh_.subscribe<geometry_msgs::Pose>(human_pose_topic, 1, boost::bind(&EbandControl::humanPoseCallback, this, _1));
}

void EbandControl::initialize(costmap_2d::Costmap2DROS* costmap, eband_local_planner::EBandPlannerROS* eband_planner){
    costmap_ros_ = costmap;
    eband_planner_ = eband_planner;
}

void EbandControl::humanPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    std::cout << "Human Callback" << std::endl;
    human_pose_ = *msg;

    std::vector<geometry_msgs::Point> human_poly;
    geometry_msgs::Point point;
    for(int i=0; i<polygon_.size(); i++){
        point = polygon_.at(i);
        point.x += human_pose_.position.x;
        point.y += human_pose_.position.y;
        human_poly.push_back(point);
    }

    costmap_ros_->resetLayers();
    costmap_ros_->getCostmap()->setConvexPolygonCost(human_poly, costmap_2d::LETHAL_OBSTACLE);
}

void EbandControl::robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    std::cout << "Robot Callback" << std::endl;
    robot_pose_ = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "eband_control");
    usleep(1000*1000);

    EbandControl ebandcontrol;
    tf::TransformListener tf_listener;
    costmap_2d::Costmap2DROS costmap_ros("ecostmap", tf_listener);
    eband_local_planner::EBandPlannerROS eband_planner("eband_planner", &tf_listener, &costmap_ros);
    ebandcontrol.initialize(&costmap_ros, &eband_planner);


    tf::StampedTransform transform;
    try {
        tf_listener.waitForTransform("map", "ee_link", ros::Time(0), ros::Duration(10.0) );
        tf_listener.lookupTransform("map", "ee_link", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    std::vector<geometry_msgs::PoseStamped> orig_global_plan;
    ros::Time now = ros::Time::now();
    geometry_msgs::PoseStamped first_point;
    first_point.header.frame_id = "map";
    first_point.header.stamp = now;
    first_point.pose.position.x = transform.getOrigin().x();
    first_point.pose.position.y = transform.getOrigin().y();
    first_point.pose.position.z = transform.getOrigin().z();
    first_point.pose.orientation.w = transform.getRotation().getW();
    first_point.pose.orientation.x = transform.getRotation().getX();
    first_point.pose.orientation.y = transform.getRotation().getY();
    first_point.pose.orientation.z = transform.getRotation().getZ();
    orig_global_plan.push_back(first_point);
    for(int i=0; i<10; i++){
        first_point.pose.position.x += 0.2;
        orig_global_plan.push_back(first_point);
    }


    geometry_msgs::Twist vel;
    eband_planner.setPlan(orig_global_plan);
    while(ros::ok()){
        eband_planner.computeVelocityCommands(vel);
    }

    ros::shutdown();
    return 0;
}
