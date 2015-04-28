#include "eband_control.h"

EbandControl::EbandControl() : spinner_(1)
{
    spinner_.start();
    ros::NodeHandle nh_(""), nh_param_("~");

    std::string costmap_name, eband_planner_name, human_pose_topic;
    nh_param_.getParam("costmap_name", costmap_name);
    nh_param_.getParam("eband_planner_name", eband_planner_name);
    nh_param_.getParam("human_pose_topic", human_pose_topic);

    std::cout << costmap_name << std::endl;
    std::cout << eband_planner_name << std::endl;

    // TODO : load the polygon from the param server //
    geometry_msgs::Point human_point;
    human_point.x = 1;
    human_point.y = 0;
    polygon_.push_back(human_point);
    human_point.x = 0.707;
    human_point.y = 0.707;
    polygon_.push_back(human_point);
    human_point.x = 0;
    human_point.y = 1;
    polygon_.push_back(human_point);
    human_point.x = -0.707;
    human_point.y = 0.707;
    polygon_.push_back(human_point);
    human_point.x = -1;
    human_point.y = 0;
    polygon_.push_back(human_point);
    human_point.x = -0.707;
    human_point.y = -0.707;
    polygon_.push_back(human_point);
    human_point.x = 0;
    human_point.y = -1;
    polygon_.push_back(human_point);
    human_point.x = 0.707;
    human_point.y = -0.707;
    polygon_.push_back(human_point);

    human_pose_sub_ = nh_.subscribe<geometry_msgs::Pose>(human_pose_topic, 1, boost::bind(&EbandControl::humanPoseCallback, this, _1));
}

void EbandControl::initialize(costmap_2d::Costmap2DROS* costmap, eband_local_planner::EBandPlannerROS* eband_planner){
    costmap_ros_ = costmap;
    eband_planner_ = eband_planner;
}

void EbandControl::humanPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eband_control");
    ros::NodeHandle nodeHandle;
    ros::Publisher wrenchPub = nodeHandle.advertise<geometry_msgs::WrenchStamped>("eband_control/wrench",1);
    ros::Publisher twistPub = nodeHandle.advertise<geometry_msgs::TwistStamped>("vel_cart_pos_ctrl/command_twist",1);
    usleep(1000*1000);

    /* Creation and initialization of the costmap and the elastic bands */
    EbandControl ebandcontrol;
    tf::TransformListener tf_listener;
    costmap_2d::Costmap2DROS costmap_ros("ecostmap", tf_listener);
    eband_local_planner::EBandPlannerROS eband_planner("eband_planner", &tf_listener, &costmap_ros);
    ebandcontrol.initialize(&costmap_ros, &eband_planner);

    /* Getting current location for creating a reasonable first path example */
    tf::StampedTransform transform;
    try {
        tf_listener.waitForTransform("map", "ee_link", ros::Time(0), ros::Duration(10.0) );
        tf_listener.lookupTransform("map", "ee_link", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    /* Setting the original path */
    std::vector<geometry_msgs::PoseStamped> first_path, path1, path2;
    geometry_msgs::PoseStamped first_point;
    first_point.header.frame_id = "map";
    first_point.pose.position.x = transform.getOrigin().x();
    first_point.pose.position.y = transform.getOrigin().y();
    first_point.pose.position.z = transform.getOrigin().z();
    first_point.pose.orientation.w = transform.getRotation().getW();
    first_point.pose.orientation.x = transform.getRotation().getX();
    first_point.pose.orientation.y = transform.getRotation().getY();
    first_point.pose.orientation.z = transform.getRotation().getZ();
    first_path.push_back(first_point);
    for(int i=0; i<10; i++){
        first_point.pose.position.x -= (transform.getOrigin().x()-0.0)/10;
        first_point.pose.position.y -= (transform.getOrigin().y()-1.5)/10;
        first_path.push_back(first_point);
    }
    eband_planner.setPlan(first_path);

    /* Loop sending velocity commands for first path */
    geometry_msgs::Twist vel;
    geometry_msgs::WrenchStamped wrench;
    geometry_msgs::TwistStamped twist;
    wrench.header.frame_id = "robot_frame";
    while(!eband_planner.isGoalReached()){
        eband_planner.computeVelocityCommands(vel);
        wrench.header.stamp = ros::Time::now();
        wrench.wrench.force.x = vel.linear.x;
        wrench.wrench.force.y = vel.linear.y;
        twist.twist.linear.x = vel.linear.x;
        twist.twist.linear.y = vel.linear.y;
        wrenchPub.publish(wrench);
        twistPub.publish(twist);
    }

    /* Setting the path1 and path2 */
    first_point.pose.position.x = 0.0;
    first_point.pose.position.y = 1.5;
    path1.push_back(first_point);
    for(int i=0; i<10; i++){
        first_point.pose.position.x -= (0.0-3.0)/10;
        first_point.pose.position.y -= (1.5-1.5)/10;
        path1.push_back(first_point);
    }
    first_point.pose.position.x = 3.0;
    first_point.pose.position.y = 1.5;
    path2.push_back(first_point);
    for(int i=0; i<10; i++){
        first_point.pose.position.x -= (3.0-0.0)/10;
        first_point.pose.position.y -= (1.5-1.5)/10;
        path2.push_back(first_point);
    }

    /* Loop sending velocity commands for path1 and path2 */
    while(ros::ok()){
        eband_planner.setPlan(path1);
        while(!eband_planner.isGoalReached()){
            eband_planner.computeVelocityCommands(vel);
            wrench.header.stamp = ros::Time::now();
            wrench.wrench.force.x = vel.linear.x;
            wrench.wrench.force.y = vel.linear.y;
            twist.twist.linear.x = vel.linear.x;
            twist.twist.linear.y = vel.linear.y;
            wrenchPub.publish(wrench);
            twistPub.publish(twist);
        }
        eband_planner.setPlan(path2);
        while(!eband_planner.isGoalReached()){
            eband_planner.computeVelocityCommands(vel);
            wrench.header.stamp = ros::Time::now();
            wrench.wrench.force.x = vel.linear.x;
            wrench.wrench.force.y = vel.linear.y;
            twist.twist.linear.x = vel.linear.x;
            twist.twist.linear.y = vel.linear.y;
            wrenchPub.publish(wrench);
            twistPub.publish(twist);
        }
    }
    ros::shutdown();
    return 0;
}
