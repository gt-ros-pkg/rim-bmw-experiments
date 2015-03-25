#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_frame_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::TransformListener tf_listener;
    tf::StampedTransform robot_transform;
    try {
        tf_listener.waitForTransform("table_link", "ee_link", ros::Time(0), ros::Duration(10.0) );
        tf_listener.lookupTransform("table_link", "ee_link", ros::Time(0), robot_transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    ros::Rate rate(10.0);
    while (node.ok()){
        try {
            tf_listener.waitForTransform("table_link", "ee_link", ros::Time(0), ros::Duration(10.0) );
            tf_listener.lookupTransform("table_link", "ee_link", ros::Time(0), robot_transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
        transform.setOrigin( tf::Vector3(robot_transform.getOrigin().x(), robot_transform.getOrigin().y(), robot_transform.getOrigin().z()) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "table_link", "robot_frame"));
        rate.sleep();
    }
    return 0;
}
