#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "human_frame_broadcaster");
    ros::NodeHandle node;

    ros::Publisher human_pose_pub = node.advertise<geometry_msgs::Pose>("human_pose",1);
    geometry_msgs::Pose human_pose;
    human_pose.position.x = 2.0;

    ros::Rate rate(10.0);
    int i = 0;
    int freq = 20;
    while (node.ok()){
        if(i>=2*freq){
            i = 0;
        }
        human_pose.position.y = 2.8+0.3*sin(2*M_PI*i/freq);
        human_pose_pub.publish(human_pose);
        rate.sleep();
        i++;
    }
    return 0;
}
