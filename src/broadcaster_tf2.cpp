#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include "tf2/transform_datatypes.h"


class TfBroad {

private:

    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    ros::Subscriber sub;

public:

    TfBroad() {
        sub = n.subscribe("/odom", 1000, &TfBroad::callback, this);
    }

    void callback(const nav_msgs::Odometry& msg){

        double x = msg.pose.pose.position.x;
        double y = msg.pose.pose.position.y;
        

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;
        
        br.sendTransform(transformStamped);

    }

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "broadcasterNode");

  TfBroad br;

  ros::spin();

  return 0;
}