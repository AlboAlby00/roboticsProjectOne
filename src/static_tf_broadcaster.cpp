#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class StaticBroadCaster{
    private:
        ros::NodeHandle n;
        ros::Subscriber initial_world_pose_subscriber;

    public:
        StaticBroadCaster(){
            initial_world_pose_subscriber = n.subscribe("robot/pose", 1000, &StaticBroadCaster::publish_transform, this);
        }
        
        void publish_transform(const geometry_msgs::PoseStamped& msg){
            
            static tf2_ros::StaticTransformBroadcaster static_broadcaster;
            geometry_msgs::TransformStamped static_transformStamped;

            static_transformStamped.header.stamp = ros::Time::now();
            static_transformStamped.header.frame_id = "world";
            static_transformStamped.child_frame_id = "odom";
            static_transformStamped.transform.translation.x = msg.pose.position.x;
            static_transformStamped.transform.translation.y = msg.pose.position.y;
            static_transformStamped.transform.translation.z = msg.pose.position.z;
            static_transformStamped.transform.rotation.x = msg.pose.orientation.x;
            static_transformStamped.transform.rotation.y = msg.pose.orientation.y;
            static_transformStamped.transform.rotation.z = msg.pose.orientation.z;
            static_transformStamped.transform.rotation.w = msg.pose.orientation.w;
            static_broadcaster.sendTransform(static_transformStamped);

            ROS_INFO("Spinning until killed, publishing odom to world");

            initial_world_pose_subscriber.shutdown();
        }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "static_tf_broadcaster");
    
    StaticBroadCaster s;

    ros::spin();
    
    return 0;
}