#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <sstream>

int main(int argc, char **argv) {

  ros::init(argc, argv, "odometryNode");
  ros::NodeHandle n;

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

  double w1=5.0,w2=7.0,w3=4.0,w4=2.0;
  double vx=w2 ,vy = w4+w3;
  double w = w1;
  ros::Rate loop_rate(10);

  while (ros::ok()) {

    // generate cmd_vel msg
    geometry_msgs::TwistStamped msg;

    msg.twist.angular.z = w;
    msg.twist.linear.x=vx;
    msg.twist.linear.y=vy;







    // print count to screen
    //ROS_INFO("%f", msg.twist.angular.z);

    // publish messages
    cmd_vel_pub.publish(msg);


    ros::spinOnce();

    loop_rate.sleep();


  }

  return 0;
}
