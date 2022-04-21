#include "ros/ros.h"
#include "robotics_project_one/Reset_odometry.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <sstream>

int main(int argc, char **argv) {

  ros::init(argc, argv, "odometryNode");
  ros::NodeHandle n;

  double r,lx,ly;
  n.getParam("/r", r);
  n.getParam("/lx", lx);
  n.getParam("/ly", ly);

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

  double w1=5.0,w2=7.0,w3=4.0,w4=3.0;
  double vx=w2 ,vy = w4+w3;
  double w = w1;
  ros::Rate loop_rate(10);

  while (ros::ok()) {

    vx=(w1+w2+w3+w4)*r/4;
    vy=(-w1+w2+w3-w4)*r/4;
    w=(-w1+w2-w3+w4)*r/(4*(lx+ly));

    // generate cmd_vel msg
    geometry_msgs::TwistStamped msg;

    msg.twist.angular.z = w;
    msg.twist.linear.x=vx;
    msg.twist.linear.y=vy;







    // print count to screen
    ROS_INFO("r: %f,lx: %f,ly: %f",r,lx,ly );

    // publish messages
    cmd_vel_pub.publish(msg);


    ros::spinOnce();

    loop_rate.sleep();


  }

  return 0;
}
