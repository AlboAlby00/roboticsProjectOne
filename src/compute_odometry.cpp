#include "ros/ros.h"
#include "robotics_project_one/Reset_odometry.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

bool reset_odometry_callback(double *x,double *y,double *theta, robotics_project_one::Reset_odometry::Request  &req, 
                    robotics_project_one::Reset_odometry::Response &res) {

  *x = req.x;
  *y = req.y;
  *theta = req.theta;
  ROS_INFO("Request to reset x to %f,y to %f,theta to %f",
      req.x,req.y,req.theta);
  return true;
}

//TODO: use the formulas to compute odometry and publish it
//      broadcast TF from world frame to base frame (?)
class OdometryNode{
  private:
    ros::NodeHandle n;
    ros::Subscriber sub_wheel_states;
    ros::Publisher pub_cmd_vel;
    ros::Publisher pub_odom;
    ros::ServiceServer srv_reset_odometry;


    double r,l_x,l_y,T,N;   //parameters that will be retrieved from the parameter server

    double x,y,theta; //initial position retreived from param server, then updated with odometry values
    

    bool firstMsg;

    double previous_ticks_fl;
    double previous_ticks_fr;
    double previous_ticks_rl;
    double previous_ticks_rr;

    ros::Time previous_time;

    double v_x,v_y,w;

    int seq;

  public:
    OdometryNode(){

      pub_cmd_vel= n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
      pub_odom = n.advertise<nav_msgs::Odometry>("odom",1000);

      sub_wheel_states = n.subscribe("wheel_states", 1000, &OdometryNode::callback, this);

      srv_reset_odometry =
                n.advertiseService<robotics_project_one::Reset_odometry::Request, 
                         robotics_project_one::Reset_odometry::Response>("reset_odometry", 
          boost::bind(&reset_odometry_callback, &x,&y,&theta, _1, _2) 
      );

      //retrieve robot parameters
      n.getParam("/r",r);
      n.getParam("/lx",l_x);
      n.getParam("/ly",l_y);
      n.getParam("/T",T);
      n.getParam("/N",N);
      
      //retrieve initial position
      n.getParam("/x0",x);
      n.getParam("/y0",y);
      n.getParam("/theta0",theta);
      

      firstMsg=true;

      seq=1;

    }

    void callback(const sensor_msgs::JointState& msg){
      double ticks_fl = msg.position[0];
      double ticks_fr = msg.position[1];
      double ticks_rl = msg.position[2];
      double ticks_rr = msg.position[3];

      double w_fl,w_fr,w_rl,w_rr;

      ros::Time current_time = msg.header.stamp;

      if(!firstMsg){
        ros::Duration dt = current_time-previous_time;
        w_fl = calculate_wheel_speed(previous_ticks_fl,ticks_fl,dt);
        w_fr = calculate_wheel_speed(previous_ticks_fr,ticks_fr,dt);
        w_rl = calculate_wheel_speed(previous_ticks_rl,ticks_rl,dt);
        w_rr = calculate_wheel_speed(previous_ticks_rr,ticks_rr,dt);

        generate_cmd_vel(w_fl,w_fr,w_rl,w_rr);

        calculate_odometry(dt);

        broadcast_tf();
      }

      previous_ticks_fl = ticks_fl;
      previous_ticks_fr = ticks_fr;
      previous_ticks_rl = ticks_rl;
      previous_ticks_rr = ticks_rr;
      previous_time = current_time;

      firstMsg = false;
      return;
    }

    void generate_cmd_vel(double w_fl,double w_fr,double w_rl,double w_rr){
      //calculate v_x,v_y, w
      v_x = (w_fl + w_fr + w_rl + w_rr) * r/4;
      v_y = (-w_fl + w_fr + w_rl - w_rr) * r/4;
      w = (-w_fl + w_fr - w_rl + w_rr) * r/(4*(l_x+l_y));

      // generate cmd_vel message

      geometry_msgs::TwistStamped msg;

      //generate header
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "base_link";
      msg.header.seq = seq;

      //generate twist
      msg.twist.angular.x = 0;
      msg.twist.angular.y = 0;
      msg.twist.angular.z = w;
      msg.twist.linear.x = v_x;
      msg.twist.linear.y = v_y;
      msg.twist.linear.z = 0;

      // publish message
      pub_cmd_vel.publish(msg);
    }

    double calculate_wheel_speed(double pre_ticks,double curr_ticks, ros::Duration dt){
      return (curr_ticks-pre_ticks)*2*M_PI/(dt.toSec()*N*T);
    }

    void calculate_odometry(ros::Duration dt){
      //TODO:choose between euler and runge-kutta with dynamic configuration. For now only euler integration is used 
      
      //converting from base to world frame
      double v_wx = v_x*cos(theta) - v_y*sin(theta);
      double v_wy = v_x*sin(theta) + v_y*cos(theta);

      //create odometry message
      nav_msgs::Odometry msg = euler_odometry(v_wx,v_wy,dt);

      //publish message
      pub_odom.publish(msg);
    }

    void generate_message(nav_msgs::Odometry* msg, double v_wx, double v_wy){
      //generate position
      msg->pose.pose.position.x = x;
      msg->pose.pose.position.y = y;

      //generate orientation using tf2::Quaternion::setRPY(x,y,theta)
      tf2::Quaternion q;
      q.setRPY(0, 0, theta);
      msg->pose.pose.orientation.w = q.getW();
      msg->pose.pose.orientation.x = q.getX();
      msg->pose.pose.orientation.y = q.getY();
      msg->pose.pose.orientation.z = q.getZ();

      //generate twist
      msg->twist.twist.angular.z = w;
      msg->twist.twist.linear.x = v_wx;
      msg->twist.twist.linear.y = v_wy;

      //generate header
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "odom";
      msg->header.seq = seq;
      msg->child_frame_id = "base_link";
      seq++;
    }

    nav_msgs::Odometry euler_odometry(double v_wx,double v_wy,ros::Duration dt){
      nav_msgs::Odometry msg;

      //odometry: Euler integration
      x = x + v_wx*dt.toSec();
      y = y + v_wy*dt.toSec();
      theta = theta + w*dt.toSec();

      generate_message(&msg,v_wx,v_wy);

      return msg;
    }

    nav_msgs::Odometry runge_kutta_odometry(double v_wx,double v_wy,ros::Duration dt){
      nav_msgs::Odometry msg;

      //odometry: Runge Kutta integration
      double adjustment = w*dt.toSec()/2.0; 
      x = x + (v_wx*cos(adjustment) - v_wy*sin(adjustment))*dt.toSec();
      y = y + (v_wy*cos(adjustment) + v_wx*sin(adjustment))*dt.toSec();
      theta = theta + w*dt.toSec();

      generate_message(&msg,v_wx,v_wy);
      
      return msg; 
    }

    void broadcast_tf(){//TODO: see if it's better to move trasnform operations on a new node subscribed to the odom topic
      return;
    }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "odometryNode");

  OdometryNode odometry;

  ros::spin();

  return 0;
}




