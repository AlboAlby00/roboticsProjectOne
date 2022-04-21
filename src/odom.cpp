#include "ros/ros.h"
#include "robotics_project_one/Reset_odometry.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

//todo: subscribe to wheel_states topic and get ticks for the 4 wheels
//      use the formulas to compute odometry and publish it
//      broadcast TF from world frame to base frame (?)
class Odom{
  private:
    ros::NodeHandle n;
    ros::Subscriber sub_wheel_states;
    ros::Publisher pub_cmd_vel;
    ros::Publisher pub_odom;
    
    double r,l_x,l_y,T,N;   //parameters that will be retrieved from the parameter server
    
    bool firstMsg;

    double previous_ticks_fl;
    double previous_ticks_fr;
    double previous_ticks_rl;
    double previous_ticks_rr;

    ros::Time previous_time;

    double v_x,v_y,w;
    
  public:
    Odom(){
      pub_cmd_vel= n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
      pub_odom = n.advertise<nav_msgs::Odometry>("odom",1000);
      sub_wheel_states = n.subscribe("wheel_states", 1000, &Odom::callback, this);
      
      //retrieve robot parameters
      n.getParam("/r",r);
      n.getParam("/lx",l_x);
      n.getParam("/ly",l_y);
      n.getParam("/T",T);
      n.getParam("/N",N);

      firstMsg=true;
    }

    void callback(const sensor_msgs::JointState& msg){
      double ticks_fl = msg.position[0];
      double ticks_fr = msg.position[1];
      double ticks_rl = msg.position[2];
      double ticks_rr = msg.position[3];

      double w_fl,w_fr,w_rl,w_rr;

      ros::Time current_time = msg.header.stamp;

      if(firstMsg){
        previous_ticks_fl = ticks_fl;
        previous_ticks_fr = ticks_fr;
        previous_ticks_rl = ticks_rl;
        previous_ticks_rr = ticks_rr;
        previous_time = current_time;

        firstMsg = false;
        return;
      }

      ros::Duration dt=current_time-previous_time;
      w_fl = calculate_wheel_speed(previous_ticks_fl,ticks_fl,dt);
      w_fr = calculate_wheel_speed(previous_ticks_fr,ticks_fr,dt);
      w_rl = calculate_wheel_speed(previous_ticks_rl,ticks_rl,dt);
      w_rr = calculate_wheel_speed(previous_ticks_rr,ticks_rr,dt);

      generate_cmd_vel(w_fl,w_fr,w_rl,w_rr);

      calculate_odometry();
    }

    void generate_cmd_vel(double w_fl,double w_fr,double w_rl,double w_rr){
      //calculate v_x,v_y, w
      v_x = (w_fl + w_fr + w_rl + w_rr) * r/4;
      v_y = (-w_fl + w_fr + w_rl - w_rr) * r/4;
      w = (-w_fl + w_fr - w_rl + w_rr) * r/(4*(l_x+l_y));

      // generate cmd_vel msg
      geometry_msgs::TwistStamped msg;

      msg.twist.angular.z = w;
      msg.twist.linear.x = v_x;
      msg.twist.linear.y = v_y;

      // publish messages
      pub_cmd_vel.publish(msg);
    }

    double calculate_wheel_speed(double pre_ticks,double curr_ticks, ros::Duration dt){
      return (curr_ticks-pre_ticks)*2*M_PI/(dt.toSec()*N*T);
    }

    void calculate_odometry(){
      //todo:choose between euler and runge-kutta
    }

};


int main(int argc, char **argv) {

  ros::init(argc, argv, "odometryNode");
  
  Odom odometry;

  ros::spin();

  return 0;
}
