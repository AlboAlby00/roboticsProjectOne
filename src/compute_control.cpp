#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include "robotics_project_one/WheelSpeeds.h"

class ComputeControl{
    
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_cmd_vel;
        ros::Publisher pub_wheels_rpm;
        double r,l_x,l_y,T;   //parameters that will be retrieved from the parameter server
        int seq;

    public:

        ComputeControl(){
            
            pub_wheels_rpm = n.advertise<robotics_project_one::WheelSpeeds>("wheels_rpm",1000);
            sub_cmd_vel = n.subscribe("cmd_vel", 1000, &ComputeControl::callback, this);
            
            //retrieve fixed robot parameters
            r=0.07;
            n.getParam("/lx",l_x);
            n.getParam("/ly",l_y);
            n.getParam("/T",T);

            ROS_INFO("set r to %f",r);

            //initialize frame_id
            seq = 1;
        }

        void callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
            robotics_project_one::WheelSpeeds response;

            double v_x = msg->twist.linear.x;
            double v_y = msg->twist.linear.y;
            double w = msg->twist.angular.z;

            //calculate speeds
            response.rpm_fl = 60*T*(v_x-v_y-(l_x+l_y)*w)/r;
            response.rpm_fr = 60*T*(v_x+v_y+(l_x+l_y)*w)/r;
            response.rpm_rl = 60*T*(v_x+v_y-(l_x+l_y)*w)/r;
            response.rpm_rr = 60*T*(v_x-v_y+(l_x+l_y)*w)/r;

            //generate header
            response.header.stamp = ros::Time::now();
            response.header.frame_id = "base_link";
            response.header.seq = seq;
            seq++;

            pub_wheels_rpm.publish(response);
        }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "compute_control");
    
    ComputeControl computer;

    ros::spin();

    return 0;
}