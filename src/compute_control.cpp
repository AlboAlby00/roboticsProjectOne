#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include "robotics_project_one/WheelSpeeds.h"
#include "robotics_project_one/Set_compute_control_param.h"

typedef robotics_project_one::Set_compute_control_param::Request SetRequest;
typedef robotics_project_one::Set_compute_control_param::Response SetResponse;

class ComputeControl{
    
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_cmd_vel;
        ros::Publisher pub_wheels_rpm;
        ros::ServiceServer srv_set_compute_control_param;
        double r,l_x,l_y,T;   //parameters that will be retrieved from dyna
        int seq;

    public:

        ComputeControl(){
            
            pub_wheels_rpm = n.advertise<robotics_project_one::WheelSpeeds>("wheels_rpm",1000);
            sub_cmd_vel = n.subscribe("cmd_vel", 1000, &ComputeControl::callback, this);

            srv_set_compute_control_param = n.advertiseService<SetRequest,SetResponse>("set_compute_control_param",boost::bind(&ComputeControl::set_param,this, _1, _2));
            
            //retrieve fixed robot parameters
           
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

        bool set_param(SetRequest& req, SetResponse& res){

            r = req.R;
            ROS_INFO("Set r to %f in compute_control node",r);
            return true;

        }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "compute_control");
    
    ComputeControl computer;

    ros::spin();

    return 0;
}