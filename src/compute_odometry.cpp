#include "ros/ros.h"
#include "robotics_project_one/Reset_odometry.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include "robotics_project_one/odometryIntegrationConfig.h"
#include "ros/service_client.h"
#include "robotics_project_one/Set_compute_control_param.h"

#define EULER 0
#define RK 1

typedef robotics_project_one::Reset_odometry::Request ResetRequest;
typedef robotics_project_one::Reset_odometry::Response ResetResponse;

class OdometryNode{

    #pragma region FIELDS
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_wheel_states;
        ros::Publisher pub_cmd_vel;
        ros::Publisher pub_odom;
        ros::ServiceClient srv_set_compute_control;

        ros::ServiceServer srv_reset_odometry;

        dynamic_reconfigure::Server<robotics_project_one::odometryIntegrationConfig> dyn_server;
        dynamic_reconfigure::Server<robotics_project_one::odometryIntegrationConfig>::CallbackType dyn_callback;

        double r;               //parameter: the radius of the robot wheels
        double l_x;             //parameter: wheel position along x_base
        double l_y;             //parameter: wheel position along y_base
        double T;               //parameter: robot gear ratio
        double N;               //parameter: sensors count per revolution (CPR)

        double x;               //current position of the robot along x_odom
        double y;               //current position of the robot along y_odom
        double theta;           //current angle of the robot (odom frame)

        double v_x;             //current chassis velocity along x_base
        double v_y;             //current chassis velocity along y_base
        double w;               //current chassis angular velocity

        bool firstMsg;          //indicates if the message received is the first message. This is used when calculating the variation of ticks sensed by the wheel robot sensor

        int integrationMethod;  //is used to configure the odometry integration method and can only assume values 0,1

        double previous_ticks_fl;
        double previous_ticks_fr;
        double previous_ticks_rl;
        double previous_ticks_rr;
        ros::Time previous_time;
        
        int seq;                //used to generate seq_id in the messages header
    
    #pragma endregion FIELDS

    #pragma region METHODS
    public:
        OdometryNode(){
            //setup dynamic_reconfigure
            dyn_callback = boost::bind(&OdometryNode::dynamic_reconfigure_callback, this, _1, _2);
            dyn_server.setCallback(dyn_callback);

            //client to set l_x,l_y,T in compute_control node
            srv_set_compute_control = n.serviceClient<robotics_project_one::Set_compute_control_param>("Set_compute_control_param");

            //setup reset service

            srv_reset_odometry = n.advertiseService<ResetRequest,ResetResponse>("reset_odometry",boost::bind(&OdometryNode::reset_odometry_callback,this, _1, _2));
            
            //setup publishers
            pub_cmd_vel= n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
            pub_odom = n.advertise<nav_msgs::Odometry>("odom",1000);

            //setup subscribers
            sub_wheel_states = n.subscribe("wheel_states", 1000, &OdometryNode::wheel_states_callback, this);

            //retrieve fixed robot parameters
            //n.getParam("/r",r);
            n.getParam("/lx",l_x);
            n.getParam("/ly",l_y);
            n.getParam("/T",T);
            //n.getParam("/N",N);

            //retrieve initial position
            n.getParam("/x0",x);
            n.getParam("/y0",y);
            n.getParam("/theta0",theta);

            ROS_INFO("Initialized with r: %f\tlx: %f\tly: %f\tT: %f\tN: %f\tintegration: %d\t",r,l_x,l_y,T,N,integrationMethod); //uncomment block to debug
            
            firstMsg=true;
            seq=1;
        }

        #pragma region SECONDARY_CALLBACKS

        void dynamic_reconfigure_callback(robotics_project_one::odometryIntegrationConfig &config, uint32_t level) { 
            integrationMethod = config.integrationMethod;
            ROS_INFO("set integrationMethod to %s", integrationMethod == 0 ? "Euler" : "Runge-Kutta");
            
            //used for robot parameters calibration
            r = config.r;
            ROS_INFO("set r to %f", r);
            N = config.N;
            ROS_INFO("set N to %f", N);

            //set param in compute_control node
            robotics_project_one::Set_compute_control_param param;
            param.request.l_x=l_x;
            param.request.l_y=l_y;
            param.request.T=T;
            srv_set_compute_control.call(param);

        }

        bool reset_odometry_callback(ResetRequest& req, ResetResponse& res){
            x = req.x;
            y = req.y;
            theta = req.theta;

            firstMsg = true;

            ROS_INFO("Reset x to %f,y to %f,theta to %f",x,y,theta);

            return true;
        }

        #pragma endregion SECONDARY_CALLBACKS

        #pragma region VELOCITY

        void wheel_states_callback(const sensor_msgs::JointState& msg){
            double ticks_fl = msg.position[0];
            double ticks_fr = msg.position[1];
            double ticks_rl = msg.position[2];
            double ticks_rr = msg.position[3];

            /* ROS_INFO("Retrieved ticks fl: %f\tfr: %f\trl: %f\trr: %f",ticks_fl,ticks_fr,ticks_rl,ticks_rr); //uncomment block for debug */

            double w_fl,w_fr,w_rl,w_rr; //angular velocities of the wheels in radians per second

            ros::Time current_time = msg.header.stamp;

            if(firstMsg){
                previous_ticks_fl = ticks_fl;
                previous_ticks_fr = ticks_fr;
                previous_ticks_rl = ticks_rl;
                previous_ticks_rr = ticks_rr;
                previous_time = current_time;
                firstMsg = false;
            }

            ros::Duration dt = current_time-previous_time;
            w_fl = calculate_wheel_speed(previous_ticks_fl,ticks_fl,dt);
            w_fr = calculate_wheel_speed(previous_ticks_fr,ticks_fr,dt);
            w_rl = calculate_wheel_speed(previous_ticks_rl,ticks_rl,dt);
            w_rr = calculate_wheel_speed(previous_ticks_rr,ticks_rr,dt);

            /* ROS_INFO("Calculated wheel speeds fl: %f\tfr: %f\trl: %f\trr: %f",w_fl,w_fr,w_rl,w_rr); //uncomment block for debug */

            generate_cmd_vel(w_fl,w_fr,w_rl,w_rr);

            calculate_odometry(dt);

            previous_ticks_fl = ticks_fl;
            previous_ticks_fr = ticks_fr;
            previous_ticks_rl = ticks_rl;
            previous_ticks_rr = ticks_rr;
            previous_time = current_time;

            return;
        }

        double calculate_wheel_speed(double pre_ticks,double curr_ticks, ros::Duration dt){
            double time = dt.toSec();
            if(time==0)
                return 0;
            return (curr_ticks-pre_ticks)*2*M_PI/(time*N*T);
        }

        void generate_cmd_vel(double w_fl,double w_fr,double w_rl,double w_rr){
            //calculate v_x,v_y, w
            v_x = (w_fl + w_fr + w_rl + w_rr) * r/4.0;
            v_y = (-w_fl + w_fr + w_rl - w_rr) * r/4.0;
            w = (-w_fl + w_fr - w_rl + w_rr) * r/(4.0*(l_x+l_y));

            /* ROS_INFO("Calculated chassis speed v_x: %f\t v_y: %f\t w: %f",v_x,v_y,w); //uncomment block for debug */

            // generate cmd_vel message
            generate_cmd_vel_message();
            
        }

        #pragma endregion VELOCITY

        #pragma region ODOMETRY

        void calculate_odometry(ros::Duration dt){  
                        
            nav_msgs::Odometry msg;

            //create odometry message
            if(integrationMethod==EULER)
                euler_odometry(v_x,v_y,dt); 
            else if(integrationMethod==RK)
                runge_kutta_odometry(v_x,v_y,dt); 
        }

        void euler_odometry(double v_x,double v_y,ros::Duration dt){
            
            //euler method
            double deltaBaseX = v_x*dt.toSec();
            double deltaBaseY = v_y*dt.toSec();
            double deltaTheta = w*dt.toSec();

            //update pose
            
            theta = theta + deltaTheta;
            if(theta > M_PI)
                theta = theta - 2.0*M_PI;
            else if(theta < -M_PI)
                theta = theta + 2.0*M_PI;

            //rotate to the fixed frame
            double deltaX = rotateX(deltaBaseX,deltaBaseY);
            double deltaY = rotateY(deltaBaseX,deltaBaseY);
            x = x + deltaX;
            y = y + deltaY;

            /* ROS_INFO("Euler generated odometry x: %f\ty: %f\ttheta: %f",x,y,theta); //uncomment block to debug */

            //generate updated pose message
            generate_odometry_message(v_x,v_y);
        }

        void runge_kutta_odometry(double v_x,double v_y,ros::Duration dt){
            double time =dt.toSec();
            double v = sqrt(pow(v_x,2.0)+pow(v_y,2.0));//todo:restore old runge-kutta formula

            //runge-kutta method
            double adjustment = w*time/2.0; 
            double deltaBaseX = (v_x*cos(adjustment) - v_y*sin(adjustment))*time;
            double deltaBaseY = (v_x*sin(adjustment) + v_y*cos(adjustment))*time;
            double deltaTheta = w*time;

            //update pose
            
            theta = theta + deltaTheta;
            if(theta > M_PI)
                theta = theta - 2.0*M_PI;
            else if(theta < -M_PI)
                theta = theta + 2.0*M_PI;

            //rotate to the fixed frame
            double deltaX = rotateX(deltaBaseX,deltaBaseY);
            double deltaY = rotateY(deltaBaseX,deltaBaseY);
            x = x + deltaX;
            y = y + deltaY;


            /* ROS_INFO("Runge-Kutta generated odometry x: %f\ty: %f\ttheta: %f",x,y,theta); //uncomment block to debug*/

            generate_odometry_message(v_x,v_y);
        }

        #pragma endregion ODOMETRY

        #pragma region ROTATION

        double rotateX(double baseX,double baseY){
            return cos(theta)*baseX - sin(theta)*baseY;
        }

        double rotateY(double baseX,double baseY){
            return sin(theta)*baseX + cos(theta)*baseY;
        }

        #pragma endregion ROTATION

        #pragma region PUBLISH_MESSAGES

        void generate_cmd_vel_message(){
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

        void generate_odometry_message(double v_x, double v_y){
            nav_msgs::Odometry msg;
            
            //generate position
            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = y;

            //generate orientation using tf2::Quaternion::setRPY(x,y,theta)
            tf2::Quaternion q;
            q.setRPY(0, 0, theta);
            msg.pose.pose.orientation.w = q.getW();
            msg.pose.pose.orientation.x = q.getX();
            msg.pose.pose.orientation.y = q.getY();
            msg.pose.pose.orientation.z = q.getZ();

            //generate twist
            msg.twist.twist.angular.z = w;
            msg.twist.twist.linear.x = rotateX(v_x,v_y);
            msg.twist.twist.linear.y = rotateY(v_x,v_y);

            //generate header
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "odom";
            msg.header.seq = seq;
            msg.child_frame_id = "base_link";
            seq++;

            //publish message
            pub_odom.publish(msg);
        }

        #pragma endregion PUBLISH_MESSAGES


    #pragma endregion METHODS
};

int main(int argc, char* argv[]){
    ros::init(argc,argv,"odometry_node");

    OdometryNode node;

    ros::spin();

    return 0;
}