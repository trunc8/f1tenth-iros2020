/*
# Copyright 2018 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <iostream>
#include <map>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include "MPC.h"
#include <Eigen/Core>
#include <Eigen/QR>

using namespace std;
using namespace Eigen;

/********************/
/* CLASS DEFINITION */
/********************/
class MPCNode
{
    public:
        MPCNode();
        int get_thread_numbers();
        
    private:

        struct CarParams {
            double wheelbase;
            double friction_coeff;
            double h_cg; // height of car's CG
            double l_f; // length from CG to front axle
            double l_r; // length from CG to rear axle
            double cs_f; // cornering stiffness coeff for front wheels
            double cs_r; // cornering stiffness coeff for rear wheels
            double mass;
            double I_z; // moment of inertia about z axis from CG
};


        ros::NodeHandle _nh;
        ros::Subscriber _sub_odom, _sub_path, _sub_goal;
        ros::Publisher _pub_ackermann;
        ros::Timer _timer1;
        tf::TransformListener _tf_listener;

        geometry_msgs::Point _goal_pos;
        nav_msgs::Odometry _odom;
        trajectory_msgs::JointTrajectory _path; 
        ackermann_msgs::AckermannDriveStamped _ackermann_msg;

        string _globalPath_topic, _goal_topic;
        string _map_frame, _odom_frame, _car_frame;

        MPC _mpc;
        map<string, double> _mpc_params;
        double _mpc_steps, _ref_cte, _ref_epsi, _ref_vel, _w_cte, _w_epsi, _w_vel, 
               _w_delta, _w_accel, _w_delta_d, _w_accel_d, _bound_value;

        double _Lf, _dt, _steering, _throttle, _speed, _max_speed;
        double _pathLength, _goalRadius, _waypointsDist;
        int _controller_freq, _downSampling, _thread_numbers;
        bool _goal_received, _goal_reached, _path_computed;

        // Parameters
        double max_speed, _max_steering_angle;
        double _max_accel, max_steering_vel, max_decel;
        double desired_speed, desired_steer_ang;
        CarParams params;
        double width;
        double slip_angle;

        double polyeval(Eigen::VectorXd coeffs, double x);
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const trajectory_msgs::JointTrajectory::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        
        void controlLoopCB(const ros::TimerEvent&);

}; // end of class


MPCNode::MPCNode()
{
    //Private parameters handler
    ros::NodeHandle pn("~");
    
    //Parameters for control loop
    pn.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node
    //pn.param("pub_twist_cmd", _pub_twist_flag, true);
    //pn.param("debug_info", _debug_info, false);
    //pn.param("delay_mode", _delay_mode, true);
    pn.param("max_speed", _max_speed,7.0); // unit: m/s
    pn.param("waypoints_dist", _waypointsDist, -1.0); // unit: m
    pn.param("path_length", _pathLength, 8.0); // unit: m
    pn.param("goal_radius", _goalRadius, 0.5); // unit: m
    pn.param("controller_freq", _controller_freq, 10);
    pn.param("vehicle_Lf", _Lf, 0.15875); // distance between the front of the vehicle and its center of gravity
    _dt = double(1.0/_controller_freq); // time step duration dt in s 

    // model parameters
    // Fetch the car parameters
    int scan_beams;
    double update_pose_rate, scan_std_dev;
    pn.getParam("wheelbase", params.wheelbase);
    pn.getParam("max_speed", max_speed);
    pn.getParam("max_steering_angle", _max_steering_angle);
    pn.getParam("max_accel", _max_accel);
    pn.getParam("max_decel", max_decel);
    pn.getParam("max_steering_vel", max_steering_vel);
    pn.getParam("friction_coeff", params.friction_coeff);
    pn.getParam("height_cg", params.h_cg);
    pn.getParam("l_cg2rear", params.l_r);
    pn.getParam("l_cg2front", params.l_f);
    pn.getParam("C_S_front", params.cs_f);
    pn.getParam("C_S_rear", params.cs_r);
    pn.getParam("moment_inertia", params.I_z);
    pn.getParam("mass", params.mass);
    pn.getParam("width", width);


    //Parameter for MPC solver
    pn.param("mpc_steps", _mpc_steps, 20.0);
    pn.param("mpc_ref_cte", _ref_cte, 0.0);
    pn.param("mpc_ref_epsi", _ref_epsi, 0.0);
    //pn.param("mpc_ref_vel", _ref_vel, 1.5);
    pn.param("mpc_w_cte", _w_cte, 100.0);
    pn.param("mpc_w_epsi", _w_epsi, 100.0);
    pn.param("mpc_w_vel", _w_vel, 100.0);
    pn.param("mpc_w_delta", _w_delta, 100.0);
    pn.param("mpc_w_accel", _w_accel, 50.0);
    pn.param("mpc_w_delta_d", _w_delta_d, 0.0);
    pn.param("mpc_w_accel_d", _w_accel_d, 0.0);
    pn.param("mpc_max_steering", _max_steering_angle, 0.4189); // Maximal steering radian (~30 deg)
    pn.param("mpc_max_throttle", _max_accel, 7.51); // Maximal throttle accel
    pn.param("mpc_bound_value", _bound_value, 1.0e3); // Bound value for other variables

    //Parameter for topics & Frame name
    //pn.param<std::string>("global_path_topic", _globalPath_topic, "/move_base/TrajectoryPlannerROS/global_plan" );
    pn.param<std::string>("goal_topic", _goal_topic, "/move_base_simple/goal" );
    pn.param<std::string>("map_frame", _map_frame, "map" );
    pn.param<std::string>("odom_frame", _odom_frame, "odom");
    pn.param<std::string>("car_frame", _car_frame, "base_link" );

    //Publishers and Subscribers
    _sub_odom   = _nh.subscribe("/odom", 1, &MPCNode::odomCB, this);
    _sub_path   = _nh.subscribe("/waypoints", 1, &MPCNode::pathCB, this);
   // _sub_goal   = _nh.subscribe( _goal_topic, 1, &MPCNode::goalCB, this);
    _pub_ackermann = _nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);

    //Timer
    _timer1 = _nh.createTimer(ros::Duration((1.0)/_controller_freq), &MPCNode::controlLoopCB, this); // 10Hz

    //Init variables
    _goal_received = false;
    _goal_reached  = false;
    _path_computed = false;
    _throttle = 0.0; 
    _steering = 0.0;
    _speed = 0.0;
    slip_angle = 0.0;
    _ackermann_msg = ackermann_msgs::AckermannDriveStamped();

    //Init parameters for MPC object
    _mpc_params["DT"] = _dt;
    _mpc_params["l_cg2front"] = _Lf;
    _mpc_params["STEPS"]    = _mpc_steps;
    _mpc_params["REF_CTE"]  = _ref_cte;
    _mpc_params["REF_EPSI"] = _ref_epsi;
    _mpc_params["REF_V"]    = _ref_vel;
    _mpc_params["W_CTE"]    = _w_cte;
    _mpc_params["W_EPSI"]   = _w_epsi;
    _mpc_params["W_V"]      = _w_vel;
    _mpc_params["W_DELTA"]  = _w_delta;
    _mpc_params["W_A"]      = _w_accel;
    _mpc_params["W_DDELTA"] = _w_delta_d;
    _mpc_params["W_DA"]     = _w_accel_d;
    _mpc_params["max_steering_angle"]   = _max_steering_angle;
    _mpc_params["max_accel"]   = _max_accel;
    _mpc_params["BOUND"]    = _bound_value;
    _mpc.LoadParams(_mpc_params);

}


// Public: return _thread_numbers
int MPCNode::get_thread_numbers()
{
    return _thread_numbers;
}


// Evaluate a polynomial.
double MPCNode::polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPCNode::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
            A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

// CallBack: Update odometry
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    _odom = *odomMsg;
    const double px = _odom.pose.pose.position.x; //pose: odom frame
    const double py = _odom.pose.pose.position.y;

    int N = _path.points.size();
    const double dx = _path.points[N].positions[0] - px;
    const double dy = _path.points[N].positions[1] - py;

    if( std::sqrt(dx*dx + dy*dy) <= 0.4 ) {
        _goal_reached = true;
        ROS_INFO("Goal reached");
    }
    else {
        _goal_reached = false;
        ROS_INFO("Goal not reached, keep going"); 
    }
}

// CallBack: Update path waypoints (conversion to odom frame)
void MPCNode::pathCB(const trajectory_msgs::JointTrajectory::ConstPtr& pathMsg)
{
    if(!_goal_reached)
    {
        _path.points = pathMsg->points;  
        _path_computed = true;
        double speed = std::sqrt(pathMsg->points.velocities[0]*pathMsg->points.velocities[0] + 
                                 pathMsg->points.velocities[1]*pathMsg->points.velocities[1]);

        ref_vel = speed;
    }
}


// Timer: Control Loop (closed loop nonlinear MPC)
void MPCNode::controlLoopCB(const ros::TimerEvent&)
{      
    if(!_goal_reached && _path_computed ) //received goal & goal not reached    
    {    
        nav_msgs::Odometry odom = _odom; 
        trajectory_msgs::JointTrajectory path = _path;   

        // Update system states: X=[x, y, psi, v]
        const double px = odom.pose.pose.position.x; //pose: odom frame
        const double py = odom.pose.pose.position.y;
        tf::Pose pose;
        tf::poseMsgToTF(odom.pose.pose, pose);
        const double psi = tf::getYaw(pose.getRotation());
        const double v = odom.twist.twist.linear.x; //twist: body fixed frame
        const double w = odom.twist.twist.angular.z; // yaw rate
        // Update system inputs: U=[steering, throttle]
        const double steer_angle = _steering;  // radian
        const double accel = _throttle; // accel: >0; brake: <0
        const double dt = _dt;
        const double Lf = _Lf;

        // Waypoints related parameters
        const int N = path.points.size(); // Number of waypoints
        const double cospsi = cos(psi);
        const double sinpsi = sin(psi);

        // Convert to the vehicle coordinate system
        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for(int i = 0; i < N; i++) 
        {
            const double dx = _path.points[i].positions[0] - px;
            const double dy = _path.points[i].positions[1] - py;
            x_veh[i] = dx * cospsi + dy * sinpsi;
            y_veh[i] = dy * cospsi - dx * sinpsi;
        }
        
        // Fit waypoints
        auto coeffs = polyfit(x_veh, y_veh, 3); 

        const double cte  = polyeval(coeffs, 0.0);
        const double epsi = atan(coeffs[1]);
        VectorXd state(8);
       
        // Kinematic model is used to predict vehicle state at the actual
        // moment of control (current time + delay dt)
        
        const double x_dot = v * std::cos(psi + slip_angle);
        const double y_dot = v * std::sin(psi + slip_angle);
        const double v_dot = accel;
        const double theta_dot = w;

        double g = 9.91;

        CarParams p;

        // for eases of next two calculations
        double rear_val = g * p.l_r - accel * p.h_cg;
        double front_val = g * p.l_f + accel * p.h_cg;

        // in case velocity is 0
        double vel_ratio, first_term;
        if (v == 0) {
            vel_ratio = 0;
            first_term = 0;
        }
        else 
        {
            vel_ratio = w / v;
            first_term = p.friction_coeff / (v * (p.l_r + p.l_f));
        }

        double theta_double_dot = (p.friction_coeff * p.mass / (p.I_z * p.wheelbase)) *
            (p.l_f * p.cs_f * steer_angle * (rear_val) +
             slip_angle * (p.l_r * p.cs_r * (front_val) - p.l_f * p.cs_f * (rear_val)) -
             vel_ratio * (std::pow(p.l_f, 2) * p.cs_f * (rear_val) + std::pow(p.l_r, 2) * p.cs_r * (front_val)));\

        double slip_angle_dot = (first_term) *
            (p.cs_f * steer_angle * (rear_val) -
             slip_angle * (p.cs_r * (front_val) + p.cs_f * (rear_val)) +
             vel_ratio * (p.cs_r * p.l_r * (front_val) - p.cs_f * p.l_f * (rear_val))) - w;

        const double px_act = px + x_dot * dt;
        const double py_act = py + y_dot*dt;
        const double psi_act = psi + theta_dot * dt;
        const double v_act = v + v_dot * dt;
        const double w_act = w + theta_double_dot * dt;
        const double slip_angle_act = slip_angle + slip_angle_dot * dt;
        const double cte_act = cte + v * sin(epsi) * dt;
        const double epsi_act = -epsi + psi_act;             
        state << px_act, py_act, psi_act, v_act, cte_act,epsi_act,w_act, slip_angle_act;
        
        // Solve MPC Problem
        vector<double> mpc_results = _mpc.Solve(state, coeffs);
              
        // MPC result (all described in car frame)        
        _steering = mpc_results[0]; // radian
        _throttle = mpc_results[1]; // acceleration
        _speed = v + _throttle*dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if(_speed <= 0.0)
            _speed = 0.0;

    }
    else
    {
        _steering = 0.0;
        _throttle = 0.0;
        _speed = 0.0;
        if(_goal_reached && _goal_received)
            cout << "Goal Reached !" << endl;
    }

    // publish cmd 
    _ackermann_msg.header.frame_id = _car_frame;
    _ackermann_msg.header.stamp = ros::Time::now();
    _ackermann_msg.drive.steering_angle = _steering;
    _ackermann_msg.drive.speed = _speed;
    _ackermann_msg.drive.acceleration = _throttle;
    _pub_ackermann.publish(_ackermann_msg);        

}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "MPC_Node");
    MPCNode mpc_node;

    ROS_INFO("Waiting for global path msgs ~");
    ros::AsyncSpinner spinner(mpc_node.get_thread_numbers()); // Use multi threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}