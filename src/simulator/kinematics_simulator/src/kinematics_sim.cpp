#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "plan_utils/traj_container.hpp"
#include "kinematics_simulator/MincoTraj.h"
#include "kinematics_simulator/SingleMinco.h"
#include "kinematics_simulator/Trajectory.h"

#define OMINIDIRECTION 0
#define DIFFERENTIAL   1
#define ACKERMANN 	   2

using namespace std;

// ros interface
ros::Subscriber traj_sub;
ros::Publisher  odom_pub;
ros::Publisher  mesh_pub;
ros::Timer simulate_timer;
visualization_msgs::Marker marker;

// simulator variables
plan_utils::TrajContainer newest_trajectory;
std::default_random_engine generator;
std::normal_distribution<double> distribution{0.0,1.0};

double x = 0.0;
double y = 0.0;
double yaw = 0.0;
double vx = 0.0;
double vy = 0.0;
double w = 0.0;
bool rcv_cmd = false;
bool rcv_traj = false;

// simulator parameters
int ugv_type = DIFFERENTIAL;
int car_id = 0;
double init_x = 0.0;
double init_y = 0.0;
double init_yaw = 0.0;
double time_resolution = 0.01;
double max_longitude_speed = 1.5;
double max_latitude_speed = 1.5;
double max_angular_vel = 2.4;
double max_steer_angle = 0.7;
double time_delay = 0.0;
double wheel_base = 0.5;
double noise_std = 0.1;
Eigen::Quaterniond q_mesh;
Eigen::Vector3d pos_mesh;

// utils
void normYaw(double& th)
{
	while (th > M_PI)
		th -= M_PI * 2;
	while (th < -M_PI)
		th += M_PI * 2;
}

Eigen::Vector2d guassRandom2d(double std)
{
	return std * Eigen::Vector2d(distribution(generator), distribution(generator));
}

Eigen::Vector3d guassRandom3d(double std)
{
	return std * Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));
}

double guassRandom(double std)
{
	return std * distribution(generator);
}

void rcvTrajCallBack(const kinematics_simulator::TrajectoryConstPtr traj_msg)
{
    rcv_traj = true;

	plan_utils::MinJerkOpt jerk_opter;
    // std::vector<plan_utils::LocalTrajData> minco_traj;
    plan_utils::TrajContainer surround_traj;
    std::vector<bool> reverse;
    double total_time = 0.0;
    for(int i = 0; i < traj_msg->minco_path.trajs.size(); i++)
    {
        double start_time = traj_msg->minco_path.trajs.at(i).start_time.toSec();
        kinematics_simulator::SingleMinco sm = traj_msg->minco_path.trajs[i];
        Eigen::MatrixXd posP(2, sm.pos_pts.size() - 2);
        Eigen::VectorXd T(sm.t_pts.size());
        Eigen::MatrixXd head(2, 3), tail(2, 3);
        const int N = sm.t_pts.size();
        reverse.push_back(sm.reverse);
        int direction = sm.reverse?-1:1;

        for(int j = 1; j < (int)sm.pos_pts.size() - 1; j++)
        {
            posP(0, j - 1) = sm.pos_pts[j].x;
            posP(1, j - 1) = sm.pos_pts[j].y;
        }
        for(int j = 0; j < (int)sm.t_pts.size(); j++)
        {
            T(j) = sm.t_pts[j];
        }
        head.row(0) = Eigen::Vector3d(sm.head_x.x, sm.head_x.y, sm.head_x.z);
        head.row(1) = Eigen::Vector3d(sm.head_y.x, sm.head_y.y, sm.head_y.z);
        tail.row(0) = Eigen::Vector3d(sm.tail_x.x, sm.tail_x.y, sm.tail_x.z);
        tail.row(1) = Eigen::Vector3d(sm.tail_y.x, sm.tail_y.y, sm.tail_y.z);

        jerk_opter.reset(head, tail, N);
        jerk_opter.generate(posP, T);
        plan_utils::Trajectory traj = jerk_opter.getTraj(direction);

        surround_traj.addSingulTraj(traj, start_time, car_id);

        total_time += traj.getTotalDuration();
        // minco_traj.push_back(sur_traj);
    }
	newest_trajectory = surround_traj;	
}

void simCallback(const ros::TimerEvent &e)
{
	nav_msgs::Odometry new_odom;

	double time_now = ros::Time::now().toSec();
	new_odom.header.stamp    = ros::Time::now();
	new_odom.header.frame_id = "map";

	if(rcv_traj)
	{
		plan_utils::Trajectory* cur_segment;
		int segment_Id = newest_trajectory.locateSingulId(time_now);
		double segment_start_time = newest_trajectory.singul_traj[segment_Id].start_time;
		double segment_end_time   = newest_trajectory.singul_traj[segment_Id].end_time;
		double pt_time;
		if(time_now - segment_end_time < 0)
			pt_time = time_now - segment_start_time;
		else
			pt_time = segment_end_time - segment_start_time;
		cur_segment = &newest_trajectory.singul_traj[segment_Id].traj;

		int singul;
		Eigen::Matrix2d B_h;
		B_h << 0, -1,
			   1,  0;
		double cur_yaw, vel, angul_vel;
		Eigen::Vector2d sigma, dsigma, ddsigma;
		sigma   = cur_segment->getPos(pt_time);
		dsigma  = cur_segment->getdSigma(pt_time);
		ddsigma = cur_segment->getddSigma(pt_time);
		singul  = cur_segment->getSingul(pt_time);
		cur_yaw = cur_segment->getAngle(pt_time);
		vel = singul * dsigma.norm();
		angul_vel = (ddsigma.transpose() * B_h * dsigma)(0, 0) / dsigma.squaredNorm();

		x = sigma(0); y = sigma(1);
		yaw = cur_yaw;
		vx = vel; vy = 0;
		w = angul_vel;
	}

	new_odom.pose.pose.position.x  = x;
	new_odom.pose.pose.position.y  = y;
	new_odom.pose.pose.position.z  = 0;
	new_odom.pose.pose.orientation.w  = cos(yaw/2.0);
	new_odom.pose.pose.orientation.x  = 0;
	new_odom.pose.pose.orientation.y  = 0;
	new_odom.pose.pose.orientation.z  = sin(yaw/2.0);;
	new_odom.twist.twist.linear.x  = vx;
	new_odom.twist.twist.linear.y  = vy;
	new_odom.twist.twist.linear.z  = 0;
	new_odom.twist.twist.angular.x = 0;
	new_odom.twist.twist.angular.y = 0;
	new_odom.twist.twist.angular.z = w;	

	odom_pub.publish(new_odom);
	

	Eigen::Quaterniond q_shift(0.0, 0.0, -0.7071, -0.7071);
	Eigen::Quaterniond q_odom;
	q_odom = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Quaterniond q_marker = q_odom * q_shift;
	marker.pose.orientation.w = q_marker.w();
	marker.pose.orientation.x = q_marker.x();
	marker.pose.orientation.y = q_marker.y();
	marker.pose.orientation.z = q_marker.z();	

	Eigen::Vector3d pos_shift{1.3, 0.0, 0.0};
	Eigen::Vector3d pos(x, y, 0.0);
	pos = q_odom * pos_shift + pos;
	marker.pose.position.x = pos.x();
	marker.pose.position.y = pos.y();
	marker.pose.position.z = pos.z();

	mesh_pub.publish(marker);
}

// main loop
int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "simulator_node");
    ros::NodeHandle nh("~");

	nh.getParam("car_id", car_id);
	nh.getParam("ugv_type", ugv_type);
	nh.getParam("init_x", init_x);
	nh.getParam("init_y", init_y);
	nh.getParam("init_yaw", init_yaw);
	nh.getParam("time_resolution", time_resolution);
	nh.getParam("max_longitude_speed", max_longitude_speed);
	nh.getParam("max_latitude_speed", max_latitude_speed);
	nh.getParam("max_angular_vel", max_angular_vel);
	nh.getParam("max_steer_angle", max_steer_angle);
	nh.getParam("time_delay", time_delay);
	nh.getParam("wheel_base", wheel_base);
	nh.getParam("noise_std", noise_std);
	
	traj_sub = nh.subscribe("traj", 100, rcvTrajCallBack);
    odom_pub  = nh.advertise<nav_msgs::Odometry>("odom", 10);
	mesh_pub = nh.advertise<visualization_msgs::Marker>("mesh", 100);
	
	x = init_x;
	y = init_y;
	yaw = init_yaw;


	marker.header.frame_id = "map";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.mesh_resource = "package://kinematics_simulator/meshes/bmw_x5.dae";
	
	marker.mesh_use_embedded_materials = true;
	marker.pose.position.x = init_x;
	marker.pose.position.y = init_y;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.w = 0.5;
	marker.pose.orientation.x = 0.5;
	marker.pose.orientation.y = 0.5;
	marker.pose.orientation.z = 0.5;
	marker.color.a = 1.0;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;	

	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	q_mesh = Eigen::Quaterniond(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2));
	pos_mesh = Eigen::Vector3d(-0.75, 0.35, 0.0);
	

    simulate_timer = nh.createTimer(ros::Duration(time_resolution), simCallback);

	ros::spin();

    return 0;
}