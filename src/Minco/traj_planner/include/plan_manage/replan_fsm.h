#ifndef _REPLAN_FSM_H_
#define _REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <mapping.h>
#include <plan_manage/traj_manager.h>
#include "swarm_bridge/Trajectory.h"


const double TIME_BUDGET = 0.1;

class ReplanFSM
{
private:

    enum FSM_EXEC_STATE
    {
        INIT,
        WAIT_TARGET,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        EMERGENCY_STOP,
        SEQUENTIAL_START
    };

    enum TARGET_TYPE
    {
        MANNUAL_TARGET = 1,
        PRESET_TARGET = 2,
        REFERENCE_PATH = 3
    };


    MappingProcess::Ptr mapping_ptr_;
    TrajPlanner::Ptr planner_ptr_;
    ros::NodeHandle nh_;

    ros::Subscriber parking_sub_, odom_sub_, swarm_traj_sub_;
    ros::Timer exec_timer_, safety_timer_;


    bool have_target_, collision_with_obs_, collision_with_othercars_;
    Eigen::Vector4d init_state_;
    Eigen::Vector4d end_pt_;
    Eigen::Vector2d cur_pos_;
    double cur_yaw_, cur_vel_;
    int car_id_;
    double car_d_cr_;
    double start_world_time_;
    double target_x_, target_y_, target_yaw_;

    FSM_EXEC_STATE exec_state_;

    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void ParkingCallback(const geometry_msgs::PoseStamped &msg);
    void OdomCallback(const nav_msgs::Odometry& msg);
    void SwarmTrajCallback(const swarm_bridge::Trajectory& traj_msg);
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);

public:
    ReplanFSM()
    {
    }
    ~ReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);
    std::string odom_topic_ = "map";
    


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif