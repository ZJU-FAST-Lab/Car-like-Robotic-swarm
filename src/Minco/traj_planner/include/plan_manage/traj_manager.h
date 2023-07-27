#ifndef _TRAJ_MANAGER_H_
#define _TRAJ_MANAGER_H_

#include <memory>
#include <set>
#include <string>
#include <thread>
#include <iostream>
#include <sstream>
 
#include <ros/ros.h>

#include "minco_config.pb.h"
#include "mapping.h"
// #include "mpc/Trajectory.h"
#include "kinematics_simulator/Trajectory.h"
#include "swarm_bridge/Trajectory.h"

#include "traj_optimizer.h"

#include "decomp_util/ellipsoid_decomp.h"
#include "decomp_ros_utils/data_ros_utils.h"

// #include "plan_utils/CorridorBuilder2d.hpp"
#include "path_searching/kino_astar.h"
#include "tf/tf.h"
#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_loader.hpp>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
using std::vector;

  
class TrajPlanner
{
public:

    ///////////////////////////////////////////////////////////////
    TrajPlanner(){}
    ~TrajPlanner(){}
    typedef std::shared_ptr<TrajPlanner> Ptr;

    ros::NodeHandle nh_;
    MappingProcess::Ptr map_ptr_;

    // void init(const std::string config_path, const int car_id);
    void init(const ros::NodeHandle& nh);
    void setMap(const MappingProcess::Ptr& ptr);
    void RunOnceParking();
    bool RunMINCOParking();
    // void publishTraj2Controller();
    void publishTraj2Simulator();
    void broadcastTraj2SwarmBridge();

    void setInitStateAndInput(const Eigen::Vector4d& state, const double& start_time);
    void setInitStateAndInput(const double& t, Eigen::Vector4d& replan_init_state);
    void setSwarmTrajs(const swarm_bridge::Trajectory& traj_msg);
    void setMapFree(double& time_now);

    std::vector<Eigen::MatrixXd> display_hPolys() const { return display_hPolys_; }
    // plan_utils::SurroundTrajData display_surround_trajs() const {return surround_trajs;};

    std::shared_ptr<plan_utils::SingulTrajData> trajectory() const {
      return std::shared_ptr<plan_utils::SingulTrajData>(
        new plan_utils::SingulTrajData(traj_container_.singul_traj));
    }
    std::shared_ptr<plan_utils::KinoTrajData> display_kino_path() const {
      return std::shared_ptr<plan_utils::KinoTrajData>(
        new plan_utils::KinoTrajData(kino_trajs_)); 
    }

    // const plan_utils::SingulTrajData* get_trajectory()
    // {
    //   // return &traj_container_.singul_traj;
    //   // const plan_utils::SingulTrajData* const singul_traj = &traj_container_.singul_traj;
    //   return &traj_container_.singul_traj;
    // }
    
    
    void setParkingEnd(Eigen::Vector4d end_pt){
      end_pt = end_pt_;
      have_parking_target_ = true;
    }

    plan_utils::TrajContainer traj_container_;
    double traj_piece_duration_;
    int traj_res_, dense_traj_res_;

    bool checkCollisionWithObs(const double& t_now);
    bool checkCollisionWithOtherCars(const double& t_now);

    bool getKinoPath(Eigen::Vector4d &end_state, bool first_search);
    void displayKinoPath(std::shared_ptr<plan_utils::KinoTrajData> kino_trajs);
    void displayPolyH(const std::vector<Eigen::MatrixXd> hPolys);
    void displayMincoTraj(std::shared_ptr<plan_utils::SingulTrajData> display_traj);


private:

    void ReadConfig(const std::string config_path);
    /**
     * @brief transform all the states in a batch
     */

    int continous_failures_count_{0};
    int car_id_;

    //_____________________________________
    /* kinodynamic a star path and parking*/
    std::unique_ptr<path_searching::KinoAstar> kino_path_finder_;
    plan_utils::KinoTrajData kino_trajs_;
    // void getKinoPath(Eigen::Vector4d &end_state);
    bool enable_urban_ = false;


    plan_manage::PolyTrajOptimizer::Ptr ploy_traj_opt_;
    std::vector<plan_utils::TrajContainer> swarm_traj_container_;
    std::vector<plan_utils::TrajContainer> swarm_last_traj_container_;
    std::vector<bool> have_received_trajs_;


    // vehicle state
    // State head_state_;
    Eigen::Vector4d start_state_;
    Eigen::Vector2d start_ctrl_;
    bool has_init_state_ = false;
    bool is_init;
    int cars_num_;
    double car_d_cr_, car_length_, car_width_, car_wheelbase_;

    //_____________
    /* Map related */
    std::vector<Eigen::MatrixXd> hPolys_, display_hPolys_;   


    Eigen::MatrixXd display_InnterPs_, display_OptimalPs_;
 
    //using rectangles to represent corridor
    void getRectangleConst(std::vector<Eigen::Vector3d> statelist);

    ros::Publisher KinopathPub_;
    ros::Publisher Rectangle_poly_pub_;
    ros::Publisher minco_traj_pub_;
    ros::Publisher MincoPathPub_;
    ros::Publisher TrajPathPub_;
    ros::Publisher wholebody_traj_pub_;
    /*for benchmark*/
    
    ros::Publisher KinopathPub;

    /*debug*/
    ros::Publisher Debugtraj0Pub;
    ros::Publisher Debugtraj1Pub;
    ros::Publisher DebugCorridorPub;
    ros::Publisher DebugtrajPub;
    /*vis dense hybridastar*/
    ros::Publisher DenseKinopathPub;
    
    /*if parking*/
    bool have_parking_target_ = false;
    Eigen::Vector4d end_pt_;
    double start_time_;
    /*vehicle param*/



};



#endif
