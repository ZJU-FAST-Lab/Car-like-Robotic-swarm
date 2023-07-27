#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>
#include <math.h>
#include <utility> 

#include "mapping.h"
#include "path_searching/raycast.h"

#include "plan_utils/traj_container.hpp"
#include <pcl/point_cloud.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/point_types.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

using namespace std;

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

namespace path_searching{


class PathNode {
public:
    /* -------------------- */
    Eigen::Vector2i index;
    int yaw_idx;
    /* --- the state is x y theta(orientation) */
    Eigen::Vector3d state;
    double g_score, f_score;
    /* control input should be steer and arc */
    Eigen::Vector2d input;
    PathNode* parent;
    char node_state;
    int singul = 0;
    int number;
    /* -------------------- */
    PathNode() {
      parent = NULL;
      node_state = NOT_EXPAND;
    }
    ~PathNode(){};
};
typedef PathNode* PathNodePtr;

class MiddleNode{
public:
    double start_s;
    double start_t;
    double start_vel;
    double end_s;
    double end_t;
    double end_vel;
    Eigen::Vector3i s_t_v_idx;
    double length;
    double distance2end;
    double acc;
    double g_score, f_score;
    char node_state;

    MiddleNode* parent;

    MiddleNode()
    {
        parent = NULL;
        node_state = NOT_EXPAND;
    }
    ~MiddleNode(){};
};
typedef MiddleNode* MiddleNodePtr;


class NodeComparator {
public:
    template <class NodePtr>
    bool operator()(NodePtr node1, NodePtr node2) {
      return node1->f_score > node2->f_score;
    }
};

  template <typename T>
  struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
      size_t seed = 0;
      for (long int i = 0; i < matrix.size(); ++i) {
        auto elem = *(matrix.data() + i);
        seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }
  };

  template <class NodePtr>
  class NodeHashTable {
  private:
    /* data */

    std::unordered_map<Eigen::Vector2i, NodePtr, matrix_hash<Eigen::Vector2i>> data_2d_;
    std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;

  public:
    NodeHashTable(/* args */) {
    }
    ~NodeHashTable() {
    }
    // : for 2d vehicle planning
    void insert(Eigen::Vector2i idx, NodePtr node) {
      data_2d_.insert(std::make_pair(idx, node));
    }
    //for 3d vehicle planning
    void insert(Eigen::Vector2i idx, int yaw_idx, NodePtr node) {
      data_3d_.insert(std::make_pair(Eigen::Vector3i(idx(0), idx(1), yaw_idx), node));
    }
    void insert(Eigen::Vector3i idx,NodePtr node ){
      data_3d_.insert(std::make_pair(idx,node));
    }

    NodePtr find(Eigen::Vector2i idx) {
      auto iter = data_2d_.find(idx);
      return iter == data_2d_.end() ? NULL : iter->second;
    }
    NodePtr find(Eigen::Vector2i idx, int yaw_idx) {
      auto iter = data_3d_.find(Eigen::Vector3i(idx(0), idx(1), yaw_idx));
      return iter == data_3d_.end() ? NULL : iter->second;
    }

    void clear() {
      data_2d_.clear();
      data_3d_.clear();
    }
  };

  template <class MiddleNode>
  class MiddleNodeHashTable {
  private:
    /* data */
    std::unordered_map<Eigen::Vector3i, MiddleNodePtr, matrix_hash<Eigen::Vector3i>> middle_data_3d_;

  public:
    MiddleNodeHashTable(/* args */) {
    }
    ~MiddleNodeHashTable() {
    }

    // For middle node searching
    void insert(Eigen::Vector3i idx, MiddleNodePtr node)
    {
      middle_data_3d_.insert(std::make_pair(idx, node));
    }

    MiddleNodePtr find(Eigen::Vector3i idx)
    {
        auto iter = middle_data_3d_.find(idx);
        return iter == middle_data_3d_.end() ? NULL : iter->second;
    }

    void clear() {
      middle_data_3d_.clear();
    }
  };  


  class KinoAstar {
  
  private:
    /* ---------- main data structure ---------- */
    vector<PathNodePtr> path_node_pool_;
    int use_node_num_, iter_num_;
    NodeHashTable<PathNodePtr> expanded_nodes_;
    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;
    std::vector<PathNodePtr> path_nodes_;
    std::vector<Eigen::Vector2d> last_path_pos_, last_path_pos_temp_;

    /*--------------speed planning-----------------*/
    MiddleNodeHashTable<MiddleNodePtr> expanded_middle_nodes_;
    double calculateMiniTime2singulPoint(double initspeed, double distance2end);
    int s2idx(double s);
    int t2idx(double t);
    int v2idx(double v);

    std::vector<MiddleNodePtr> middle_node_pool_;
    std::vector<MiddleNodePtr> middle_nodes_;
    int use_time_node_num_;
    double total_s_;
    std::multimap<double, MiddleNodePtr> openSet_;
    double gettimeHeu(int& idx, double& vel);
    double calculateCurNode2nearestSingul(int idx);
    /*------------------------------------------*/

    /*-------for speed planning and dynamic avoidance-----*/
    bool ifdynamic_;
    std::vector<bool> have_received_trajs_;
    std::vector<plan_utils::TrajContainer> swarm_traj_container_;
    std::vector<plan_utils::TrajContainer> swarm_last_traj_container_;
    /*-------------------------------------------------*/

    /* ---------- record data ---------- */
    Eigen::Vector4d start_state_, end_state_;
    Eigen::Vector2d start_ctrl_;
    bool is_shot_succ_ = false;
    bool has_path_ = false;
    /* ---------- parameter ---------- */
    /* search */
    double step_arc = 1.0;
    double max_vel_ = 10.0;
    double max_acc_ = 3.0;
    double min_vel_ = -1.0;
    double min_acc_ = -1.0;
    double max_cur_ = 0.35;
    double max_steer_ = 0.78539815;
    double max_seach_time = 0.1;
    double traj_forward_penalty = 1.0;
    double traj_back_penalty = 1.0;
    double traj_gear_switch_penalty = 10.0;
    double traj_steer_penalty = 0.0;
    double traj_steer_change_penalty = 0.0;
    double horizon_;
    double lambda_heu_;

    double time_resolution_ = 0.1;
    double distance_resolution_ = 0.5;
    double velocity_resolution_ = 0.5;

    //double margin_;
    int allocate_num_;
    int check_num_;
    double tie_breaker_ = 1.0 + 1.0 / 10000; 
    int nearest_idx_;
    double nearest_init_distance_;

    /* vehicle parameters*/
    double car_width_, car_length_, car_wheelbase_, car_front_suspension_, 
           car_rear_suspension_, car_max_steering_angle_, car_d_cr_;

    int cars_num_, car_id_;
    /* map */
    double resolution_, inv_resolution_, yaw_resolution_, inv_yaw_resolution_;
    Eigen::Vector2d origin_, map_size_3d_;
    double yaw_origin_ = -M_PI;
    /*shot hzc*/
    std::vector<double>  shot_timeList;
    std::vector<double>  shot_lengthList;
    std::vector<int>     shotindex;
    std::vector<int>     shot_SList; 
    std::vector<Eigen::Vector3d> SampleTraj;
    std::vector<Eigen::Vector2d> positionList_;
    std::vector<int> direction_change_idx_;

    /* helper */
    Eigen::Vector2i posToIndex(Eigen::Vector2d pt);
    int yawToIndex(double& yaw);
    double normalize_angle(const double& theta);
    void retrievePath(PathNodePtr end_node);

    /* shot trajectory */
    vector<double> cubic(double a, double b, double c, double d);
    vector<double> quartic(double a, double b, double c, double d, double e);
    double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);


    void getFlatState(Eigen::Vector4d state, Eigen::Vector2d control_input,
                      Eigen::MatrixXd &flat_state, int singul);

    bool is_shot_sucess(Eigen::Vector3d state1,Eigen::Vector3d state2);
    double computeShotTraj(Eigen::Vector3d &state1, Eigen::Vector3d &state2,
                          std::vector<Eigen::Vector3d> &path_list,
                          double& len); 
    void stateTransit(Eigen::Vector3d &state0,  Eigen::Vector3d &state1,
              Eigen::Vector2d &ctrl_input);

    double evaluateLength(double curt,double locallength,double localtime, double startV = 0.0, double endV = 0.0);
    double evaluateDuration(double length, double startV = 0.0, double endV = 0.0);
    void resetEndState(Eigen::Vector4d& start_state, Eigen::Vector4d& end_state);

    double non_siguav;
    ompl::base::StateSpacePtr shotptr;

    inline int getSingularity(double vel)
    {
      int singul = 0;
      if (fabs(vel) > 1e-2){
        if (vel >= 0.0){singul = 1;} 
        else{singul = -1;}      
      }
      
      return singul;
    }
    // getHeu
    inline double getHeu(Eigen::VectorXd x1, Eigen::VectorXd x2)
    {
      double dx = abs(x1(0) - x2(0));
      double dy = abs(x1(1) - x2(1));
      return tie_breaker_* sqrt(dx * dx + dy * dy);
    }

  public:
    KinoAstar(){};
    ~KinoAstar();
    ros::NodeHandle nh_;
    MappingProcess::Ptr map_ptr_;
    std::vector<Eigen::Vector2d> car_vertex_small_, car_vertex_, car_vertex_big_;

    enum { REACH_HORIZON = 1, REACH_END = 2,  NO_PATH = 3, REACH_END_BUT_SHOT_FAILS = 4};

    /* main API */
    void setParam(ros::NodeHandle& nh);
    // void init(planning::minco::Config cfg_, ros::NodeHandle nh);
    void init(ros::NodeHandle& nh);

    void reset();
    void findNearestNode(Eigen::Vector2d& start_pos, bool first_search);
    int search(Eigen::Vector4d start_state, Eigen::Vector2d init_ctrl,
              Eigen::Vector4d end_state, bool use3d = false);
    bool searchTime(plan_utils::KinoTrajData &flat_trajs, double &start_world_time);
    void getTruncatedposLists();
    void getSingulNodes();
    Eigen::Vector3d CalculateInitPos(double& t, int& singul);
    // inital semantic map
    // void intialMap(map_utils::TrajPlannerMapItf *map_itf);
    void setMap(MappingProcess::Ptr& ptr);
    void setFreeSpaces(std::vector<Eigen::Vector2d>& pos_vec, std::vector<double>& yaw_vec);
    void setAllCarsTrajs(plan_utils::TrajContainer& trajectory, int& car_id);
    void setAllCarsLastTrajs(plan_utils::TrajContainer& trajectory, int& car_id);
    // get kino traj for optimization  
    void getKinoNode(plan_utils::KinoTrajData &flat_trajs);
    void NodeVis(Eigen::Vector3d state);
    void checkCollisionUsingPosAndYaw(const Eigen::Vector3d &state, bool& res);
    void checkCollisionUsingLine(const Eigen::Vector2d &start_pt, const Eigen::Vector2d &end_pt, bool &res);    

    /*hzchzc*/
    Eigen::Vector3d evaluatePos(double t);
    std::vector<Eigen::Vector4d> SamplePosList(int N); //px py yaw t 
    std::vector<Eigen::Vector3d> getSampleTraj();
    double totalTrajTime;
    double checkl = 0.2;

    ros::Publisher expandNodesVis;



    std::vector<PathNodePtr> getVisitedNodes();

    std::vector<Eigen::Vector4d>  state_list;
    std::vector<Eigen::Vector3d> acc_list;

    typedef shared_ptr<KinoAstar> Ptr;


  };

}

#endif
