#ifndef _TRAJ_OPTIMIZER_H_
#define _TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <chrono>

#include <plan_utils/traj_container.hpp>

#include "geo_utils2d/lbfgs.hpp"
#include "geo_utils2d/geoutils2d.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>


namespace plan_manage
{

  using namespace std;


  class PolyTrajOptimizer
  {

  private:


    ros::NodeHandle nh_;
    plan_utils::SurroundTrajData *surround_trajs_{NULL}; // Can not use shared_ptr and no need to free
    std::vector<plan_utils::TrajContainer> swarm_traj_container_;
    std::vector<plan_utils::TrajContainer> swarm_last_traj_container_;
    bool ifdynamic_;
    int traj_seg_num_;

    int drone_id_;
    int car_id_, cars_num_;
    int traj_resolution_; // number of distinctive constrain points each piece
    int destraj_resolution_; //number of distinctive constrain points of the first and last piece (should be more dense!
    int variable_num_;     // optimization variables
    int iter_num_;         // iteration of the solver
    double min_ellip_dist2_; // min trajectory distance in surround

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_;                         // obstacle weight
    double wei_surround_;                       // surround weight
    double wei_feas_;                        // feasibility weight
    double wei_sqrvar_;                      // squared variance weight
    double wei_time_;                        // time weight
    double surround_clearance_; // safe distance
    double max_vel_, max_acc_, max_cur_;       // dynamic limits
    double min_vel_, min_acc_;
    double half_margin;                        // safe margin
    // common::VehicleParam veh_param_;    
    double car_length_, car_width_, car_d_cr_, car_wheelbase_;


    double t_now_;
    double delta_t_;
    double L_, max_phidot_;
    std::vector<Eigen::Vector2d> lz_set_;
    std::vector<Eigen::Vector2d> vec_le_, vec_lo_;
    int number_of_hyperplanes_of_ego_car_, number_of_hyperplanes_of_surround_car_;
    std::vector<bool> have_received_trajs_;
    // Each col of cfgHs denotes a facet (outter_normal^T,point^T)^T
    Eigen::Matrix<double, 2, 2> B_h;
    double epis;
    std::vector<int> singul_container;
    std::vector<Eigen::MatrixXd> iniState_container;
    std::vector<Eigen::MatrixXd> finState_container;
    std::vector<std::vector<Eigen::MatrixXd>> cfgHs_container;
    int trajnum;
    /*debug*/
    Eigen::MatrixXd ctrl_points_;
    std::vector<Eigen::Vector2d> cos_points;
    std::vector<Eigen::Vector2d> key_points;
    std::vector<Eigen::MatrixXd> debug_hPolys;
    std::vector<plan_utils::MinJerkOpt> jerkOpt_container;
    std::vector<int> piece_num_container;

  public:
    
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    /* set variables */
    void init(ros::NodeHandle &nh);
    ros::Publisher debug_pub,debug_pub1,debug_galaxy_poly_pub_,debug_key_points_pub;
    void displayPoints();
    void displayCosPoints();
    void displayBugPoly();
    void displayKeyPoints();
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSurroundTrajs(plan_utils::SurroundTrajData *surround_trajs_ptr);
    void setSwarmTrajs(std::vector<plan_utils::TrajContainer> &swarm_traj_container, bool ifdynamic);
    void setAllCarsTrajs(plan_utils::TrajContainer& trajectory, int& car_id);
    void setAllCarsLastTrajs(plan_utils::TrajContainer& trajectory, int& car_id);
    bool checkCollisionWithSurroundCars(const double& time);

    /* helper functions */
    inline const std::vector<plan_utils::MinJerkOpt> *getMinJerkOptPtr(void) { return &jerkOpt_container; }
    inline int get_traj_resolution_() { return traj_resolution_; };
    inline int get_destraj_resolution_() { return destraj_resolution_; };
    inline double getsurroundClearance(void) { return surround_clearance_; }

    /* main planning API */
    bool OptimizeTrajectory(const std::vector<Eigen::MatrixXd> &iniStates, const std::vector<Eigen::MatrixXd> &finStates,
                            std::vector<Eigen::MatrixXd> &initInnerPts, const Eigen::VectorXd &initTs,
                            std::vector<std::vector<Eigen::MatrixXd>> &hPoly_container,std::vector<int> singuls,double now = ros::Time::now().toSec(),double help_eps = 1.0e-4);


    double log_sum_exp(double alpha, Eigen::VectorXd &all_dists, double &exp_sum);


  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback(void *func_data, const Eigen::VectorXd &x, Eigen::VectorXd &grad);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);
    void VirtualTGradCost(const double &RT, const double & VT, const double & gdRT, double & gdVT, double & costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost, int trajid);
    //std::vector<Eigen::VectorXd> 
    void addPVAGradCost2CT(std::vector<Eigen::VectorXd>  &gdTs, Eigen::VectorXd &costs, const int trajid, const double trajtime);

    bool obstacleGradCostP(const int i_dp,
                           const Eigen::Vector2d &p,
                           Eigen::Vector2d &gradp,
                           double &costp);

    bool surroundGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector2d &p,
                        const Eigen::Vector2d &v,
                        Eigen::Vector2d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    double dynamicObsGradCostP( const double &omg_j,
                                const int &time_int_pena,
                                const double &t,               // current absolute time
                                const Eigen::Matrix<double, 6, 1> &beta0,
                                const Eigen::Matrix<double, 6, 1> &beta1,
                                const int& pieceid,
                                const int& K,
                                const Eigen::Vector2d &sigma, // the rear model
                                const Eigen::Vector2d &dsigma,
                                const Eigen::Vector2d &ddsigma,
                                const Eigen::Matrix2d &ego_R,
                                const Eigen::Matrix2d &help_R,
                                std::vector<Eigen::VectorXd>  &gdTs,
                                const int &trajid, const double& trajtime);
    double debugGradCheck(const int i_dp, // index of constraint point
                                         double t, // current absolute time
                                        Eigen::Vector2d sigma, // the rear model 
                                        Eigen::Vector2d dsigma,
                                        Eigen::Vector2d ddsigma,                                                                             
                                        const int trajid, const int sur_id, double res_t,Eigen::Matrix<double, 6, 2> c
                                        ,int i ,int j,double omg,double step,double wei_surround_,int K);
    bool dynamicObsCosCheck(double t_now, const Eigen::MatrixXd iniStates,  int trajid, int sur_id);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    inline bool extractVs(const std::vector<Eigen::MatrixXd> &hPs,
                          std::vector<Eigen::MatrixXd> &vPs) const
    {
        const int M = hPs.size() - 1;

        vPs.clear();
        vPs.reserve(2 * M + 1);

        int nv;
        Eigen::MatrixXd curIH, curIV, curIOB;
        for (int i = 0; i < M; i++)
        {
            if (!geoutils::enumerateVs(hPs[i], curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            curIH.resize(6, hPs[i].cols() + hPs[i + 1].cols());
            curIH << hPs[i], hPs[i + 1];
            if (!geoutils::enumerateVs(curIH, curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);
        }

        if (!geoutils::enumerateVs(hPs.back(), curIV))
        {
            return false;
        }
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        vPs.push_back(curIOB);

        return true;
    }

    std::vector<Eigen::MatrixXd> debug_P;
    std::vector<Eigen::VectorXd> debug_T;
    std::vector<double> debug_cost;

    void updatePT(const Eigen::MatrixXd &inPs,const Eigen::VectorXd &ts, double cost);
    void getBoundPts(Eigen::Vector2d &position, double angle, std::vector<Eigen::Vector2d> &BoundVertices);
    void positiveSmoothedL1(const double &x, double &f, double &df);
    void positiveSmoothedL3(const double &x, double &f, double &df);

  public:
    typedef unique_ptr<PolyTrajOptimizer> Ptr;

  };

} // namespace plan_manage
#endif