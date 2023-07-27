#include "plan_manage/traj_optimizer.h"
// using namespace std;

namespace plan_manage
{
  /* main planning API */
  bool PolyTrajOptimizer::OptimizeTrajectory(
      const std::vector<Eigen::MatrixXd> &iniStates, const std::vector<Eigen::MatrixXd> &finStates,
      std::vector<Eigen::MatrixXd> &initInnerPts, const Eigen::VectorXd &initTs,
      std::vector<std::vector<Eigen::MatrixXd>> &hPoly_container,std::vector<int> singuls,double now, double help_eps)
  {
    trajnum = initInnerPts.size();
    epis = help_eps;
    cfgHs_container = hPoly_container;
    iniState_container = iniStates;
    finState_container = finStates;
    singul_container = singuls;
    variable_num_ = 0;
    jerkOpt_container.clear();
    piece_num_container.clear();
    jerkOpt_container.resize(trajnum);
    piece_num_container.resize(trajnum);
    double final_cost;

    if(initTs.size()!=trajnum){
      ROS_ERROR("initTs.size()!=trajnum");
      return false;
    }

    

    for(int i = 0; i < trajnum; i++){
      //check
      if(initInnerPts[i].cols()==0){
        ROS_ERROR("There is only a piece?");
        return false;
      }
      int piece_num_ = initInnerPts[i].cols() + 1;
      piece_num_container[i] = piece_num_;

      if(cfgHs_container[i].size()!=(piece_num_ - 2) * (traj_resolution_ + 1) + 2 * (destraj_resolution_ + 1)){
        std::cout<<"cfgHs size: "<<cfgHs_container[i].size()<<std::endl;
        ROS_ERROR("cfgHs size error!");
        return false;
      }
      for (int k = 0; k < (piece_num_ - 2) * (traj_resolution_ + 1) + 2 * (destraj_resolution_ + 1); k++)
      {
        cfgHs_container[i][k].topRows<2>().colwise().normalize(); // norm vector outside
      }

      //reset the start end max_vel_
      if(iniState_container[i].col(1).norm()>=max_vel_){
        iniState_container[i].col(1) = iniState_container[i].col(1).normalized()*(max_vel_-1.0e-2);
      }
      if(iniState_container[i].col(2).norm()>=max_acc_){
        iniState_container[i].col(2) = iniState_container[i].col(2).normalized()*(max_acc_-1.0e-2);
      }
      if(finState_container[i].col(1).norm()>=max_vel_){
        finState_container[i].col(1) = finState_container[i].col(1).normalized()*(max_vel_-1.0e-2);
      }
      if(finState_container[i].col(2).norm()>=max_acc_){
        finState_container[i].col(2) = finState_container[i].col(2).normalized()*(max_acc_-1.0e-2);
      }
      jerkOpt_container[i].reset(iniState_container[i], finState_container[i], piece_num_);
      variable_num_ += 2 * (piece_num_ - 1);


    }  
    variable_num_ += trajnum;
    

    // ros::Time t0 = ros::Time::now();
    auto t0 = std::chrono::high_resolution_clock::now();
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_still_occ, flag_success;
    Eigen::VectorXd x;
    x.resize(variable_num_);
    int offset = 0;
    for(int i = 0; i<trajnum; i++){
      memcpy(x.data()+offset,initInnerPts[i].data(), initInnerPts[i].size() * sizeof(x[0]));
      offset += initInnerPts[i].size();
    }
    Eigen::Map<Eigen::VectorXd> Vt(x.data()+offset, initTs.size());
    RealT2VirtualT(initTs, Vt);



    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 64;//128
    lbfgs_params.past = 3; //3 
    lbfgs_params.g_epsilon = 0.0;
    // lbfgs_params.max_linesearch = 200;
    lbfgs_params.min_step = 1.0e-12;
    lbfgs_params.delta = 1.0e-4;
    lbfgs_params.max_iterations = 1000;
    t_now_ = now;



    /* ---------- prepare ---------- */
    iter_num_ = 0;
    flag_force_return = false;
    force_stop_type_ = DONT_STOP;
    flag_still_occ = false;
    flag_success = false;
    /* ---------- optimize ---------- */
    // t1 = ros::Time::now();
    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout<<"begin to optimize!\n";
    int result = lbfgs::lbfgs_optimize(
        x,
        final_cost,
        PolyTrajOptimizer::costFunctionCallback,
        NULL,
        NULL,
        this,
        lbfgs_params);
    // t2 = ros::Time::now();
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_ms = t2 - t1;
    std::chrono::duration<double, std::milli> total_time_ms = t2 - t0;
    // double time_ms = (t2 - t1).toSec() * 1000;
    // double total_time_ms = (t2 - t0).toSec() * 1000;
    /* ---------- get result and check collision ---------- */
    if (result == lbfgs::LBFGS_CONVERGENCE ||
        result == lbfgs::LBFGS_CANCELED ||
        result == lbfgs::LBFGS_STOP||result == lbfgs::LBFGSERR_MAXIMUMITERATION)
    {
      flag_force_return = false;
      flag_success = true;
      printf("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms.count(), total_time_ms.count(), final_cost);

    } 
    else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
      printf("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms.count(), total_time_ms.count(), final_cost);
      ROS_WARN("Lbfgs: The line-search routine reaches the maximum number of evaluations.");
      flag_force_return = false;
      flag_success = true;
    }
    else
    {
      printf("\033[31m[PolyTrajOptimizer]iter=%d,time(ms)=%5.3f, error.\n\033[0m", iter_num_, time_ms.count());
      ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
    }

    
    // initInnerPts  = ctrl_points_;
    // ros::shutdown();
    // if(final_cost>=50000.0){
    //   ROS_ERROR("optimization fails! cost is too high!");
    //   flag_success = false;
    // }
    return flag_success;
  }


  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const Eigen::VectorXd &x, Eigen::VectorXd &grad)
  { 

    double total_smcost = 0.0, total_timecost = 0.0, penalty_cost = 0.0;
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    int offset = 0;
    
    std::vector<Eigen::Map<const Eigen::MatrixXd>> P_container;
    std::vector<Eigen::Map<Eigen::MatrixXd>> gradP_container;
    std::vector<Eigen::VectorXd> arrayt_container;
    std::vector<Eigen::VectorXd> arraygradt_container;
    std::vector<Eigen::VectorXd> arraygradT_container;
    std::vector<Eigen::VectorXd> arrayT_container;
    std::vector<double> trajtimes; trajtimes.push_back(0.0);
    /*
        int offset = 0;
    for(int i = 0; i<trajnum; i++){
      memcpy(x.data()+offset,initInnerPts[i].data(), initInnerPts[i].size() * sizeof(x[0]));
      offset += initInnerPts[i].size();
    }
    Eigen::Map<Eigen::VectorXd> Vt(x.data()+offset, initTs.size());
    RealT2VirtualT(initTs, Vt);
    */
    for(int trajid = 0; trajid < opt->trajnum; trajid++){
      Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, opt->piece_num_container[trajid] - 1);
      Eigen::Map<Eigen::MatrixXd>gradP(grad.data()+offset, 2, opt->piece_num_container[trajid] - 1);
      offset += 2 * (opt->piece_num_container[trajid] - 1);
      gradP.setZero();
      P_container.push_back(P);
      gradP_container.push_back(gradP);
    }
    Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, opt->trajnum);
    Eigen::Map<Eigen::VectorXd>gradt(grad.data()+offset, opt->trajnum);
    Eigen::VectorXd T(opt->trajnum);
    opt->VirtualT2RealT(t, T);
    //T is sum time

    for(int trajid = 0; trajid < opt->trajnum; trajid++){
      Eigen::VectorXd arraygradT(opt->piece_num_container[trajid]);
      Eigen::VectorXd arrayT(opt->piece_num_container[trajid]);
      arrayT.setConstant(T[trajid]/opt->piece_num_container[trajid]);
      arraygradT.setZero();
      arrayT_container.push_back(arrayT);
      arraygradT_container.push_back(arraygradT);
      trajtimes.push_back(T[trajid]);
    }


    if(T.sum() > 1000 || T.sum() < 0.1)
    {
      for(int trajid = 0; trajid < opt->trajnum; trajid++)
      {
        gradP_container[trajid].setZero();
      }
      // gradP.setZero();
      gradt.setZero();
      return 999999.0;
    }
          

    double smoothness_cost, time_of_cost, collision_penalty, dynamic_penalty, feasibility_penalty;

    // Eigen::VectorXd gradt;
    for(int trajid = 0; trajid < opt->trajnum; trajid++){
      double smoo_cost = 0;
      Eigen::VectorXd obs_surround_feas_qvar_costs(3);
      obs_surround_feas_qvar_costs.setZero();
      opt->jerkOpt_container[trajid].generate(P_container[trajid], arrayT_container[trajid]);
      opt->initAndGetSmoothnessGradCost2PT(arraygradT_container[trajid], smoo_cost, trajid); // Smoothness cost   
      opt->addPVAGradCost2CT(arraygradT_container, obs_surround_feas_qvar_costs, trajid, trajtimes[trajid]); // Time int cost
      //Get gradT gradC
      total_smcost += smoo_cost;
      penalty_cost += obs_surround_feas_qvar_costs.sum();
      
      
      smoothness_cost = total_smcost;
      collision_penalty = obs_surround_feas_qvar_costs(0);
      dynamic_penalty = obs_surround_feas_qvar_costs(1);
      feasibility_penalty = obs_surround_feas_qvar_costs(2);
      // std::cout<<"Trajid: "<<trajid<<" penalty: "<<obs_surround_feas_qvar_costs.transpose()<<std::endl;
    }
    for(int trajid = 0; trajid < opt->trajnum; trajid++){
      double time_cost = 0.0;
      opt->jerkOpt_container[trajid].getGrad2TP(arraygradT_container[trajid], gradP_container[trajid]); // gradC->gradT gradC->gradP
      // double gT = gradT_container[trajid].sum() / gradT_container[trajid].size();// 
      // gradT_container[trajid].setConstant(gT);
      //VirtualTGradCost
      // opt->VirtualTGradCost(arrayT_container[trajid], arrayt_container[trajid], arraygradT_container[trajid], arraygradt_container[trajid], time_cost);
      // gradt[trajid] = arraygradt_container[trajid].sum();
      // std::cout<<"trajid: "<<trajid<<" grad: "<<gradt_container[trajid].transpose()<<std::endl;
      double gradsumT,gradsumt;
      gradsumT = arraygradT_container[trajid].sum() / arraygradT_container[trajid].size();
      opt->VirtualTGradCost(T[trajid],t[trajid],gradsumT,gradsumt,time_cost);
      gradt[trajid] = gradsumt;
      total_timecost += time_cost;

      time_of_cost = total_timecost;
    }

    opt->iter_num_ += 1;

    // std::cout << "sm_cost: " << smoothness_cost << " time_cost: " << time_of_cost << " colli_pen: " << collision_penalty << " dyn_pen: " << dynamic_penalty << "feas_pen: " << feasibility_penalty << std::endl;
    
    return total_smcost + total_timecost + penalty_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }
  void PolyTrajOptimizer::VirtualTGradCost(const double &RT, const double &VT, const double &gdRT, double &gdVT, double& costT){
    double gdVT2Rt;
    if (VT > 0)
    {
      gdVT2Rt = VT + 1.0;
    }
    else
    {
      double denSqrt = (0.5 * VT - 1.0) * VT + 1.0;
      gdVT2Rt = (1.0 - VT) / (denSqrt * denSqrt);
    }

    gdVT = (gdRT + wei_time_) * gdVT2Rt;
    costT = RT * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost, int trajid)
  {
    jerkOpt_container[trajid].initGradCost(gdT, cost);
  }

  void PolyTrajOptimizer::addPVAGradCost2CT(std::vector<Eigen::VectorXd>  &gdTs, Eigen::VectorXd &costs,  const int trajid, const double trajtime)
  {
    int M = gdTs[trajid].size();                         // number of pieces
    double T = jerkOpt_container[trajid].get_T1().sum(); // total duration of the trajectory
    double delta_T = T / M;                              // time duration of one piece
    int K = std::floor(T / delta_t_);                    // number of constrain points
    // cout << "number of pieces: " << M << " number of constrain points: " << K << " total time: " << T << endl;

    // Eigen::Vector2d outerNormal;
    Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma, ddddsigma;
    double z_h0, z_h1, z_h2, z_h3, z_h4;    
    double vel2_reci, vel2_reci_e, vel3_2_reci_e, acc2, cur2, cur;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;

    Eigen::Matrix<double, 6, 2> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaKc, gradViolaKLc, gradViolaKRc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaKt, gradViolaKLt, gradViolaKRt;
    double violaPos, violaVel, violaAcc, violaCur, violaCurL, violaCurR, violaDynamicObs;
    double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaCurPenaD, violaCurPenaDL, violaCurPenaDR, violaDynamicObsPenaD;
    double violaPosPena, violaVelPena, violaAccPena, violaCurPena, violaCurPenaL, violaCurPenaR, violaDynamicObsPena;

    // std::vector<Eigen::MatrixXd> cfgHs = cfgHs_container[trajid];
    int singul_ = singul_container[trajid];

    double omg_j;
    costs.setZero();
    Eigen::Matrix2d ego_R, help_R;


    /*----------------fixed time sampling strategy-----------------------*/
    /*----------constrain on feasibility and dynamic avoidance-----------*/

    for(int j = 0; j <= K + 1; j++) // iterate over all constrain points
    {
      double constrain_pt_t = (j == K + 1) ? T : j * delta_t_;

      //--- locate piece id: i ---
      int i = 0;
      for(i = 0; i < M; i++)
      {
        if(constrain_pt_t > delta_T * i - 1e-3 && constrain_pt_t < delta_T * (i + 1) + 1e-3)
          break;
      }
      /**************************/
      int time_int_pena = (j == K + 1) ? 1 : (-i);
      // int time_int_pena = -i;

      double t_bar = constrain_pt_t - i * delta_T;
      const Eigen::Matrix<double, 6, 2> &c = jerkOpt_container[trajid].get_b().block<6, 2>(i * 6, 0);
      
      s1 = t_bar;
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;

      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
      beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1;

      sigma = c.transpose() * beta0;
      dsigma = c.transpose() * beta1;
      ddsigma = c.transpose() * beta2;
      dddsigma = c.transpose() * beta3;
      ddddsigma = c.transpose() * beta4;
      
      if(j == 0)
        omg_j = 0.5;
      else if(j == K)
      {
        omg_j = 0.5 * (T / delta_t_ - K + 1);
      }
      else if(j == K + 1)
      {
        omg_j = 0.5 * (T / delta_t_ - K);
      }
        // omg_j = T / delta_t_ - K - 0.5;
      // else if(K - j <= 3)
      //   omg_j = 0.0;
      else
        omg_j = 1.0;

      z_h0 = dsigma.norm();
      z_h1 = ddsigma.transpose() * dsigma;
      z_h2 = dddsigma.transpose() * dsigma;
      z_h3 = ddsigma.transpose() * B_h * dsigma;

      if ( z_h0 < 1e-4 )
      {
        continue;
      }
      //avoid siguality

      vel2_reci = 1.0 / (z_h0 * z_h0);
      vel2_reci_e = 1.0 / (z_h0 * z_h0+epis);
      vel3_2_reci_e = vel2_reci_e * sqrt(vel2_reci_e);
      z_h0 = 1.0 / z_h0;
      z_h4 = z_h1 * vel2_reci;
      
      acc2 = z_h1 * z_h1 * vel2_reci;
      cur2 = z_h3 * z_h3 * (vel2_reci_e * vel2_reci_e * vel2_reci_e);
      cur = z_h3 * vel3_2_reci_e;

      // add feasibility with curvature
      violaCur = cur2 - max_cur_ * max_cur_;
      violaCurL = cur-max_cur_;
      violaCurR = -cur-max_cur_;

      ego_R << dsigma(0), -dsigma(1),
               dsigma(1), dsigma(0);
      ego_R = singul_ * ego_R * z_h0;

      Eigen::Matrix2d temp_a, temp_v;
      temp_a << ddsigma(0), -ddsigma(1),
                ddsigma(1), ddsigma(0);
      temp_v << dsigma(0), -dsigma(1),
                dsigma(1), dsigma(0);
      Eigen::Matrix2d R_dot = singul_ * (temp_a * z_h0 - temp_v * vel2_reci * z_h0 * z_h1);

      violaVel = 1.0 / vel2_reci - max_vel_ * max_vel_;
      violaAcc = acc2 - max_acc_ * max_acc_;

      if (violaVel > 0.0)
      {
        positiveSmoothedL1(violaVel, violaVelPena, violaVelPenaD);

        gradViolaVc = 2.0 * beta1 * dsigma.transpose(); // 6*2
        gradViolaVt = 2.0 * z_h1;                       // 1*1
        jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += delta_t_ * omg_j * wei_feas_ * violaVelPenaD * gradViolaVc;
        gdTs[trajid](i) += delta_t_ * omg_j * wei_feas_ * violaVelPenaD * gradViolaVt * /*(-i)*/time_int_pena;
        costs(2) += delta_t_ * omg_j * wei_feas_ * violaVelPena;

      }
        
      if (violaAcc > 0.0)
      {
        positiveSmoothedL1(violaAcc, violaAccPena, violaAccPenaD);

        gradViolaAc = 2.0 * beta1 * (z_h4 * ddsigma.transpose() - z_h4 * z_h4 * dsigma.transpose()) +
                      2.0 * beta2 * z_h4 * dsigma.transpose(); // 6*2
        gradViolaAt = 2.0 * (z_h4 * (ddsigma.squaredNorm() + z_h2) - z_h4 * z_h4 * z_h1);
        jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += delta_t_ * omg_j * wei_feas_ * violaAccPenaD * gradViolaAc;
        gdTs[trajid](i) += delta_t_ * omg_j * wei_feas_ * violaAccPenaD * gradViolaAt * /*(-i)*/time_int_pena;
        costs(2) += delta_t_ * omg_j * wei_feas_ * violaAccPena;
      }      
       

     /*******************************/
       
      /*violaCurL = cur-max_cur_;
      violaCurR = -cur-max_cur_;*/

      // if(violaCurL > 0.0)
      // {
      //   positiveSmoothedL1(violaCurL, violaCurPenaL, violaCurPenaDL);
          
      //   gradViolaKLc = beta1 * (vel3_2_reci_e * ddsigma.transpose()*B_h - 3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) +
      //                  beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose(); // 6*2
      //   gradViolaKLt = vel3_2_reci_e*(dddsigma.transpose()*B_h*dsigma-3*vel2_reci_e*z_h3*z_h1);
      //   jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += delta_t_ * omg_j * wei_feas_ * 10 * violaCurPenaDL * gradViolaKLc;
      //   gdTs[trajid](i) += delta_t_ * omg_j * wei_feas_ * 10 * violaCurPenaDL * gradViolaKLt * /*(-i)*/time_int_pena;
      //   costs(2) += delta_t_ * omg_j * wei_feas_ * 10 * violaCurPenaL;
      //   // if(j == K)
      //   //   cout << "the gradient of T: " << delta_t_ * omg_j * wei_feas_ * 10 * violaCurPenaDL * gradViolaKLt * /*(-i)*/time_int_pena << endl;
      // }
      // if(violaCurR > 0.0)
      // {
      //   positiveSmoothedL1(violaCurR, violaCurPenaR, violaCurPenaDR);
        
      //   gradViolaKRc = -(beta1 * (vel3_2_reci_e * ddsigma.transpose()*B_h - 3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) +
      //                    beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose()); // 6*2
      //   gradViolaKRt  = -vel3_2_reci_e*(dddsigma.transpose()*B_h*dsigma-3*vel2_reci_e*z_h3*z_h1);
      //   jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += delta_t_ * omg_j * wei_feas_ * 10 * violaCurPenaDR * gradViolaKRc;
      //   gdTs[trajid](i) += delta_t_ * omg_j * wei_feas_ * 10 * violaCurPenaDR * gradViolaKRt * /*(-i)*/time_int_pena;
      //   costs(2) += delta_t_ * omg_j * wei_feas_ * 10 * violaCurPenaR;
      //   // if(j == K)
      //   //   cout << "the gradient of T: " << delta_t_ * omg_j * wei_feas_ * 10 * violaCurPenaDL * gradViolaKLt * /*(-i)*/time_int_pena << endl;
      // }           

      if(ifdynamic_)
      {
        // costs(1) += dynamicObsGradCostP(omg,step,t + step * j,beta0,beta1,alpha,i,K,sigma,dsigma,ddsigma,ego_R,help_R,gdTs,trajid,trajtime);
        costs(1) += dynamicObsGradCostP(omg_j, time_int_pena, constrain_pt_t, beta0, beta1, i, K, sigma, dsigma, ddsigma, ego_R, help_R, gdTs, trajid, trajtime);
      }

    }
    /*-----------------------------------------------------------------------------------*/
    /*--------------------------fixed time sampling strategy-----------------------------*/



    /*-----------------------fixed number sampling strategy------------------------------*/
    /*------This part mainly constrains the curvature and static obstacle avoidance------*/

    // output gradT gradC 
    int N = gdTs[trajid].size();
    Eigen::Vector2d outerNormal;

    double step, alpha;

    double approxcur2, approxviolaCur,approxviolaCurPenaD,approxviolaCurPena;
    Eigen::Matrix<double, 6, 2> gradapproxViolaKc;
    double gradapproxViolaKt;
    
    std::vector<Eigen::MatrixXd> cfgHs = cfgHs_container[trajid];

    double omg;
    int i_dp = 0; // the index of constrain points
    // costs.setZero();
    // double z_h0, z_h1, z_h2, z_h3, z_h4;
    double t = 0;

    int pointid = -1;


    for (int i = 0; i < N; ++i)
    {
      int K;
      if(i==0 || i==N-1){
        K = destraj_resolution_;
      }
      else{
        K = traj_resolution_;
      }
      const Eigen::Matrix<double, 6, 2> &c = jerkOpt_container[trajid].get_b().block<6, 2>(i * 6, 0);
      step = jerkOpt_container[trajid].get_T1()(i) / K; // T_i /k
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1;
        alpha = 1.0 / K * j;
        
        //update s1 for the next iteration
        s1 += step;
        pointid++;

        sigma = c.transpose() * beta0;
        dsigma = c.transpose() * beta1;
        ddsigma = c.transpose() * beta2;
        dddsigma = c.transpose() * beta3;
        ddddsigma = c.transpose() * beta4;
         
        // ctrl_points_.col(i_dp) = sigma;
        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        // some help values
        
        z_h0 = dsigma.norm();
        z_h1 = ddsigma.transpose() * dsigma;
        z_h2 = dddsigma.transpose() * dsigma;
        z_h3 = ddsigma.transpose() * B_h * dsigma;
       
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }


        // add cost z_h0 = ||v||
        if ( z_h0 < 1e-4 || (j==0&&i==0) || (i==N-1&&j==K))
        {
          continue;
        }
        //avoid siguality

        vel2_reci = 1.0 / (z_h0 * z_h0);
        vel2_reci_e = 1.0 / (z_h0 * z_h0+epis);
        vel3_2_reci_e = vel2_reci_e * sqrt(vel2_reci_e);
        z_h0 = 1.0 / z_h0;

        z_h4 = z_h1 * vel2_reci;
        violaVel = 1.0 / vel2_reci - max_vel_ * max_vel_;
        acc2 = z_h1 * z_h1 * vel2_reci;
        cur2 = z_h3 * z_h3 * (vel2_reci_e * vel2_reci_e * vel2_reci_e);
        cur = z_h3 * vel3_2_reci_e;
        violaAcc = acc2 - max_acc_ * max_acc_;

        //@hzc: add feasibility with curvature
        violaCur = cur2 - max_cur_ * max_cur_;
        violaCurL = cur-max_cur_;
        violaCurR = -cur-max_cur_;

        ego_R << dsigma(0), -dsigma(1),
                 dsigma(1), dsigma(0);
        ego_R = singul_ * ego_R * z_h0;

        Eigen::Matrix2d temp_a, temp_v;
        temp_a << ddsigma(0), -ddsigma(1),
                  ddsigma(1), ddsigma(0);
        temp_v << dsigma(0), -dsigma(1),
                  dsigma(1), dsigma(0);
        Eigen::Matrix2d R_dot = singul_ * (temp_a * z_h0 - temp_v * vel2_reci * z_h0 * z_h1);

        for(auto le : vec_le_)
        {
          Eigen::Vector2d bpt = sigma + ego_R * le;

          Eigen::Matrix2d temp_l_Bl;
          temp_l_Bl << le(0), -le(1),
                       le(1), le(0);          

          int corr_k = cfgHs[pointid].cols();

          for(int k = 0; k < corr_k; k++)
          {
            outerNormal = cfgHs[pointid].col(k).head<2>();
            violaPos = outerNormal.dot(bpt - cfgHs[pointid].col(k).tail<2>());

            if(violaPos > 0)
            {
              positiveSmoothedL1(violaPos, violaPosPena, violaPosPenaD);
              
              gradViolaPc = beta0 * outerNormal.transpose() + 
                            beta1 * outerNormal.transpose() * (singul_ * temp_l_Bl * z_h0 - ego_R * le * dsigma.transpose() * vel2_reci);
              
              gradViolaPt = alpha * outerNormal.transpose() * (dsigma + R_dot * le);

              jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_obs_ * violaPosPenaD * gradViolaPc;
              gdTs[trajid](i) += omg * wei_obs_ * (violaPosPenaD * gradViolaPt * step + violaPosPena / K);

              costs(0) += omg * step * wei_obs_ * violaPosPena; // cost is the same
            }
          }
        }


        /*violaCurL = cur-max_cur_;
        violaCurR = -cur-max_cur_;*/

        if(violaCurL > 0.0){
          positiveSmoothedL1(violaCurL, violaCurPenaL, violaCurPenaDL);
          //@hzc
          gradViolaKLc = beta1 * (vel3_2_reci_e * ddsigma.transpose()*B_h - 3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) 
                         + beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose(); // 6*2
          gradViolaKLt  = alpha*vel3_2_reci_e*(dddsigma.transpose()*B_h*dsigma-3*vel2_reci_e*z_h3*z_h1);
          jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * 10.0 * violaCurPenaDL * gradViolaKLc;
          gdTs[trajid](i) += omg * wei_feas_ * 10.0 * (violaCurPenaDL * gradViolaKLt * step + violaCurPenaL / K);
          costs(2) += omg * step * wei_feas_ * 10.0 * violaCurPenaL;
        }
        if(violaCurR > 0.0){
          positiveSmoothedL1(violaCurR, violaCurPenaR, violaCurPenaDR);
          //@hzc
          gradViolaKRc = -(beta1 * (vel3_2_reci_e * ddsigma.transpose()*B_h - 3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) 
                         + beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose()); // 6*2
          gradViolaKRt  = -(alpha*vel3_2_reci_e*(dddsigma.transpose()*B_h*dsigma-3*vel2_reci_e*z_h3*z_h1));
          jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * 10.0 * violaCurPenaDR * gradViolaKRc;
          gdTs[trajid](i) += omg * wei_feas_ * 10.0 * (violaCurPenaDR * gradViolaKRt * step + violaCurPenaR / K);
          costs(2) += omg * step * wei_feas_ * 10.0 * violaCurPenaR;
        }
      }
      t += jerkOpt_container[trajid].get_T1()(i);
    }

    /*-------------------------------------------------------------------------------------------*/
    /*----------------------------fixed number sampling strategy---------------------------------*/

  }


  ///viola*, viola*Pena, viola*PenaD
  void PolyTrajOptimizer::positiveSmoothedL1(const double &x, double &f, double &df)
  {
        const double pe = 1.0e-4;
        const double half = 0.5 * pe;
        const double f3c = 1.0 / (pe * pe);
        const double f4c = -0.5 * f3c / pe;
        const double d2c = 3.0 * f3c;
        const double d3c = 4.0 * f4c;

        if (x < pe)
        {
            f = (f4c * x + f3c) * x * x * x;
            df = (d3c * x + d2c) * x * x;
        }
        else
        {
            f = x - half;
            df = 1.0;
        }
    //       df = x * x;
    // f = df *x;
    // df *= 3.0;
        return;
  }
  void PolyTrajOptimizer::positiveSmoothedL3(const double &x, double &f, double &df){
    df = x * x;
    f = df *x;
    df *= 3.0;
   

    return ;
  }

  // using the circle model, for other cars(with same parameters)
  bool PolyTrajOptimizer::surroundGradCostP(const int i_dp,           // index of constraint point
                                            const double t,           // current absolute time
                                            const Eigen::Vector2d &p, // the rear model
                                            const Eigen::Vector2d &v,
                                            Eigen::Vector2d &gradp,
                                            double &gradt,
                                            double &grad_prev_t,
                                            double &costp)
  {
    if (i_dp <= 0) return false;
    if (surround_trajs_->size() < 1) return false;

    bool ret = false;

    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    const double CLEARANCE2 = (surround_clearance_ * 1.5) * (surround_clearance_ * 1.5);
    // only counts when the distance is smaller than clearance

    constexpr double b = 1.0, inv_b2 = 1 / b / b;

    double pt_time = t_now_ + t;

    if (surround_trajs_->size() < 1) return false;

    for (size_t id = 0; id < surround_trajs_->size(); id++)
    {

      double traj_i_satrt_time = surround_trajs_->at(id).start_time;

      Eigen::Vector2d surround_p, surround_v;
      if (pt_time < traj_i_satrt_time + surround_trajs_->at(id).duration)
      {
        surround_p = surround_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        surround_v = surround_trajs_->at(id).traj.getdSigma(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + surround_trajs_->at(id).duration);
        surround_v = surround_trajs_->at(id).traj.getdSigma(surround_trajs_->at(id).duration);
        surround_p = surround_trajs_->at(id).traj.getPos(surround_trajs_->at(id).duration) +
                     exceed_time * surround_v;
      }

      Eigen::Vector2d dist_vec = p - surround_p;
      double ellip_dist2 = (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
      double dist2_err = CLEARANCE2 - ellip_dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0) // only accout the cost term when the distance is within the clearance
      {
        ret = true;

        costp += wei_surround_ * dist2_err3;
        Eigen::Vector2d dJ_dP = wei_surround_ * 3 * dist2_err2 * (-2) * Eigen::Vector2d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1));
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - surround_v);
        grad_prev_t += dJ_dP.dot(-surround_v);
      }

      if (min_ellip_dist2_ > ellip_dist2)
      {
        min_ellip_dist2_ = ellip_dist2;
      }
    }

    return ret;
  }


  // develop with signed distance
  // for current simulation, they add only vehicles with the same size.
  // in the future, more size can be added
  // 111
  double PolyTrajOptimizer::dynamicObsGradCostP(
                                              const double &omg_j,
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
                                              const int &trajid,const double& trajtime)
  {
    

    int singul_ = singul_container[trajid];
    // int sur_singul = 1; //moving obstacles always move forward
    // if (surround_trajs_==NULL||surround_trajs_->size() < 1) return 0.0;
    //t means the cur-t of the constraint point
    Eigen::Matrix<double, 6, 2> gradViolaPc;
    double gradViolaPt;
    // cout << "entered dynamic!!" << endl;
    double alpha = 100.0, d_min = surround_clearance_ + std::log(8.0) / alpha; // may have problems hzc

    double temp0 = dsigma.norm(); //  ||dsigma||_2
    double temp0_reci;
    if (temp0 != 0.0){
      temp0_reci = 1.0 / temp0;
    }else{
      temp0_reci = 0.0;
      ROS_ERROR("temp0_reci");
    }

    double temp3 = temp0_reci * temp0_reci; // ||dsigma||_2^2

    /*-------------prerequiste------------------*/
    int number_of_hyperplanes_of_ego_car = number_of_hyperplanes_of_ego_car_;
    int number_of_hyperplanes_of_surround_car = number_of_hyperplanes_of_surround_car_;
    std::vector<Eigen::Vector2d> vec_le = vec_le_;
    std::vector<Eigen::Vector2d> vec_lo = vec_lo_;
    // Eigen::Matrix2d ego_R_test;                     // Rotation matrix of the ego car
    // ego_R_test << dsigma(0), -dsigma(1),
    //               dsigma(1), dsigma(0);
    // ego_R_test = singul_ * temp0_reci * ego_R_test;
    /*------------------------------------------*/

    Eigen::Vector2d gradp,gradp2;
    double gradt,grad_prev_t;
    double totalPenalty = 0.0;
    double costp,violaDynamicObsPena,violaDynamicObsPenaD;

    // cout << "The number of swarm_traj_container is : " << swarm_traj_container_.size() << endl;

    for (size_t sur_id = 0; sur_id < cars_num_; sur_id++){
      if(sur_id == car_id_)
      {
        continue;
      }
      gradp.setZero();
      gradp2.setZero();
      gradt = 0;
      grad_prev_t = 0;

      double pt_time;
      plan_utils::Trajectory* surround_segment;

      double constrain_t_now = t_now_ + trajtime + t;
      int sur_segmentId = swarm_traj_container_[sur_id].locateSingulId(constrain_t_now);
      double sur_segment_start_time = swarm_traj_container_[sur_id].singul_traj[sur_segmentId].start_time;
      double sur_segment_end_time = swarm_traj_container_[sur_id].singul_traj[sur_segmentId].end_time;
      
      if(constrain_t_now < sur_segment_start_time)
      {
          int sur_last_segmentId = swarm_last_traj_container_[sur_id].locateSingulId(constrain_t_now);
          double sur_last_segment_start_time = swarm_last_traj_container_[sur_id].singul_traj[sur_last_segmentId].start_time;
          double sur_last_segment_end_time = swarm_last_traj_container_[sur_id].singul_traj[sur_last_segmentId].end_time;
          if(constrain_t_now < sur_last_segment_start_time)
          {
              pt_time = 0.0;
              surround_segment = &swarm_last_traj_container_[sur_id].singul_traj[sur_last_segmentId].traj;
          }
          else if(constrain_t_now > sur_last_segment_end_time)
          {
              pt_time = swarm_last_traj_container_[sur_id].singul_traj[sur_last_segmentId].duration;
              surround_segment = &swarm_last_traj_container_[sur_id].singul_traj[sur_last_segmentId].traj;
          }
          else
          {
              pt_time = constrain_t_now - sur_last_segment_start_time;
              surround_segment = &swarm_last_traj_container_[sur_id].singul_traj[sur_last_segmentId].traj;
          }
      }
      else if(constrain_t_now > sur_segment_end_time)
      {
          pt_time = swarm_traj_container_[sur_id].singul_traj[sur_segmentId].duration;
          surround_segment = &swarm_traj_container_[sur_id].singul_traj[sur_segmentId].traj;
      }
      else
      {
          pt_time = constrain_t_now - sur_segment_start_time;
          surround_segment = &swarm_traj_container_[sur_id].singul_traj[sur_segmentId].traj;
      }


      /**/
      // todo: account for the future motion
      /**/

      Eigen::Vector2d surround_p, surround_v, surround_a;

      surround_p = surround_segment->getPos(pt_time);


      if((surround_p-sigma).norm()>car_length_ * 2.0){
        continue;
      }

      surround_v = surround_segment->getdSigma(pt_time);
      surround_a = surround_segment->getddSigma(pt_time);
      int sur_singul = surround_segment->getSingul(pt_time);
 
      double temp_sur0 = surround_v.norm();
      double temp_sur_reci0;
      if (temp_sur0 != 0.0)
      {
        
        temp_sur_reci0 = 1.0 / temp_sur0;
      }
      else
      {
        temp_sur_reci0 = 0.0;
        ROS_ERROR("temp_sur_reci0 = 0.0!");
      }

      // Eigen::Matrix2d surround_R_test = surround_trajs_->at(sur_id).traj.getR(pt_time);

      Eigen::Matrix2d surround_R_test;                // Rotation matrix of the surround dynamic car
      surround_R_test << surround_v(0), -surround_v(1),
                         surround_v(1), surround_v(0);
      surround_R_test = sur_singul * temp_sur_reci0 * surround_R_test; 


      Eigen::VectorXd surround2ego_sum_exp_vec(number_of_hyperplanes_of_ego_car);
      Eigen::VectorXd d_U(number_of_hyperplanes_of_ego_car);
      std::vector<Eigen::Vector2d> ego_normal_vectors_vec;  ego_normal_vectors_vec.clear(); // This vector is used to store the normal vectors of each hyperplane of the ego car
      std::vector<Eigen::VectorXd> vec_d_Uo_e_vec; vec_d_Uo_e_vec.clear();
      std::vector<Eigen::Matrix2d> F_delta_le_vec;  F_delta_le_vec.clear(); // This vector stores the F(delta_le) which is later used in the gradient calculation
      std::vector<Eigen::Matrix2d> F_le_vec;  F_le_vec.clear(); // This vector stores the F(le) which is later used in the gradient calcualtion
      for(int e = 0; e < number_of_hyperplanes_of_ego_car; e++)
      {
        Eigen::Vector2d le = vec_le[e];
        Eigen::Vector2d delta_le = vec_le[e + 1] - vec_le[e];

        double delta_le_norm = delta_le.norm();
        double delta_le_norm_inverse = 1 / delta_le_norm;

        /*--------------calculate F(delta_le) below-----------------*/
        Eigen::Matrix2d temp_l_Bl;
        temp_l_Bl << delta_le(0), -delta_le(1),  //[l, Bl] in F(l)
                     delta_le(1), delta_le(0);
        Eigen::Matrix2d F_delta_le = singul_ * temp_l_Bl.transpose() * temp0_reci - dsigma * (ego_R * delta_le).transpose() * temp3;
        F_delta_le_vec.push_back(F_delta_le);
        /*----------------------------------------------------------*/

        /*-----------------calculate F(le) below--------------------*/
        temp_l_Bl << le(0), -le(1),
                     le(1), le(0);
        Eigen::Matrix2d F_le = singul_ * temp_l_Bl.transpose() * temp0_reci - dsigma * (ego_R * le).transpose() * temp3;
        F_le_vec.push_back(F_le);
        /*----------------------------------------------------------*/

        Eigen::VectorXd d_Uo_e_vec(number_of_hyperplanes_of_surround_car); // vector stores the d_Uo_e in the paper which is later put into the lse function

        Eigen::Vector2d H_tilde = B_h * ego_R * delta_le * delta_le_norm_inverse; 
        ego_normal_vectors_vec.push_back(H_tilde);
        double d_U_e_tilde = H_tilde.transpose() * (surround_p - sigma - ego_R * le);

        for(int o = 0; o < number_of_hyperplanes_of_surround_car; o++)
        {
          Eigen::Vector2d lo = vec_lo[o];
          double d_Uo_e = H_tilde.transpose() * surround_R_test * lo;
          d_Uo_e_vec(o) = d_Uo_e;
        }

        double exp_sum_d_Uo_e;
        double d_U_e = log_sum_exp(-alpha, d_Uo_e_vec, exp_sum_d_Uo_e) + d_U_e_tilde;
        d_U(e) = d_U_e;

        vec_d_Uo_e_vec.push_back(d_Uo_e_vec);

        // This vector stores the exp sum of the d_Uo_e, because lse' = sum(d_Uo_e / exp_sum_d_Uo_e)
        surround2ego_sum_exp_vec(e) = exp_sum_d_Uo_e;
      }


      Eigen::VectorXd ego2surround_sum_exp_vec(number_of_hyperplanes_of_surround_car);
      Eigen::VectorXd d_E(number_of_hyperplanes_of_surround_car);
      std::vector<Eigen::Vector2d> surround_normal_vectors_vec;  surround_normal_vectors_vec.clear(); // This vector is used to store the normal vectors of each hyperplane of the surround car
      std::vector<Eigen::VectorXd> vec_d_Ee_o_vec; vec_d_Ee_o_vec.clear();
      for(int o = 0; o < number_of_hyperplanes_of_surround_car; o++)
      {
        Eigen::Vector2d lo = vec_lo[o];
        Eigen::Vector2d delta_lo = vec_lo[o + 1] - vec_lo[o];

        double delta_lo_norm = delta_lo.norm();
        double delta_lo_norm_inverse = 1 / delta_lo_norm;

        Eigen::VectorXd d_Ee_o_vec(number_of_hyperplanes_of_ego_car); // vector stores the d_Ee_o in the paper which is later put into the lse function

        Eigen::Vector2d H_tilde = B_h * surround_R_test * delta_lo * delta_lo_norm_inverse;
        surround_normal_vectors_vec.push_back(H_tilde);
        double d_E_o_tilde = H_tilde.transpose() * (sigma - surround_p - surround_R_test * lo);

        for(int e = 0; e < number_of_hyperplanes_of_ego_car; e++)
        {
          Eigen::Vector2d le = vec_le[e];
          double d_Ee_o = H_tilde.transpose() * ego_R * le;
          d_Ee_o_vec(e) = d_Ee_o;
        }
        
        double exp_sum_d_Ee_o;
        double d_E_o = log_sum_exp(-alpha, d_Ee_o_vec, exp_sum_d_Ee_o) + d_E_o_tilde;
        d_E(o) = d_E_o;

        vec_d_Ee_o_vec.push_back(d_Ee_o_vec);

        ego2surround_sum_exp_vec(o) = exp_sum_d_Ee_o;
      }

      Eigen::VectorXd d_test(number_of_hyperplanes_of_ego_car + number_of_hyperplanes_of_surround_car);
      d_test << d_U, d_E;

      double exp_sum_d = 0;
      double d_value_test = d_min - log_sum_exp(alpha, d_test, exp_sum_d); 
      costp = d_value_test;

      if(costp <= 0) continue;
      positiveSmoothedL1(costp, violaDynamicObsPena, violaDynamicObsPenaD);
      // !!!!!!!!!!!!!!!!!This line is later put out!!!!!!!!!!!!!!!!
      totalPenalty += omg_j * delta_t_ * wei_surround_ * violaDynamicObsPena;


      /*---------------This part calculates the parital G over partial sigma-----------------*/
      Eigen::Vector2d partial_G_over_partial_sigma(0.0, 0.0); 
      for(int e = 0; e < number_of_hyperplanes_of_ego_car; e++)
      {
        Eigen::Vector2d partial_d_U_e_tilde_over_partial_sigma = - ego_normal_vectors_vec[e];
        partial_G_over_partial_sigma -= d_test(e) / exp_sum_d * partial_d_U_e_tilde_over_partial_sigma;
        // d_test(e) / exp_sum_d  is the derivative of the lse(alpha > 0) function
      }
      for(int o = 0; o < number_of_hyperplanes_of_surround_car; o++)
      {
        Eigen::Vector2d partial_d_E_o_tilde_over_partial_sigma = surround_normal_vectors_vec[o];
        partial_G_over_partial_sigma -= d_test(o + number_of_hyperplanes_of_ego_car) / exp_sum_d * partial_d_E_o_tilde_over_partial_sigma;
        // d_test(o) / exp_sum_d  is the derivative of the lse(alpha > 0) function
      }
      /*--------------------------------------------------------------------------------------*/


      /*-------------This part calculates the partial G over partial sigma_dot----------------*/
      Eigen::Vector2d partial_G_over_partial_dsigma(0.0, 0.0);
      for(int e = 0; e < number_of_hyperplanes_of_ego_car; e++)
      {
        Eigen::Matrix2d F_delta_le = F_delta_le_vec[e];
        Eigen::Matrix2d F_le = F_le_vec[e];
        Eigen::Vector2d le = vec_le[e];
        Eigen::Vector2d delta_le = vec_le[e + 1] - vec_le[e];
        double d_Uo_e_exp_sum = surround2ego_sum_exp_vec(e);

        Eigen::Vector2d partial_d_U_e_tilde_over_partial_dsigma 
              = (F_delta_le * B_h * (-surround_p + sigma + ego_R * le) - F_le * B_h * ego_R * delta_le) / delta_le.norm();
                //  = F_delta_le * B_h * sigma / (delta_le.norm());

        Eigen::Vector2d partial_d_U_e_over_partial_dsigma = partial_d_U_e_tilde_over_partial_dsigma;
        for(int o = 0; o < number_of_hyperplanes_of_surround_car; o++)
        {
          double d_Uo_e = vec_d_Uo_e_vec[e](o);
          Eigen::Vector2d lo = vec_lo[o];
          Eigen::Vector2d partial_d_Uo_e_over_partial_dsigma 
                      = F_delta_le * B_h.transpose() * (surround_R_test * lo) / (delta_le.norm());

          partial_d_U_e_over_partial_dsigma += d_Uo_e / d_Uo_e_exp_sum * partial_d_Uo_e_over_partial_dsigma;
        }

        partial_G_over_partial_dsigma -= d_test(e) / exp_sum_d * partial_d_U_e_over_partial_dsigma;
      }

      for(int o = 0; o < number_of_hyperplanes_of_surround_car; o++)
      {
        Eigen::Vector2d delta_lo = vec_lo[o + 1] - vec_lo[o];
        double d_Ee_o_exp_sum = ego2surround_sum_exp_vec(o);

        Eigen::Vector2d partial_d_E_o_over_partial_dsigma(0.0, 0.0);
        for(int e = 0; e < number_of_hyperplanes_of_ego_car; e++)
        {
          Eigen::Matrix2d F_le = F_le_vec[e];
          double d_Ee_o = vec_d_Ee_o_vec[o](e);

          Eigen::Vector2d partial_d_Ee_o_over_partial_dsigma
                                = F_le * B_h * surround_R_test * delta_lo / (delta_lo.norm());

          partial_d_E_o_over_partial_dsigma += d_Ee_o / d_Ee_o_exp_sum * partial_d_Ee_o_over_partial_dsigma;
        }

        partial_G_over_partial_dsigma -= d_test(o + number_of_hyperplanes_of_ego_car) / exp_sum_d * partial_d_E_o_over_partial_dsigma;
      }
      /*--------------------------------------------------------------------------------------*/


      /*-------------This part calculates the partial G over partial t_bar--------------------*/
      Eigen::Matrix<double, 1, 1> partial_G_over_partial_t_bar_mat
                = partial_G_over_partial_sigma.transpose() * dsigma + partial_G_over_partial_dsigma.transpose() * ddsigma;
      double partial_G_over_partial_t_bar = partial_G_over_partial_t_bar_mat(0, 0);
      // Even though partial G over partial t_bar is a double, the calculation is still a 1*1 matrix
      /*--------------------------------------------------------------------------------------*/


      /*---------------This part calculates the partial G over partial t_hat------------------*/
      // double partial_G_over_partial_t_hat = 0.0;
      // // calculate R_u_t_hat_dot first, which means the derivative of surround car's rotation matrix
      // Eigen::Matrix2d temp_ddsigma_Bddsigma, temp_dsigma_Bdsigma;
      // temp_ddsigma_Bddsigma << surround_a(0), -surround_a(1),
      //                          surround_a(1), surround_a(0);
      // temp_dsigma_Bdsigma << surround_v(0), -surround_v(1),
      //                        surround_v(1), surround_v(0);

      // Eigen::Matrix2d R_u_t_hat_dot 
      //   = sur_singul * (temp_ddsigma_Bddsigma * temp_sur_reci0 - temp_dsigma_Bdsigma * pow(temp_sur_reci0, 3) * (surround_v.transpose() * surround_a));
      // //////////////////////////////////////////////////////////////////////////////////////////////
      // for(int e = 0; e < number_of_hyperplanes_of_ego_car; e++)
      // {
      //   Eigen::Vector2d le = vec_le[e];
      //   Eigen::Vector2d delta_le = vec_le[e + 1] - vec_le[e];
      //   double d_Uo_e_exp_sum = surround2ego_sum_exp_vec(e);

      //   // double partial_d_U_e_tilde_over_partial_t_hat = (surround_v.transpose() * B_h * ego_R_test * delta_le / delta_le.norm())(0, 0);
      //   double partial_d_U_e_tilde_over_partial_t_hat = ego_normal_vectors_vec[e].transpose() * surround_v;
        
      //   double partial_d_U_e_over_partial_t_hat = partial_d_U_e_tilde_over_partial_t_hat;
      //   for(int o = 0; o < number_of_hyperplanes_of_surround_car; o++)
      //   {
      //     Eigen::Vector2d lo = vec_lo[o];
      //     double partial_d_Uo_e_over_partial_t_hat 
      //           = ego_normal_vectors_vec[e].transpose() * R_u_t_hat_dot * lo;
          
      //     double d_Uo_e = vec_d_Uo_e_vec[e](o);
      //     partial_d_U_e_over_partial_t_hat += d_Uo_e / d_Uo_e_exp_sum * partial_d_Uo_e_over_partial_t_hat;
      //   }

      //   partial_G_over_partial_t_hat -= d_test(e) / exp_sum_d * partial_d_U_e_over_partial_t_hat;
      // }

      // for(int o = 0; o < number_of_hyperplanes_of_surround_car; o++)
      // {
      //   double d_Ee_o_exp_sum = ego2surround_sum_exp_vec(o);

      //   Eigen::Vector2d lo = vec_lo[o];
      //   Eigen::Vector2d delta_lo = vec_lo[o + 1] - vec_lo[o];
      //   Eigen::Matrix<double, 1, 1>  temp_variable
      //       =   (B_h * R_u_t_hat_dot * delta_lo).transpose() / delta_lo.norm() * (sigma - surround_p - surround_R_test * lo)
      //         + (B_h * surround_R_test * delta_lo).transpose() / delta_lo.norm() *  (-surround_v - R_u_t_hat_dot * lo);
      //   double partial_d_E_o_tilde_over_partial_t_hat = temp_variable(0, 0);

      //   double partial_d_E_o_over_partial_t_hat = partial_d_E_o_tilde_over_partial_t_hat;
      //   for(int e = 0; e < number_of_hyperplanes_of_ego_car; e++)
      //   {
      //     double d_Ee_o = vec_d_Ee_o_vec[o](e);
      //     Eigen::Vector2d le = vec_le[e];
      //     double partial_d_Ee_o_over_partial_t_hat = ((ego_R * le).transpose() * B_h * R_u_t_hat_dot * delta_lo / delta_lo.norm())(0, 0);

      //     partial_d_E_o_over_partial_t_hat += d_Ee_o / d_Ee_o_exp_sum * partial_d_Ee_o_over_partial_t_hat;
      //   }

      //   partial_G_over_partial_t_hat -= d_test(o + number_of_hyperplanes_of_ego_car) / exp_sum_d * partial_d_E_o_over_partial_t_hat;
      // }
      /*--------------------------------------------------------------------------------------*/


      gradp = partial_G_over_partial_sigma;
      gradp2 = partial_G_over_partial_dsigma;
      gradt = partial_G_over_partial_t_bar;
      // grad_prev_t = partial_G_over_partial_t_hat;

      gradViolaPc = beta0 * gradp.transpose() + beta1 * gradp2.transpose();
      gradViolaPt = gradt;
      jerkOpt_container[trajid].get_gdC().block<6, 2>(pieceid * 6, 0) += omg_j * delta_t_ * wei_surround_ * violaDynamicObsPenaD * gradViolaPc; // j gradient to c
      gdTs[trajid](pieceid) += omg_j * delta_t_ * wei_surround_ * violaDynamicObsPenaD * gradViolaPt * time_int_pena;                     // j gradient to t
      // if (pieceid > 0)
      // {
      //   gdTs[trajid].head(pieceid).array() += omg * step * wei_surround_ * grad_prev_t * violaDynamicObsPenaD; // the gradient of absolute t
      // }
      // gdTs[trajid](pieceid) += omg * step * wei_surround_ *  gama * grad_prev_t * violaDynamicObsPenaD; 

      // for(int idx = 0; idx < trajid; idx++){
      //   gdTs[idx].array() += omg * step * wei_surround_ * grad_prev_t * violaDynamicObsPenaD;
      // }
    
    }
    return totalPenalty;
  }

  double PolyTrajOptimizer::log_sum_exp(double alpha, Eigen::VectorXd &all_dists, double &exp_sum)
  {
    // all_dists will be std::exp(alpha * (all_dists(j) - d_max));
    double d_0;
    if (alpha > 0)
    {
      d_0 = all_dists.maxCoeff();
    }
    else
    {
      d_0 = all_dists.minCoeff();
    }

    exp_sum = 0;
    for (unsigned int j = 0; j < all_dists.size(); j++)
    {
      all_dists(j) = std::exp(alpha * (all_dists(j) - d_0));
      exp_sum += all_dists(j);
    }

    return std::log(exp_sum) / alpha + d_0;
  }

  void PolyTrajOptimizer::init(ros::NodeHandle& nh)
  {
      nh_ = nh;
      nh_.param("optimizing/traj_resolution", traj_resolution_, 8);
      nh_.param("optimizing/des_traj_resolution", destraj_resolution_, 20);
      nh_.param("optimizing/wei_sta_obs", wei_obs_, 7000.0);
      nh_.param("optimizing/wei_dyn_obs", wei_surround_, 7000.0);
      nh_.param("optimizing/wei_feas", wei_feas_, 1000.0);
      nh_.param("optimizing/wei_sqrvar", wei_sqrvar_, 1000.0);
      nh_.param("optimizing/wei_time", wei_time_, 500.0);
      nh_.param("optimizing/dyn_obs_clearance", surround_clearance_, 1.0);
      nh_.param("optimizing/max_vel", max_vel_, 10.0);
      nh_.param("optimizing/min_vel", min_vel_, -10.0);
      nh_.param("optimizing/max_acc", max_acc_, 5.0);
      nh_.param("optimizing/min_acc", min_acc_, -5.0);
      nh_.param("optimizing/max_cur", max_cur_, 0.350877);
      nh_.param("optimizing/half_margin", half_margin, 0.25);

      nh_.param("vehicle/cars_num", cars_num_, 1);
      nh_.param("vehicle/car_id", car_id_, 0);
      nh_.param("vehicle/car_length", car_length_, 4.88);
      nh_.param("vehicle/car_width", car_width_, 1.5);
      nh_.param("vehicle/car_d_cr", car_d_cr_, 1.015);
      nh_.param("vehicle/wheelbase", car_wheelbase_, 2.85);

      B_h << 0, -1,
             1, 0;
      delta_t_ = 0.2;
      have_received_trajs_.resize(cars_num_);
      swarm_traj_container_.resize(cars_num_);
      swarm_last_traj_container_.resize(cars_num_);
      fill(have_received_trajs_.begin(), have_received_trajs_.end(), false);
      ifdynamic_ = false;

      car_width_ += 2 * half_margin;
      car_length_ += 2 * half_margin;

      double half_wid = 0.5 * car_width_;
      double half_len = 0.5 * car_length_;

      lz_set_.push_back(Eigen::Vector2d(  half_len,  half_wid));
      lz_set_.push_back(Eigen::Vector2d(- half_len,  half_wid));
      lz_set_.push_back(Eigen::Vector2d(  half_len, -half_wid));
      lz_set_.push_back(Eigen::Vector2d(- half_len, -half_wid));
    
      Eigen::Vector2d le_1, le_2, le_3, le_4;        // vertexs of the ego car in the body frame
      Eigen::Vector2d lo_1, lo_2, lo_3, lo_4;        // vertexs of the surround car in the body frame
      vec_le_.clear(); vec_lo_.clear();
      le_1 << car_d_cr_ + car_length_ / 2.0, car_width_ / 2.0;
      le_2 << car_d_cr_ + car_length_ / 2.0, -car_width_ / 2.0;
      le_3 << car_d_cr_ - car_length_ / 2.0, -car_width_ / 2.0;
      le_4 << car_d_cr_ - car_length_ / 2.0, car_width_ / 2.0;
      lo_1 = le_1; lo_2 = le_2; lo_3 = le_3; lo_4 = le_4;

      // attention here! These vectors store one more of the vertexs! The vertexs are stored Clockwise！
      vec_le_.push_back(le_1); vec_le_.push_back(le_2); vec_le_.push_back(le_3); vec_le_.push_back(le_4); 
      vec_le_.push_back(le_1); // !!!!!
      vec_lo_.push_back(lo_1); vec_lo_.push_back(lo_2); vec_lo_.push_back(lo_3); vec_lo_.push_back(lo_4); 
      vec_lo_.push_back(lo_1); // !!!!!   

      number_of_hyperplanes_of_ego_car_ = vec_le_.size() - 1;
      number_of_hyperplanes_of_surround_car_ = vec_lo_.size() - 1;  


  }
  
  void PolyTrajOptimizer::setSurroundTrajs(plan_utils::SurroundTrajData *surround_trajs_ptr) { surround_trajs_ = surround_trajs_ptr; 
  }

  void PolyTrajOptimizer::setAllCarsTrajs(plan_utils::TrajContainer& trajectory, int& car_id)
  {
      swarm_traj_container_.at(car_id) = trajectory;
      have_received_trajs_[car_id] = true;
      if(!ifdynamic_)
      {
          int number_of_received_trajs = 0;
          for(int i = 0; i < cars_num_; i++)
          {
              if(have_received_trajs_[i])
              {
                  number_of_received_trajs++;
              }
          }
          if(number_of_received_trajs == cars_num_)
          {
              ifdynamic_ = true;
          }
      }
  }

  void PolyTrajOptimizer::setAllCarsLastTrajs(plan_utils::TrajContainer& trajectory, int& car_id)
  {
      swarm_last_traj_container_.at(car_id) = trajectory;
  }

  bool PolyTrajOptimizer::checkCollisionWithSurroundCars(const double& time)
  { 
      bool collision = false;
      if(!ifdynamic_)
        return collision;

      int ego_segmentId = swarm_traj_container_[car_id_].locateSingulId(time);
      double ego_start_time = swarm_traj_container_[car_id_].singul_traj[ego_segmentId].start_time;
      double ego_end_time = swarm_traj_container_[car_id_].singul_traj[ego_segmentId].end_time;
      // double cal_time = time;
      // if(time < ego_start_time)
      // {
      //     cal_time = ego_start_time;
      // }
      // else if(time > ego_end_time)
      // {
      //     cal_time = ego_end_time;
      // }
      // Eigen::Vector2d ego_pos = swarm_traj_container_[car_id_].singul_traj[ego_segmentId].traj.getPos(cal_time - ego_start_time);
      // double ego_yaw = swarm_traj_container_[car_id_].singul_traj[ego_segmentId].traj.getAngle(cal_time - ego_start_time);
      Eigen::Vector2d ego_pos;
      double ego_yaw;
      if(time < ego_start_time)
      {
          int ego_last_segmentId = swarm_last_traj_container_[car_id_].locateSingulId(time);
          double ego_last_start_time = swarm_last_traj_container_[car_id_].singul_traj[ego_last_segmentId].start_time;
          double ego_last_end_time = swarm_last_traj_container_[car_id_].singul_traj[ego_last_segmentId].end_time;
          if(time < ego_last_start_time)
          {
              ego_pos = swarm_last_traj_container_[car_id_].singul_traj[ego_last_segmentId].traj.getPos(0.0);
              ego_yaw = swarm_last_traj_container_[car_id_].singul_traj[ego_last_segmentId].traj.getAngle(0.0);
          }
          else if(time > ego_last_end_time)
          {
              ego_pos = swarm_last_traj_container_[car_id_].singul_traj[ego_last_segmentId].traj.getPos(ego_last_end_time - ego_last_start_time);
              ego_yaw = swarm_last_traj_container_[car_id_].singul_traj[ego_last_segmentId].traj.getAngle(ego_last_end_time - ego_last_start_time);
          }
          else
          {
              ego_pos = swarm_last_traj_container_[car_id_].singul_traj[ego_last_segmentId].traj.getPos(time - ego_last_start_time);
              ego_yaw = swarm_last_traj_container_[car_id_].singul_traj[ego_last_segmentId].traj.getAngle(time - ego_last_start_time);
          }
      }
      else if(time > ego_end_time)
      {
          ego_pos = swarm_traj_container_[car_id_].singul_traj[ego_segmentId].traj.getPos(ego_end_time - ego_start_time);
          ego_yaw = swarm_traj_container_[car_id_].singul_traj[ego_segmentId].traj.getAngle(ego_end_time - ego_start_time);
      }
      else
      {
          ego_pos = swarm_traj_container_[car_id_].singul_traj[ego_segmentId].traj.getPos(time - ego_start_time);
          ego_yaw = swarm_traj_container_[car_id_].singul_traj[ego_segmentId].traj.getAngle(time - ego_start_time);
      }


      Eigen::Matrix2d ego_R;
      ego_R << cos(ego_yaw), -sin(ego_yaw),
               sin(ego_yaw),  cos(ego_yaw);
      std::vector<Eigen::Vector2d> Vec_ego_car_vertexs;
      std::vector<Eigen::Vector2d> Vec_ego_car_normal_vectors;
      for(int e = 0; e < number_of_hyperplanes_of_ego_car_; e++)
      {
          Eigen::Vector2d vertex_pos1 = ego_pos + ego_R * vec_le_[e];
          Eigen::Vector2d vertex_pos2 = ego_pos + ego_R * vec_le_[e+1];
          Eigen::Vector2d normal_vec = B_h * (vertex_pos2 - vertex_pos1);

          Vec_ego_car_vertexs.push_back(vertex_pos1);
          Vec_ego_car_normal_vectors.push_back(normal_vec);
      }

      for(int car = 0; car < cars_num_; car++)
      {
          if(car == car_id_)
              continue;
          
          /*todo*/
          // account for the future motion
          /*----*/

          int sur_segmentId = swarm_traj_container_[car].locateSingulId(time);
          double sur_start_time = swarm_traj_container_[car].singul_traj[sur_segmentId].start_time;
          double sur_end_time = swarm_traj_container_[car].singul_traj[sur_segmentId].end_time;
          int sur_singul = swarm_traj_container_[car].singul_traj[sur_segmentId].traj.getDirection();
          // double cal_time;
          // if(time < sur_start_time)
          //     cal_time = sur_start_time;
          // else if(time > sur_end_time)
          //     cal_time = sur_end_time;
          // Eigen::Vector2d sur_pos = swarm_traj_container_[car].singul_traj[sur_segmentId].traj.getPos(cal_time - sur_start_time);
          // double sur_yaw = swarm_traj_container_[car].singul_traj[sur_segmentId].traj.getAngle(cal_time - sur_start_time);
          Eigen::Vector2d sur_pos;
          double sur_yaw;
          if(time < sur_start_time)
          {
              int sur_last_segmentId = swarm_last_traj_container_[car].locateSingulId(time);
              double sur_last_start_time = swarm_last_traj_container_[car].singul_traj[sur_last_segmentId].start_time;
              double sur_last_end_time = swarm_last_traj_container_[car].singul_traj[sur_last_segmentId].end_time;
              if(time < sur_last_start_time)
              {
                  sur_pos = swarm_last_traj_container_[car].singul_traj[sur_last_segmentId].traj.getPos(0.0);
                  sur_yaw = swarm_last_traj_container_[car].singul_traj[sur_last_segmentId].traj.getAngle(0.0);
              }
              else if(time > sur_last_end_time)
              {
                  sur_pos = swarm_last_traj_container_[car].singul_traj[sur_last_segmentId].traj.getPos(sur_last_end_time - sur_last_start_time);
                  sur_yaw = swarm_last_traj_container_[car].singul_traj[sur_last_segmentId].traj.getAngle(sur_last_end_time - sur_last_start_time);
              }
              else
              {
                  sur_pos = swarm_last_traj_container_[car].singul_traj[sur_last_segmentId].traj.getPos(time - sur_last_start_time);
                  sur_yaw = swarm_last_traj_container_[car].singul_traj[sur_last_segmentId].traj.getAngle(time - sur_last_start_time);
              }
          }
          else if(time > sur_end_time)
          {
              sur_pos = swarm_traj_container_[car].singul_traj[sur_segmentId].traj.getPos(sur_end_time - sur_start_time);
              sur_yaw = swarm_traj_container_[car].singul_traj[sur_segmentId].traj.getAngle(sur_end_time - sur_start_time);
          }
          else
          {
              sur_pos = swarm_traj_container_[car].singul_traj[sur_segmentId].traj.getPos(time - sur_start_time);
              sur_yaw = swarm_traj_container_[car].singul_traj[sur_segmentId].traj.getAngle(time - sur_start_time);
          }
          
          
          
          // if((sur_pos - ego_pos).norm() > 2 * car_length_)
          //     continue;
          
          // else
          // {
          //     collision = true;
          //     ROS_ERROR("About to collide! Replan!");
          //     break;              
          // }


          if((sur_pos - ego_pos).norm() > 2 * car_length_)
              continue;
          
          Eigen::Matrix2d sur_R;
          sur_R << cos(sur_yaw), -sin(sur_yaw),
                   sin(sur_yaw),  cos(sur_yaw);

          std::vector<Eigen::Vector2d> Vec_sur_car_vertexs;
          std::vector<Eigen::Vector2d> Vec_sur_car_normal_vectors;
          for(int o = 0; o < number_of_hyperplanes_of_surround_car_; o++)
          {
              Eigen::Vector2d vertex_pos1 = sur_pos + sur_R * vec_lo_[o];
              Eigen::Vector2d vertex_pos2 = sur_pos + sur_R * vec_lo_[o+1];
              Eigen::Vector2d normal_vec = B_h * (vertex_pos2 - vertex_pos1);

              Vec_sur_car_vertexs.push_back(vertex_pos1);
              Vec_sur_car_normal_vectors.push_back(normal_vec);
          }

          
          std::vector<double> Vec_sur_to_ego_min_distance;
          for(int e = 0; e < number_of_hyperplanes_of_ego_car_; e++)
          {
              std::vector<double> sur_to_ego_distance;
              Eigen::Vector2d ego_point = Vec_ego_car_vertexs[e];
              Eigen::Vector2d ego_norm_vec = Vec_ego_car_normal_vectors[e];
              for(int o = 0; o < number_of_hyperplanes_of_surround_car_; o++)
              {
                  Eigen::Vector2d sur_point = Vec_sur_car_vertexs[o];
                  double signed_distance = (sur_point - ego_point).dot(ego_norm_vec) / ego_norm_vec.norm();
                  sur_to_ego_distance.push_back(signed_distance);
              }
              double sur_to_ego_min_distance = *min_element(sur_to_ego_distance.begin(), sur_to_ego_distance.end());
              Vec_sur_to_ego_min_distance.push_back(sur_to_ego_min_distance);
          }
          double sur_to_ego_min_max_distance = *max_element(Vec_sur_to_ego_min_distance.begin(), Vec_sur_to_ego_min_distance.end());
          
          std::vector<double> Vec_ego_to_sur_min_distance;
          for(int o = 0; o < number_of_hyperplanes_of_surround_car_; o++)
          {
              std::vector<double> ego_to_sur_distance;
              Eigen::Vector2d sur_point = Vec_sur_car_vertexs[o];
              Eigen::Vector2d sur_norm_vec = Vec_sur_car_normal_vectors[o];
              for(int e = 0; e < number_of_hyperplanes_of_ego_car_; e++)
              {
                  Eigen::Vector2d ego_point = Vec_ego_car_vertexs[e];
                  double signed_distance = (ego_point - sur_point).dot(sur_norm_vec) / sur_norm_vec.norm();
                  ego_to_sur_distance.push_back(signed_distance);
              }
              double ego_to_sur_min_distance = *min_element(ego_to_sur_distance.begin(), ego_to_sur_distance.end());
              Vec_ego_to_sur_min_distance.push_back(ego_to_sur_min_distance);
          }
          double ego_to_sur_min_max_distance = *max_element(Vec_ego_to_sur_min_distance.begin(), Vec_ego_to_sur_min_distance.end());
          
          double signed_distance_between_2cars = std::max(sur_to_ego_min_max_distance, ego_to_sur_min_max_distance);
          if(signed_distance_between_2cars < 0.0)
          {
              collision = true;
              // ROS_ERROR("About to collide! Replan!");
              break;
          }
      }

      return collision;
  }

} // namespace plan_manage