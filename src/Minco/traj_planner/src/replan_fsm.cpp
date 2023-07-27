#include <plan_manage/replan_fsm.h>
#include <chrono>

void ReplanFSM::init(ros::NodeHandle &nh)
{
    nh_ = nh;
    exec_state_ = ReplanFSM::FSM_EXEC_STATE::INIT;
    have_target_ = false;
    collision_with_obs_ = false;
    collision_with_othercars_ = false;

    nh_.param("mapping/odometry_topic", odom_topic_, odom_topic_);
    nh_.param("vehicle/car_d_cr", car_d_cr_, 1.015);
    nh_.param("vehicle/car_id", car_id_, 0);
    nh_.param("fsm/target_x", target_x_, 0.0);
    nh_.param("fsm/target_y", target_y_, 0.0);
    nh_.param("fsm/target_yaw", target_yaw_, 0.0);
    mapping_ptr_.reset(new MappingProcess);
    mapping_ptr_->init(nh);

    planner_ptr_.reset(new TrajPlanner);
    planner_ptr_->setMap(mapping_ptr_);
    planner_ptr_->init(nh);
    
    // planner_ptr_->Init(minco_config_path, car_id);
    odom_sub_    = nh_.subscribe(odom_topic_, 100, &ReplanFSM::OdomCallback, this);
    parking_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &ReplanFSM::ParkingCallback, this);
    swarm_traj_sub_ = nh_.subscribe("/broadcast_traj_to_planner", 100, &ReplanFSM::SwarmTrajCallback, this);

    exec_timer_ = nh_.createTimer(ros::Duration(0.02), &ReplanFSM::execFSMCallback, this);
    safety_timer_ = nh_.createTimer(ros::Duration(0.01), &ReplanFSM::checkCollisionCallback, this);
}

void ReplanFSM::OdomCallback(const nav_msgs::Odometry& msg)
{
    Eigen::Vector3d center_pos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    Eigen::Vector3d pos2center(-car_d_cr_, 0, 0);

    Eigen::Quaterniond quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, 
                                  msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d R = quaternion.toRotationMatrix();
    Eigen::Vector3d pos = center_pos /*+ R * pos2center*/;

    cur_pos_ = pos.head(2);
    cur_vel_ = msg.twist.twist.linear.x;
    cur_yaw_ = tf::getYaw(msg.pose.pose.orientation);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(cur_pos_(0), cur_pos_(1), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, cur_yaw_);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg.header.stamp, "map", "car_"+to_string(car_id_)+"_pos"));
}

void ReplanFSM::ParkingCallback(const geometry_msgs::PoseStamped &msg)
{
    std::cout << "Triggered parking mode!" << std::endl;
    // end_pt_ << msg.pose.position.x, msg.pose.position.y, 
    //            tf::getYaw(msg.pose.orientation), 1.0e-2;
    end_pt_ << target_x_, target_y_, 
               target_yaw_, 1.0e-2;
    std::cout<<"end_pt: "<<end_pt_.transpose()<<std::endl;
    
    have_target_ = true;
}

void ReplanFSM::SwarmTrajCallback(const swarm_bridge::Trajectory& traj_msg)
{
    planner_ptr_->setSwarmTrajs(traj_msg);
}

void ReplanFSM::execFSMCallback(const ros::TimerEvent &e)
{
    exec_timer_.stop();
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 50)
    {
    //   printFSMExecState();
    //   if (!have_odom_)
    //     cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
        case INIT:
        {
            // if(!have_odom_)
            // {
            //     goto force_return;
            //     // return;
            // }

            changeFSMExecState(WAIT_TARGET, "FSM");
            break;
        }

        case WAIT_TARGET:
        {
            if(!have_target_ /*|| !have_trigger_*/)
                goto force_return;

            else
            {
                changeFSMExecState(SEQUENTIAL_START, "FSM");
            }
            break;
        }

        case SEQUENTIAL_START:
        {
            // Eigen::Vector4d init_state;
            init_state_ << cur_pos_, cur_yaw_, cur_vel_;

            double start_time = ros::Time::now().toSec() + TIME_BUDGET;
            start_world_time_ = start_time;
            
            ros::Time t1 = ros::Time::now();
            planner_ptr_->setInitStateAndInput(init_state_, start_time);
            planner_ptr_->setParkingEnd(end_pt_);
            planner_ptr_->getKinoPath(end_pt_, true);
            // planner_ptr_->displayPolyH(planner_ptr_->display_hPolys());
            planner_ptr_->displayKinoPath(planner_ptr_->display_kino_path());
            planner_ptr_->RunMINCOParking();
            planner_ptr_->broadcastTraj2SwarmBridge();
            ros::Time t2 = ros::Time::now();
            double time_spent_in_planning = (t2 - t1).toSec();
            if(TIME_BUDGET > time_spent_in_planning)
            {
                ros::Duration(TIME_BUDGET - time_spent_in_planning).sleep();
            }
            else
            {
                ROS_ERROR("Out of time budget!");
            }
            // planner_ptr_->publishTraj2Controller();
            planner_ptr_->publishTraj2Simulator();
            
            planner_ptr_->displayMincoTraj(planner_ptr_->trajectory());

            changeFSMExecState(EXEC_TRAJ, "FSM");

            break;
        }

        // case GEN_NEW_TRAJ:
        // {
            
        // }

        case REPLAN_TRAJ:
        {
            ros::Time t_now = ros::Time::now();
            double replan_start_time = t_now.toSec() + TIME_BUDGET;
            start_world_time_ = replan_start_time;
            
            ros::Time t1 = ros::Time::now();
            Eigen::Vector4d replan_init_state;
            planner_ptr_->setInitStateAndInput(replan_start_time, replan_init_state);
            init_state_ = replan_init_state;

            planner_ptr_->setParkingEnd(end_pt_);
            if(!planner_ptr_->getKinoPath(end_pt_, false))
            {
                while(true)
                {
                    double t_cur = ros::Time::now().toSec();
                    if(t_cur > replan_start_time + 0.2)
                    {
                        break;
                    }
                    else
                    {
                        planner_ptr_->setMapFree(t_cur);
                    }
                }
                // ros::Duration(0.5).sleep();
                break;
            }
            planner_ptr_->displayKinoPath(planner_ptr_->display_kino_path());
            if(!planner_ptr_->RunMINCOParking())
            {
                while(true)
                {
                    double t_cur = ros::Time::now().toSec();
                    if(t_cur > replan_start_time + 0.1)
                    {
                        break;
                    }
                    else
                    {
                        planner_ptr_->setMapFree(t_cur);
                    }
                }
                break;
            }
            planner_ptr_->broadcastTraj2SwarmBridge();
            ros::Time t2 = ros::Time::now();
            double time_spent_in_planning = (t2 - t1).toSec();
            if(TIME_BUDGET > time_spent_in_planning)
            {
                // ros::Duration(TIME_BUDGET - time_spent_in_planning).sleep();
                while(true)
                {
                    double t_cur = ros::Time::now().toSec();
                    if(t_cur > replan_start_time)
                    {
                        break;
                    }
                    else
                    {
                        planner_ptr_->setMapFree(t_cur);
                    }
                }
            }
            else
            {
                ROS_ERROR("Out of time budget!");
            }
            // planner_ptr_->publishTraj2Controller();
            planner_ptr_->publishTraj2Simulator();
            
            // planner_ptr_->displayPolyH(planner_ptr_->display_hPolys());
            
            planner_ptr_->displayMincoTraj(planner_ptr_->trajectory());
            changeFSMExecState(EXEC_TRAJ, "FSM");

            break;
            // goto force_return;
        }

        case EXEC_TRAJ:
        {
            ros::Time t_now = ros::Time::now();
            if(((cur_pos_ - init_state_.head(2)).norm() > 10.0 || (t_now.toSec() - start_world_time_) > 2.5) && (cur_pos_ - end_pt_.head(2)).norm() > 5.0 /*&& !collision_with_othercars_*/)
            {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }

            if((collision_with_obs_ || collision_with_othercars_) && t_now.toSec() - start_world_time_ > TIME_BUDGET /*+ 0.3*/) // make sure the new trajectory have been executed and then replan
            {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
                collision_with_obs_ = false;
                collision_with_othercars_ = false;
                break;
            }
            
            // reach end
            if((cur_pos_ - end_pt_.head(2)).norm() < 0.5 && abs(cur_yaw_ - end_pt_(2) < 0.15 && abs(cur_vel_) < 0.2))
            {
                changeFSMExecState(WAIT_TARGET, "FSM");
                have_target_ = false;
                goto force_return;
            }

            break;
        }
    }

    force_return:;
    exec_timer_.start();
}

void ReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
{

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;    
}

void ReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
{   

    double time_now = ros::Time::now().toSec();
    // set other cars' position of map is free
    planner_ptr_->setMapFree(time_now);

    // check collision with static obstacles
    if(exec_state_ == EXEC_TRAJ)
        collision_with_obs_ = planner_ptr_->checkCollisionWithObs(time_now);

    // check collision with surround cars
    if(exec_state_ == EXEC_TRAJ)
        collision_with_othercars_ = planner_ptr_->checkCollisionWithOtherCars(time_now);

}