#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <ego_planner/Bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <shared_msgs/TaskResult.h>
// #include <shared_mutex>
// #include <atomic>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };//状态机状态设置
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };//目标点设置
    std::shared_ptr<GridMap> grid_map_;
    std::string current_task_;
    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    ego_planner::DataDisp data_disp_;


    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;//重规划阈值
    double waypoints_[50][3];
    int waypoint_num_;
    double planning_horizen_, planning_horizen_time_;// 规划视野和时间
    double emergency_time_;// 紧急时间
    // mutable std::shared_timed_mutex fsm_mutex;
    // std::atomic<bool> trigger_{false};
    // std::atomic<bool> have_target_{false};
    // std::atomic<bool> have_odom_{false};
    // std::atomic<bool> have_new_target_{false};
    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;// 各种状态标志
    FSM_EXEC_STATE exec_state_;// 当前执行状态
    int continously_called_times_{0};// 连续调用次数

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    int current_wp_;
    int drone_id_;

    bool flag_escape_emergency_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_, odom_to_decision_pub_;

    /* helper functions */
    //bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    //void odometryCallback(const nav_msgs::OdometryConstPtr &msg);


    bool checkCollision();
  public:
    EGOReplanFSM(std::shared_ptr<GridMap> grid_map): grid_map_(grid_map), drone_id_(0){}
    ~EGOReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);
    void setGridMap(const std::shared_ptr<GridMap>& grid_map);
    public:
    Eigen::Vector3d getOdomPosition() const { return odom_pos_; }
    Eigen::Vector3d getOdomVelocity() const { return odom_vel_; }
    Eigen::Vector3d getOdomAcceleration() const { return odom_acc_; }
    Eigen::Quaterniond getOdomOrientation() const { return odom_orient_; }
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void planGlobalTraj(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel, 
                        const Eigen::Vector3d& start_acc, const Eigen::Vector3d& end_pt, 
                        const Eigen::Vector3d& end_vel, const Eigen::Vector3d& end_acc);
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    void setCurrentTask(const std::string& task);
    void updateOdom(const Eigen::Vector3d & pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, const Eigen::Quaterniond& orient);
    void setDroneId(int id) { drone_id_ = id; }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif