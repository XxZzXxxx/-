#ifndef DECISION_TREE_H
#define DECISION_TREE_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>
#include <memory>
#include "visualization_msgs/Marker.h" // zx-todo
#include "traj_utils/IdOdom.h"
#include <geometry_msgs/Point.h>
#include <mutex>
#include <thread>
#include "path_searching/dyn_a_star.h"


struct TaskResultMsg {
    int drone_id;
    std::string task_result;
    geometry_msgs::Point position;
    typedef boost::shared_ptr<TaskResultMsg const> ConstPtr;
};//订阅任务消息的结构体

struct SimulationResult {
    int virtual_targets;
    int true_targets;
};

class TASK_NODE{
    private:
        AStar::Ptr astar;
        int drone_num;
        std::vector<Eigen::Vector3d> virtual_target_positions;
        std::vector<Eigen::Vector3d> true_target_positions;
        std::map<int, Eigen::Vector3d> current_position;//不同id飞机的当前位置
        /* 0 : search, 1: check, 2: attack */
        TASK_NODE(){};

    public:
        enum {SEARCH = 0, CHECK = 1, ATTACK = 2};
        std::string task_name_[3] = {"search", "check", "attack"};
        int virtual_target_;
        int true_target_;
        std::map<int, Eigen::Vector3d> current_drone_positions;
        std::map<int, std::string> current_drone_tasks;
        std::map<int, std::map<int, double>> task_cost_;
        std::map<int, std::map<int, double>> action_cost_;
        std::map<int, std::map<int, double>> task_bonus_;
        std::map<int, std::map<int, Eigen::Vector3d>> selected_targets;
        std::map<int, SimulationResult> decided_drones_simulations;

        TASK_NODE(int virtual_target, int true_target, int drone_num, AStar::Ptr astar_ptr);
        void calculate_cost(int drone_id);
        bool precon(int drone_id, int task_name, int simulated_virtual_target, int simulated_true_target);
        void update_drone_status(int drone_id, const Eigen::Vector3d& position, const std::string& task);
        void add_virtual_target(const Eigen::Vector3d& position);
        void remove_virtual_target(const Eigen::Vector3d& position);
        Eigen::Vector3d getNextSearchPosition(int drone_id);
        std::pair<Eigen::Vector3d, double> getBestVirtualTarget(int drone_id);//找到cost最小的check点
        std::pair<Eigen::Vector3d, double> getBestTrueTarget(int drone_id);//找到cost最小的attack点
        SimulationResult get_current_state() const;

        typedef std::shared_ptr<TASK_NODE> Ptr;

};//task_node保存所有的飞机的相关信息

class DecisionTree
{
    private:
        int drone_id;
        std::mt19937 gen;
        
    public:
        int best_first_layer_task;
        double best_first_layer_fscore;
        bool has_decided;
        bool has_executed_first_layer;
        std::string current_task;
        Eigen::Vector3d current_position_;


        DecisionTree(int id);

        void three_layer_decision(const TASK_NODE::Ptr& task_node, const SimulationResult& current_state);
        void first_layer_decision(const TASK_NODE::Ptr& task_node, const SimulationResult& current_state);
        double simulate_mission_cost(int task, int drone_id, const TASK_NODE::Ptr& task_node, double current_cost = -1);
        int simulate_virtual_target(int task, int drone_id, int current_virtual_target);
        int simulate_true_target(int task, int drone_id, int current_true_target);
        void apply_first_layer_decision();
        void update_status(Eigen::Vector3d position);
        void reset_decision_state();
        int getid() { return drone_id; }
        std::string get_task() { return current_task; }
        int get_priority(const TASK_NODE::Ptr& task_node) const { return -best_first_layer_fscore; }
        

        typedef std::shared_ptr<DecisionTree> Ptr;
    private:

    struct FirstDecision {
        int task;
        double cost;
        int final_virtual_targets;
        int final_true_targets;
    };
    FirstDecision backtrack(int depth, double current_cost, const SimulationResult& current_state, const TASK_NODE::Ptr& task_node);
        
};


class Decision
{
    private:
        int drone_num;//无人机数量
        ros::Timer decision_timer_;//ROS定时器
        ros::NodeHandle nh_;
        ros::Publisher marker_pub;//发布目标点
        ros::Publisher marker_pub2;//发布占用地图，这两个是用于rviz？
        
        ros::Subscriber idodom_sub;//订阅无人机位姿，更新当前位置
        ros::Subscriber target_sub;//订阅目标信息
        ros::Publisher target_pub;//发布目标位置，用于rviz可视化
        ros::Publisher id_target_pub;//发布无人机对于目标应该有的位姿
        ros::Publisher drone_task_pub; //发布每个飞机的任务
        ros::Subscriber task_result_sub;//订阅任务完成信息
        Eigen::Vector3d target_position_;
        geometry_msgs::Point target_point;
        bool replanning_needed;
        std::vector<DecisionTree::Ptr> decision_trees;
        std::set<int> decided_drones;

    public:
        TASK_NODE::Ptr task_node;

        Decision(ros::NodeHandle &nh);
        Decision(){};
        void decision_making(const ros::TimerEvent &e);
        void initDecision(ros::NodeHandle &nh);
        void rviz_display();
        void update_status(int id, Eigen::Vector3d position);
        void IdOdomCallback(const traj_utils::IdOdomConstPtr &msg);
        void task_result_callback(const TaskResultMsg::ConstPtr& msg);
        void targetCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void execute_tasks();
        bool check_replanning_condition();
        void replan();
        void update_task_results(int drone_id, const std::string& task_result, const Eigen::Vector3d& position);
        void calculate_priorities();
        DecisionTree::Ptr get_highest_priority_drone();
        void displayOccupancyMap();
        void displayTargetPoint();
        void displayDroneTasks();
        void publishDroneTasks();
        void reset_simulation_results();
        typedef std::shared_ptr<Decision> Ptr;

};

#endif