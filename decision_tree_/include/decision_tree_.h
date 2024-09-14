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
#include <plan_manage/ego_replan_fsm.h>


extern AStar::Ptr astar;

struct TaskResult {
    int drone_id;
    std::string task_type;
    Eigen::Vector3d position;
};

struct SimulationResult {
    int virtual_targets;
    int true_targets;
};//存储模拟结果和真实结果

class TASK_NODE{
    private:
        AStar::Ptr astar;
        int drone_num;
        std::map<int, Eigen::Vector3d> current_position;//不同id飞机的当前位置
        /* 0 : search, 1: check, 2: attack */
        TASK_NODE(){};

    public:
        enum {SEARCH = 0, CHECK = 1, ATTACK = 2};
        enum class TargetType {VIRTUAL,TRUE};
        std::string task_name_[3] = {"search", "check", "attack"};
        int virtual_target_;
        int true_target_;
        std::vector<Eigen::Vector3d> virtual_target_positions;
        std::vector<Eigen::Vector3d> true_target_positions;
        std::map<int, std::string> current_drone_tasks;
        std::map<int, std::map<int, double>> task_cost_;//存储执行任务所消耗的cost
        std::map<int, std::map<int, double>> action_cost_;//存储去执行任务所消耗的cost
        std::map<int, std::map<int, double>> task_bonus_;//完成任务对于总任务的推进
        std::map<int, std::map<int, Eigen::Vector3d>> selected_targets;//存储每次决策完选择的任务目标
        SimulationResult decided_drones_simulations;

        TASK_NODE(int virtual_target, int true_target, int drone_num, AStar::Ptr astar_ptr);
        void calculate_cost(int drone_id);//计算指定无人机的任务成本
        bool precon(int drone_id, int task_name, int simulated_virtual_target, int simulated_true_target);//检查任务的前置条件是否满足(会利用到真实信息和模拟结果)
        void update_drone_status(int drone_id, const Eigen::Vector3d& position, const std::string& task);//更新指定无人机的状态（位置和任务）
        void add_virtual_target(const Eigen::Vector3d& position);//添加新的虚拟目标
        void remove_virtual_target(const Eigen::Vector3d& position);//移除指定位置的虚拟目标
        Eigen::Vector3d getNextSearchPosition(int drone_id);//获取指定无人机的下一个搜索位置
        std::pair<Eigen::Vector3d, double> getBestTarget(int drone_id, TargetType type);//获取成本最小的虚拟目标及其成本
        SimulationResult get_current_state() const;//获取当前的模拟状态（虚拟目标和真实目标数量）
        void update_simulation_result(int drone_id, int virtual_targets, int true_targets);
        SimulationResult get_actual_state() const {
        return {
            virtual_target_,
            true_target_
        };
    }
        typedef std::shared_ptr<TASK_NODE> Ptr;

};//task_node保存所有的飞机的相关信息

class DecisionTree
{
    private:
        int drone_id;//无人机ID
        Eigen::Vector3d current_position_;//当前位置
    public:
        int best_first_layer_task;//最佳第一层任务
        double best_first_layer_fscore;//最佳第一层cost
        bool has_decided;//是否做出决策
        bool has_executed_first_layer;//是否执行第一层决策（即为是否发布任务节点出去）
        std::string current_task;//当前任务


        DecisionTree(int id);

        void three_layer_decision(const TASK_NODE::Ptr& task_node, const SimulationResult& current_state);//执行三层决策过程
        void first_layer_decision(const TASK_NODE::Ptr& task_node, const SimulationResult& current_state);//执行第一层决策过程
        double simulate_mission_cost(int task, int drone_id, const TASK_NODE::Ptr& task_node, double current_cost = -1);//模拟任务成本
        int simulate_virtual_target(int task, int drone_id, int current_virtual_target);//模拟虚拟目标数量变化
        int simulate_true_target(int task, int drone_id, int current_true_target);//模拟真实目标数量变化
        void apply_first_layer_decision();//应用第一层决策结果
        void update_status(Eigen::Vector3d position);//更新无人机状态
        void reset_decision_state();//重置决策状态
        int getid() { return drone_id; }
        std::string get_task() { return current_task; }
        int get_priority() const { return -best_first_layer_fscore; }
        Eigen::Vector3d get_current_position() const { return current_position_; }
        

        typedef std::shared_ptr<DecisionTree> Ptr;
    private:

    struct FirstDecision {
        int task;
        double cost;
        int final_virtual_targets;
        int final_true_targets;
    };
    FirstDecision backtrack(int depth, double current_cost, const SimulationResult& current_state, const TASK_NODE::Ptr& task_node);// 执行回溯算法，用于多层决策
        
};


class Decision
{
    private:
        std::mutex task_node_mutex;
        int drone_num;//无人机数量
        // ros::Timer decision_timer_;//ROS定时器
        ros::NodeHandle nh_;

        std::vector<ros::Subscriber> odom_subs;//订阅无人机位姿，更新当前位置,需要修改并且加入任务结果
        std::vector<ros::Subscriber> task_result_sub;//需要自己创立一个ROS包

        std::vector<DecisionTree::Ptr> decision_trees;//决策树集合
        std::set<int> decided_drones;// 已经做出决策的无人机集合
        std::vector<std::unique_ptr<ego_planner::EGOReplanFSM>> ego_planners;// 每个无人机的EGO Planner实例
        std::shared_ptr<GridMap> grid_map_;
        // 任务完成检查
        std::vector<bool> task_completed;  // 每个无人机的任务完成状态
        double completion_threshold;  // 任务完成的阈值距离
        ros::Time last_planning_time;

        std::thread decision_thread;
        std::atomic<bool> is_decision_running;
        std::condition_variable cv;
        std::mutex decision_mutex;

    public:
        TASK_NODE::Ptr task_node;

        Decision(ros::NodeHandle &nh);
        Decision(){};
        ~Decision();
        // void decision_making(const ros::TimerEvent &e);
        void decision_making();
        void initDecision(ros::NodeHandle &nh);
        void update_status(int id, Eigen::Vector3d position);
        void odomCallback(const nav_msgs::OdometryConstPtr &msg, int drone_id);
        void taskResultCallback(const your_package::TaskResultConstPtr& msg);
        void execute_tasks();
        bool check_replanning_condition();
        void replan();
        void update_task_results(int drone_id, const std::string& task_result, const Eigen::Vector3d& position);
        void calculate_priorities();
        DecisionTree::Ptr get_highest_priority_drone();
        void reset_simulation_results();
        void continue_current_task(int drone_id, const Eigen::Vector3d& current_pos, const Eigen::Vector3d& target_pos);
        bool allAreasExplored();
        void start_decision_making();
        void decision_thread_function();
        typedef std::shared_ptr<Decision> Ptr;
};

#endif