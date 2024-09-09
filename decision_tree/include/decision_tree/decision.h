#ifndef _DECISION_H_
#define _DECISION_H_

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

class TASK_NODE{
    private:
        std::string current_task;
        
        
        Eigen::Vector3d virtual_target_position_;
        Eigen::Vector3d target_position_;
        Eigen::Vector3d current_position_;
        double unknown_size;
        /* 0 : search, 1: check, 2: attack */
        std::vector<double>  task_cost;
        std::vector<bool> task_precon;//任务前置条件是否完成

        
        int true_target_;
        int virtual_target_;
        bool uncover_range_;
        bool finish_last_;
        TASK_NODE(){};

    public:
        enum{SEARCH =0, 
             CHECK=1,
             ATTACK=2
            };
        bool doing_last_task;//表示是否还在执行上一个任务？
        string task_name_[3] = {"search","check","attack"};
        TASK_NODE(int virutal,int true_target){
            true_target_ = true_target;
            virtual_target_ = virutal;
            fscore = {{SEARCH,100},{CHECK,100},{ATTACK,100}};//完成任务得分
            c_fscore =100;//当前总得分
            task_name =0;//初始化任务
            doing_last_task = false;//初始化是否执行上一次任务
        };
        std::map<int, double> fscore;
        double c_fscore;
        void calculate_cost();//计算在去执行该任务上的cost
        int task_name;
        double task_cost_;//完成任务本身需要的cost
        double action_cost_;//在去执行该任务所需要的cost
        double task_bonus_;//执行完任务对于总任务的推进奖励,假设16个位置点共有4个目标点，假设check是1贡献值，search就是1/4，attack是1
        bool precon(int task_name);//检查执行该任务的前置条件是否满足
        void is_finish_last();//检查是否完成了上一次的任务
        typedef std::shared_ptr<TASK_NODE> Ptr;

};

class DecisionTree
{
    private:
        int drone_id;
        
    public:
        int virtual_target_;
        int true_target_;
        // true target, global position
        std::vector<Eigen::Vector3d> target_;//记录多个目标位置
        // detect target
        std::string current_task="search";//初始化当前任务
        // current position of robot between the detect target
        std::vector<double> distance_current_point;//记录机器人当前位置和上一次任务应该到达的位置之间的距离
        std::vector<double> distance_target;//记录机器人当前位置和本次任务应该到达位置之间的距离
        int current_potential_point_index;
        int current_target_index;
        typedef std::shared_ptr<DecisionTree> Ptr;
        Eigen::Vector3d current_position_;//当前位置
        DecisionTree(int id){
            drone_id = id;
            true_target_ = 0;
            virtual_target_ = 0;
        };
        
        void update_dis_cur_pt(Eigen::Vector3d target_position);//更新无人当前位置和目标点之间距离
        void decision_making_node();//决策部分
        void print_task_name();
        TASK_NODE::Ptr task_node;
        
        
        void update_status(Eigen::Vector3d position);//更新无人机的当前状态
        inline int getid(void){ return drone_id;};
        inline std::string get_task(void){ return current_task;};
        inline void incre_virtual_target_() {
            std::lock_guard<std::mutex> lock(mutex_);
            ++virtual_target_;
        }//增加虚拟目标的数量，search到目标点
        inline int getCounter() {
            std::lock_guard<std::mutex> lock(mutex_);
            return virtual_target_;
        }//获取虚拟目标的数量
        inline int des_virtual_target_() {
            std::lock_guard<std::mutex> lock(mutex_);
            --virtual_target_;
        }//减少虚拟目标的数量，check完潜在目标点
        std::mutex mutex_;
};


class Decision
{
    private:
        int drone_num;//无人机数量
        ros::Timer decision_timer_;//ROS定时器
        ros::NodeHandle nh_;
        ros::Publisher marker_pub;//发布目标点
        ros::Publisher marker_pub2;//发布占用地图
        
        ros::Subscriber idodom_sub;//订阅无人机位姿，更新当前位置
        ros::Publisher target_pub;//发布目标位置
        ros::Publisher id_target_pub;//发布无人机对于目标应该有的位姿
        Eigen::Vector3d target_position_;//目标位置
        geometry_msgs::Point target_point;
    public:
        struct CompareDecisionTreePtr{
                bool operator()(const DecisionTree::Ptr& lhs, const DecisionTree::Ptr& rhs) const {
                return lhs->distance_current_point > rhs->distance_current_point;
            }
        };//需要修改优先级排序规则

        // DecistionTree::Ptr decision_tree_;
        std::priority_queue<DecisionTree::Ptr,std::vector<DecisionTree::Ptr>,CompareDecisionTreePtr> decision_queue_1, decision_queue_2;//两个优先级队列，分别用于存储和管理决策树节点的优先级
        Decision(ros::NodeHandle &nh);
        Decision(){};
        typedef std::shared_ptr<Decision> Ptr;
        
        void decision_making(const ros::TimerEvent &e);
        void initDecision(ros::NodeHandle &nh);//初始化决策系统
        void rviz_display();
        void update_status(int id, Eigen::Vector3d position);
        void IdOdomCallback(const traj_utils::IdOdomConstPtr &msg);//处理收到的位置信息，更新无人机当前位置
};

#endif
