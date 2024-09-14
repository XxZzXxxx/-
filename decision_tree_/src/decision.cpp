#include "decision_tree_/decision.h"

double missionallcost = 40.0;
std::array<Eigen::Vector3d, 16> map_position;
std::array<int, 16> occ_map;

AStar::Ptr astar = std::make_shared<AStar>();

Eigen::Vector3d choosenextwaypoint(std::map<int, Eigen::Vector3d> current_position, int id, AStar::Ptr astar) 
{
    Eigen::Vector3d next_position = current_position[id];
    double min_cost = std::numeric_limits<double>::max();
    int min_index = -1;
    


    for(int i = 0; i < 16; i++) {
        if(occ_map[i] == 0 && (map_position[i] - current_position[id]).norm() < 5.3) {
            double cost = astar->calculatePathCost(current_position[id], map_position[i]);
            if(cost < min_cost) {
                min_cost = cost;
                min_index = i;
                next_position = map_position[i];
            }
        }  
    }
        
        if(min_index != -1){
            occ_map[min_index] = 1;
        }

    return next_position;

}

TASK_NODE::TASK_NODE(int virtual_target, int true_target, int drone_num, AStar::Ptr astar_ptr)
    : virtual_target_(virtual_target), true_target_(true_target), drone_num(drone_num), astar(astar_ptr) {
    for (int i = 0; i < drone_num; i++) {
        task_cost_[i] = {{SEARCH, 1.0}, {CHECK, 2.0}, {ATTACK, 3.0}};
        action_cost_[i] = {{SEARCH, 5.0}, {CHECK, 100.0}, {ATTACK, 100.0}};
        task_bonus_[i] = {{SEARCH, 2.5}, {CHECK, 10.0}, {ATTACK, 10.0}};
    }
    decided_drones_simulations.virtual_targets = 0;
    decided_drones_simulations.true_targets = 0;
}


void TASK_NODE::calculate_cost(int drone_id) {
    for (int i = 0; i < 3; i++) {

        if (precon(drone_id, i, decided_drones_simulations.virtual_targets, decided_drones_simulations.true_targets)) {
                double task_execution_cost = task_cost_[drone_id][i];
                double action_cost = 0.0;
                Eigen::Vector3d target_position;

            switch (i) {
                    case SEARCH:
                        target_position = getNextSearchPosition(drone_id);
                        if(target_position == current_position[drone_id]){
                            action_cost = std::numeric_limits<double>::max();
                        }else{
                            action_cost = astar->calculatePathCost(current_position[drone_id], target_position);
                        }
                        break;
                    case CHECK:
                        std::tie(target_position, action_cost) = getBestTarget(drone_id, TargetType::VIRTUAL);
                        selected_targets[drone_id][CHECK] = target_position;
                        break;
                    case ATTACK:
                        std::tie(target_position, action_cost) = getBestTarget(drone_id, TargetType::TRUE);
                        selected_targets[drone_id][ATTACK] = target_position;
                        break;
                }

                action_cost_[drone_id][i] = action_cost;
            }
        }

}

bool TASK_NODE::precon(int drone_id, int task_name, int simulated_virtual_target, int simulated_true_target) {
    bool precon = false;

    switch (task_name) {
        case SEARCH:
                precon = true;
            break;
        case CHECK:
           precon = (simulated_virtual_target > 0);
            break;
        case ATTACK:
            precon = (simulated_true_target > 0);
            break;
    }
    return precon;
}

void TASK_NODE::update_drone_status(int drone_id, const Eigen::Vector3d& position, const std::string& task) {
    current_position[drone_id] = position;
    current_drone_tasks[drone_id] = task;
}

void TASK_NODE::add_virtual_target(const Eigen::Vector3d& position) {
    virtual_target_positions.push_back(position);
    virtual_target_++;
}

void TASK_NODE::remove_virtual_target(const Eigen::Vector3d& position) {
    auto it = std::find(virtual_target_positions.begin(), virtual_target_positions.end(), position);
    if (it != virtual_target_positions.end()) {
        virtual_target_positions.erase(it);
        virtual_target_--;
    }
}

Eigen::Vector3d TASK_NODE::getNextSearchPosition(int drone_id) {
        return choosenextwaypoint(current_position, drone_id);
    }

std::pair<Eigen::Vector3d, double> TASK_NODE::getBestTarget(int drone_id, TargetType type) {
    const auto& target_positions = (type == TargetType::VIRTUAL) ? virtual_target_positions : true_target_positions;

    if (target_positions.empty()) {
        // 返回一个"空"位置和最大 cost
        return {Eigen::Vector3d::Zero(), std::numeric_limits<double>::max()};
    }

    Eigen::Vector3d best_target;
    double min_cost = std::numeric_limits<double>::max();

    for (const auto& target : target_positions) {
        double cost = astar->calculatePathCost(current_position.at(drone_id), target);
        if (cost < min_cost) {
            min_cost = cost;
            best_target = target;
        }
    }

    return {best_target, min_cost};
}

void TASK_NODE::update_simulation_result(int drone_id, int virtual_targets, int true_targets) {
    decided_drones_simulations.virtual_targets += virtual_targets;
    decided_drones_simulations.true_targets += true_targets;
}

SimulationResult TASK_NODE::get_current_state() const {
    return {decided_drones_simulations.virtual_targets, decided_drones_simulations.true_targets};
}

DecisionTree::DecisionTree(int id) : drone_id(id), gen(std::random_device{}()) {
    reset_decision_state();
}

void DecisionTree::three_layer_decision(const TASK_NODE::Ptr& task_node, const SimulationResult& current_state) {
    FirstDecision best_decision = backtrack(0, missionallcost, current_state, task_node);
    
    if (best_decision.task == -1) {
            ROS_WARN("Decision making failed: No valid decision found.");
            // 在这里可以添加额外的失败处理逻辑
            return;
        }

    best_first_layer_task = best_decision.task;
    best_first_layer_fscore = best_decision.cost;
    current_task = task_node->task_name_[best_decision.task];
    has_decided = true;
    has_executed_first_layer = false;
}

void DecisionTree::first_layer_decision(TASK_NODE::Ptr& task_node, const SimulationResult& current_state){
    double best_cost = std::numeric_limits<double>::max();
    int best_task = -1;

    for (int task = 0; task < 3; ++task) {

        if (!task_node->precon(drone_id, task, current_state.virtual_targets, current_state.true_targets)) {
            continue;
        }

        double task_cost = simulate_mission_cost(task, drone_id, task_node);

        if (task_cost < best_cost) {
            best_cost = task_cost;
            best_task = task;
        }
    }

    best_first_layer_task = best_task;
    best_first_layer_fscore = best_cost;

}

DecisionTree::FirstDecision DecisionTree::backtrack(int depth, double current_cost, const SimulationResult& current_state, const TASK_NODE::Ptr& task_node) {
    if (depth == 3) {
        return FirstDecision{-1, current_cost, current_state.virtual_targets, current_state.true_targets};
    }

    FirstDecision best_path{-1, std::numeric_limits<double>::max(), 0, 0};

    for (int task = 0; task < 3; ++task) {
        if (!task_node->precon(drone_id, task, current_state.virtual_targets, current_state.true_targets)) {
            continue;
        }//检查任务前置条件

        double task_cost = simulate_mission_cost(task, drone_id, task_node, current_cost);
        int new_virtual_target = simulate_virtual_target(task, drone_id, current_state.virtual_targets);
        int new_true_target = simulate_true_target(task, drone_id, current_state.true_targets);

        if(task_cost>=best_path.cost){
            continue;
        }//减枝策略

        SimulationResult new_state = current_state + {new_virtual_target, new_true_target};
        FirstDecision sub_path = backtrack(depth + 1, task_cost, new_state, task_node);

        if (sub_path.cost < best_path.cost) {
            best_path = sub_path;
            if (depth == 0) {
                best_path.task = task;
            }
        }
    }


    return best_path;
}

double DecisionTree::simulate_mission_cost(int task, int drone_id, const TASK_NODE::Ptr& task_node, double current_cost) {
    if (current_cost == -1) current_cost = missionallcost;

    if (!task_node->precon(drone_id, task, current_state.virtual_targets, current_state.true_targets)) {
            // 如果前置条件不满足，返回一个非常大的成本值
            return std::numeric_limits<double>::max();
        }

    task_node->calculate_cost(drone_id);
    double simulated_action_cost = task_node->action_cost_[drone_id][task];
    double simulated_task_cost = task_node->task_cost_[drone_id][task];

    return current_cost - task_node->task_bonus_[drone_id][task] + simulated_action_cost + simulated_task_cost;
}

int DecisionTree::simulate_virtual_target(int task, int drone_id, int current_virtual_target) {
    std::uniform_int_distribution<> dis(0, 1);
    bool success = dis(gen) > 0;//悲观预测
    
    if (task == TASK_NODE::SEARCH && success) {
        return current_virtual_target + 1;
    } else if (task == TASK_NODE::CHECK && success) {
        return std::max(0, current_virtual_target - 1);
    }
    return current_virtual_target;
}

int DecisionTree::simulate_true_target(int task, int drone_id, int current_true_target) {
    std::uniform_int_distribution<> dis(0, 1);
    bool success = dis(gen) > 1;//悲观预测

    if (task == TASK_NODE::CHECK && success) {
        return current_true_target + 1;
    } else if (task == TASK_NODE::ATTACK && success) {
        return std::max(0, current_true_target - 1);
    }
    return current_true_target;
}

void DecisionTree::apply_first_layer_decision() {
    has_executed_first_layer = true;
}//需要添加发布任务出去的代码

void DecisionTree::update_status(Eigen::Vector3d position) {
    current_position_ = position;
}

void DecisionTree::reset_decision_state() {
    has_decided = false;
    has_executed_first_layer = false;
    current_task = "search";
    best_first_layer_task = -1;
    best_first_layer_fscore = std::numeric_limits<double>::max();
}

Decision::Decision(ros::NodeHandle &nh) : is_decision_running(false) {
    initDecision(nh);
}

Decision::~Decision() {
    if (decision_thread.joinable()) {
        decision_thread.join();
    }
}

void Decision::start_decision_making() {
    if (decision_thread.joinable()) {
        decision_thread.join();
    }
    is_decision_running = true;
    decision_thread = std::thread(&Decision::decision_thread_function, this);
}

void Decision::decision_thread_function() {
    decision_making();
    {
        std::lock_guard<std::mutex> lock(decision_mutex);
        is_decision_running = false;
    }
    cv.notify_all();
    ROS_INFO("Decision making completed");
}

void Decision::decision_making() {
    if(!allAreasExplored()){
    std::lock_guard<std::mutex> lock(task_node_mutex);
    ROS_INFO("Starting decision making process");
    // 重置任务完成状态
    std::fill(task_completed.begin(), task_completed.end(), false);

    if (decision_trees.empty()) {
        ROS_WARN("No decision trees available. Check initialization.");
        return;
    }

    // 重置决策状态和已决策飞机集合
    for (auto& tree : decision_trees) {
        tree->reset_decision_state();
    }
    decided_drones.clear();
    reset_simulation_results();
    for (int i = 0; i < drone_num; ++i) {
        task_node->selected_targets[i].clear();
    }

    SimulationResult current_state = task_node->get_actual_state();
    
    // 决策过程
    while (decided_drones.size() < decision_trees.size()) {
        calculate_priorities();
        DecisionTree::Ptr next_drone = get_highest_priority_drone();
        if (!next_drone) {
            ROS_WARN("Unable to find next priority drone. This should not happen.");
            break;
        }

        next_drone->three_layer_decision(task_node, current_state);
        decided_drones.insert(next_drone->getid());

        int simulated_virtual_targets = next_drone->simulate_virtual_target(next_drone->best_first_layer_task, next_drone->getid(), current_state.virtual_targets);
        int simulated_true_targets = next_drone->simulate_true_target(next_drone->best_first_layer_task, next_drone->getid(), current_state.true_targets);
        task_node->update_simulation_result(next_drone->getid(), simulated_virtual_targets, simulated_true_targets);
        
        // 更新当前状态
        current_state = task_node->get_current_state();
    }
    // 执行任务
    execute_tasks();
    }else{
        ROS_INFO("All areas explored. Mission completed.");
    }

}

void Decision::initDecision(ros::NodeHandle &nh) {
    nh_ = nh;
    nh.param<int>("/decision_tree/manager/drone_num", drone_num, 1);
    nh_.param("/decision_tree/completion_threshold", completion_threshold, 0.5);

    // 初始化网格地图
    grid_map_ = std::make_shared<GridMap>();
    grid_map_->initMap(nh);
    grid_map_->divideMapAndFindCenters(); 

    // 获取地图信息
    map_position = grid_map_->get_map_position();
    occ_map = grid_map_->get_occ_map();
    
    task_node = std::make_shared<TASK_NODE>(0, 0, drone_num, std::make_shared<AStar>());
    ego_planners.reserve(drone_num);
    decision_trees.reserve(drone_num);
    odom_subs.resize(drone_num);
    for (int i = 0; i < drone_num; i++) {
        decision_trees.push_back(std::make_shared<DecisionTree>(i));
        auto planner = std::make_unique<ego_planner::EGOReplanFSM>();
        planner->init(nh);
        planner->planner_manager_->grid_map_ = grid_map_;
        ego_planners.push_back(std::move(planner));
        
        std::string topic = "/drone_" + std::to_string(i) + "/odom_world";
        odom_subs[i] = nh_.subscribe<nav_msgs::Odometry>(topic, 10, boost::bind(&Decision::odomCallback, this, _1, i));
    }
    
    task_completed.resize(drone_num, false);

    task_result_sub = nh_.subscribe("/task_result", 10, &Decision::taskResultCallback, this);
    bspline_pub = nh_.advertise<ego_planner::Bspline>("/planning/bspline", 10);

    // decision_timer_ = nh.createTimer(ros::Duration(0.1), &Decision::decision_making, this);
}

void Decision::calculate_priorities() {
    SimulationResult current_state = task_node->get_current_state();
    for (auto& tree : decision_trees) {
        if (decided_drones.find(tree->getid()) == decided_drones.end()) {
            tree->first_layer_decision(task_node, current_state);
        }
    }
}

DecisionTree::Ptr Decision::get_highest_priority_drone() {
    DecisionTree::Ptr best_drone = nullptr;
    double highest_priority = std::numeric_limits<double>::lowest();

    for (auto& tree : decision_trees) {
        if (decided_drones.find(tree->getid()) == decided_drones.end()) {
            double priority = tree->get_priority();
            if (priority > highest_priority) {
                highest_priority = priority;
                best_drone = tree;
            }
        }
    }

    return best_drone;
}

void Decision::execute_tasks() {
   for (int i = 0; i < drone_num; i++) {
            auto& decision_tree = decision_trees[i];
            if (decision_tree->has_decided && !decision_tree->has_executed_first_layer) {
                int task = decision_tree->best_first_layer_task;
                
                // 获取起点和终点
                Eigen::Vector3d start_pt = ego_planners[i]->odom_pos_;
                Eigen::Vector3d start_vel = ego_planners[i]->odom_vel_;
                Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();
                Eigen::Vector3d end_pt = task_node->selected_targets[i][task];
                Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();

                // 使用EGO Planner生成轨迹
                ego_planners[i]->planGlobalTraj(start_pt, start_vel, start_acc, end_pt, end_vel, end_vel);

                // 调用EGO Planner的重新规划功能来生成和发布轨迹
                bool success = ego_planners[i]->callReboundReplan(false, false);
                if (success) {
                ROS_INFO("Successfully planned and published trajectory for drone %d", i);
                } else {
                ROS_WARN("Failed to plan trajectory for drone %d", i);
                }
                // 标记任务已执行
                decision_tree->apply_first_layer_decision();
                // 更新无人机状态
                task_node->update_drone_status(i, end_pt, decision_tree->current_task);
            }
        }
}//需要添加发布任务出去的代码

void Decision::update_status(int id, Eigen::Vector3d position) {
    for (auto& decision_tree : decision_trees) {
        if (decision_tree->getid() == id) {
            decision_tree->update_status(position);
            task_node->update_drone_status(id, position, decision_tree->current_task);
            break;
        }
    }
}


void Decision::odomCallback(const nav_msgs::OdometryConstPtr &msg, int drone_id) {
    std::lock_guard<std::mutex> lock(task_node_mutex);
    if (!msg) {
        ROS_WARN("Received null pointer in odomCallback for drone %d", drone_id);
        return;
    }
    ego_planners[drone_id]->odometryCallback(msg);
    
    Eigen::Vector3d current_pos = ego_planners[drone_id]->odom_pos_;
    update_status(drone_id, current_pos);

    // 检查当前任务是否完成
    if (!task_completed[drone_id] && !task_node->selected_targets[drone_id].empty()) {
        int current_task = decision_trees[drone_id]->best_first_layer_task;
        Eigen::Vector3d target_pos = task_node->selected_targets[drone_id][current_task];
        
        if ((current_pos - target_pos).norm() < completion_threshold) {
            // 任务完成
            task_completed[drone_id] = true;
           // 检查是否所有无人机都完成了任务
            if (std::all_of(task_completed.begin(), task_completed.end(), [](bool v) { return v; })) {
            // 所有无人机都完成了任务，触发重新规划
            replan();
            }
        } else {
            // 任务未完成，继续执行当前任务
            continue_current_task(drone_id, current_pos, target_pos);
        }
    }
    if (check_replanning_condition()) {
        replan();
    }

}

void Decision::taskResultCallback(const your_package::TaskResultConstPtr& msg) {
    std::lock_guard<std::mutex> lock(task_node_mutex);
    
    TaskResult result;
    result.drone_id = msg->drone_id;
    result.task_type = msg->task_type;
    result.position = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);

    update_task_results(result.drone_id, result.task_type, result.position);
    // 任务结果更新后立即重新规划
    replan();
}

bool Decision::check_replanning_condition() {
    // ros::Time current_time = ros::Time::now();
    // ros::Duration time_since_last_planning = current_time - last_planning_time;

    // 如果距离上次规划超过5秒，就重新规划
    if (time_since_last_planning.toSec() > 5.0) {
        return true;
    }

    // 简单示例：如果任何无人机完成了任务，就重新规划
    // for (bool completed : task_completed) {
    //     if (completed) return true;
    // }

    //示例条件：如果发现新的虚拟目标或真实目标，触发重新规划
    static int last_virtual_target_count = task_node->virtual_target_;
    static int last_true_target_count = task_node->true_target_;

    bool new_targets_found = (task_node->virtual_target_ > last_virtual_target_count) || 
                             (task_node->true_target_ > last_true_target_count);

    last_virtual_target_count = task_node->virtual_target_;
    last_true_target_count = task_node->true_target_;

    return new_targets_found;

    return false;
}

void Decision::replan() {
    ROS_INFO("Starting replanning process");
    std::lock_guard<std::mutex> lock(decision_mutex);
    if (is_decision_running) {
        ROS_WARN("Decision making is already running. Skipping this replan request.");
        return;
    }
    // Reset decision states
    for (auto& decision_tree : decision_trees) {
        decision_tree->reset_decision_state();
    }
    decided_drones.clear();
    // Recalculate task costs
    for (int i = 0; i < drone_num; ++i) {
        task_node->calculate_cost(i);
    }
    
    start_decision_making();
    last_planning_time = ros::Time::now();
}

void Decision::continue_current_task(int drone_id, const Eigen::Vector3d& current_pos, const Eigen::Vector3d& target_pos) {
    // 获取当前速度和加速度
    Eigen::Vector3d current_vel = ego_planners[drone_id]->odom_vel_;
    Eigen::Vector3d current_acc = Eigen::Vector3d::Zero(); // 假设当前加速度为零

    // 目标速度设为零
    Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();

    // 使用EGO Planner重新规划轨迹
    ego_planners[drone_id]->planGlobalTraj(current_pos, current_vel, current_acc, target_pos, target_vel, target_vel);

    // 调用EGO Planner的重新规划功能来生成和发布轨迹
    bool success = ego_planners[drone_id]->callReboundReplan(false, false);
    if (success) {
        ROS_INFO("Successfully replanned trajectory for drone %d to continue current task", drone_id);
    } else {
        ROS_WARN("Failed to replan trajectory for drone %d to continue current task", drone_id);
    }
}

void Decision::update_task_results(int drone_id, const std::string& task_result, const Eigen::Vector3d& position) {
    task_node->update_drone_status(drone_id, position, task_node->current_drone_tasks[drone_id]);

    if (task_result == "found_virtual_target") {
        task_node->add_virtual_target(position);
    } else if (task_result == "confirmed_real_target") {
        task_node->true_target_++;
        task_node->remove_virtual_target(position);
    } else if (task_result == "eliminated_target") {
        task_node->true_target_--;
    }

    task_node->calculate_cost(drone_id);
}

void Decision::reset_simulation_results() {
    task_node->decided_drones_simulations = SimulationResult{
        task_node->virtual_target_,  // 保留实际虚拟目标数量
        task_node->true_target_,     // 保留实际真实目标数量
    };
}

bool Decision::allAreasExplored() {
    // 检查是否所有区域都被探索
    // 这里的实现取决于如何定义"探索完成"
    for (int occupied : grid_map_->get_occ_map()) {
        if (occupied == 0) return false;
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "decision_tree");
    ros::NodeHandle nh;
    Decision decision(nh);
    decision.start_decision_making();
    ros::spin();
    return 0;
}

