#include "decision_tree_/decision.h"

double missionallcost = 40.0;
std::array<Eigen::Vector3d, 16> map_position;
int occ_map[16];

AStar::Ptr astar = std::make_shared<AStar>();

void initSearchMap()
{
   id_init[0] = false;
    id_init[1] = false;
    id_init[2] = false;
    for(int i=-1; i<3;i++){
      for (int j=-1; j<3; j++){
        Eigen::Vector3d pos;
        pos << i*5.0, j*5.0, 1.0 ;
        map_position[i*4+j+5] = pos;
        occ_map[i*4+j+5] = 0;    
      }
    }
}//需要外接来初始化地图

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
    decided_drones_simulations.virtual_targets = virtual_target_;
    decided_drones_simulations.true_targets = true_target_;
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
                        std::tie(target_position, action_cost) = getBestVirtualTarget(drone_id);
                        selected_targets[drone_id][CHECK] = target_position;
                        break;
                    case ATTACK:
                        std::tie(target_position, action_cost) = getBestTrueTarget(drone_id);
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

std::pair<Eigen::Vector3d, double> TASK_NODE::getBestVirtualTarget(int drone_id){

    if (virtual_target_positions.empty()) {
            // 返回一个"空"位置和最大 cost
            return {Eigen::Vector3d::Zero(), std::numeric_limits<double>::max()};
        }

    Eigen::Vector3d best_target;
    double min_cost = std::numeric_limits<double>::max();
    for (const auto& target : virtual_target_positions) {
            double cost = astar->calculatePathCost(current_position.at(drone_id), target);
            if (cost < min_cost) {
                min_cost = cost;
                best_target = target;
            }
        }
    return {best_target, min_cost};
}

std::pair<Eigen::Vector3d, double> TASK_NODE::getBestTrueTarget(int drone_id){

    if (true_target_positions.empty()) {
            // 返回一个"空"位置和最大 cost
            return {Eigen::Vector3d::Zero(), std::numeric_limits<double>::max()};
        }

    Eigen::Vector3d best_target;
    double min_cost = std::numeric_limits<double>::max();
    for (const auto& target : true_target_positions) {
        double cost = astar->calculatePathCost(current_position[drone_id], target);
        if (cost < min_cost) {
                min_cost = cost;
                best_target = target;
            }
        }
    return {best_target, min_cost};
}

void TASK_NODE::update_simulation_result(int drone_id, int virtual_targets, int true_targets) {
    decided_drones_simulations += {virtual_targets, true_targets};
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

Decision::Decision(ros::NodeHandle &nh) {
    initDecision(nh);
    task_result_sub = nh.subscribe("/task_result", 10, &Decision::task_result_callback, this);//订阅任务结果信息
    replanning_needed = false;
}

void Decision::decision_making(const ros::TimerEvent &e) {
    std::cout << "decision_making" << std::endl;

    ROS_INFO_STREAM("Number of decision trees: " << decision_trees.size());

    if (decision_trees.empty()) {
        ROS_WARN("No decision trees available. Check initialization.");
        return;
    }

    if (replanning_needed || check_replanning_condition()) {
        replan();
    }//需要放在外面充当外部接口判断是否重规划

    // 重置决策状态和已决策飞机集合
    for (auto& tree : decision_trees) {
        tree->reset_decision_state();
    }
    decided_drones.clear();
    reset_simulation_results();

    // 决策过程
    while (decided_drones.size() < decision_trees.size()) {
        calculate_priorities();
        DecisionTree::Ptr next_drone = get_highest_priority_drone();
        if (!next_drone) {
            ROS_WARN("Unable to find next priority drone. This should not happen.");
            break;
        }

        next_drone->three_layer_decision(task_node);
        decided_drones.insert(next_drone->getid());

        SimulationResult current_state = task_node->get_current_state();
        next_drone->three_layer_decision(task_node, current_state);
        decided_drones.insert(next_drone->getid());

        // Update task node with simulated results
        int simulated_virtual_targets = next_drone->simulate_virtual_target(next_drone->best_first_layer_task, next_drone->getid(), current_state.virtual_targets);
        int simulated_true_targets = next_drone->simulate_true_target(next_drone->best_first_layer_task, next_drone->getid(), current_state.true_targets);
        task_node->update_simulation_result(next_drone->getid(), simulated_virtual_targets, simulated_true_targets);
    }

    // 执行任务
    execute_tasks();

    // 发布目标点和显示信息
    target_pub.publish(target_point);
    rviz_display();
}

void Decision::initDecision(ros::NodeHandle &nh) {
    nh_ = nh;
    nh.param("/decision_tree/manager/drone_num", drone_num, 1);
    
    task_node = std::make_shared<TASK_NODE>(0, 0, drone_num, std::make_shared<AStar>());
    
    for (int i = 0; i < drone_num; i++) {
        decision_trees.push_back(std::make_shared<DecisionTree>(i));
    }
    
    decision_timer_ = nh.createTimer(ros::Duration(0.1), &Decision::decision_making, this);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);//
    marker_pub2 = nh.advertise<visualization_msgs::Marker>("/visualization_marker2", 10);
    drone_task_pub = nh.advertise<diagnostic_msgs::KeyValue>("/drone_tasks", 10);

    target_sub = nh.subscribe("/target_position", 10, &Decision::targetCallback, this);//订阅真实信息(例如虚拟目标或者真实目标的坐标)
    idodom_sub = nh.subscribe("/drone_planning/odom", 10, &Decision::IdOdomCallback, this, ros::TransportHints().tcpNoDelay());//订阅无人机的当前位置状态
    target_pub = nh.advertise<geometry_msgs::Point>("/target_position", 10);
    id_target_pub = nh.advertise<traj_utils::IdOdom>("/id_target", 10);

    initSearchMap();
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
    for (const auto& decision_tree : decision_trees) {
        if (decision_tree->has_decided && !decision_tree->has_executed_first_layer) {
            traj_utils::IdOdom id_target;
            Eigen::Vector3d next_position;
            
            int drone_id = decision_tree->getid();
            int task = decision_tree->best_first_layer_task;
            
            if (task == TASK_NODE::SEARCH) {
                next_position = task_node->getNextSearchPosition(drone_id);
            } else if (task == TASK_NODE::CHECK || task == TASK_NODE::ATTACK) {
                next_position = task_node->selected_targets[drone_id][task];
            }
            
            id_target.drone_id = drone_id;
            id_target.pose.x = next_position[0];
            id_target.pose.y = next_position[1];
            id_target.pose.z = next_position[2];
            id_target_pub.publish(id_target);
            
            decision_tree->apply_first_layer_decision();
            task_node->update_drone_status(drone_id, next_position, decision_tree->current_task);
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

void Decision::IdOdomCallback(const traj_utils::IdOdomConstPtr &msg) {
    if (!msg) {
        ROS_WARN("Received null pointer in IdOdomCallback");
        return;
    }
    update_status(msg->drone_id, Eigen::Vector3d(msg->pose.x, msg->pose.y, msg->pose.z));
}

void Decision::task_result_callback(const TaskResultMsg::ConstPtr& msg) {
    ROS_INFO("Received task result for drone %d: %s", msg->drone_id, msg->task_result.c_str());

    update_task_results(msg->drone_id, msg->task_result, Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z));
    replanning_needed = true;
}

void Decision::targetCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    target_point.x = msg->point.x;
    target_point.y = msg->point.y;
    target_point.z = msg->point.z;
    target_position_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
}

bool Decision::check_replanning_condition() {
    // Implement logic to check if replanning is needed
    return false;
}

void Decision::replan() {
    // Reset decision states
    for (auto& decision_tree : decision_trees) {
        decision_tree->reset_decision_state();
    }
    
    // Recalculate task costs
    for (const auto& [drone_id, position] : task_node->current_position) {
        task_node->calculate_cost(drone_id);
    }
    
    replanning_needed = false;
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
    task_node->decided_drones_simulations.clear();
}

void Decision::rviz_display() {
    displayOccupancyMap();
    displayTargetPoint();
    displayDroneTasks();
    publishDroneTasks();
}

void Decision::displayOccupancyMap() {
    for (int i = 0; i < 16; i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "occupancy_map";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = marker.scale.y = 5.0;
        marker.scale.z = 1.0;
        marker.color.a = 0.3f;
        marker.color.r = (occ_map[i] == 1) ? 0.0f : 1.0f;
        marker.color.g = (occ_map[i] == 1) ? 1.0f : 0.0f;
        marker.color.b = 0.0f;
        marker.pose.position.x = map_position[i][0];
        marker.pose.position.y = map_position[i][1];
        marker.pose.position.z = map_position[i][2];
        marker.pose.orientation.w = 1.0;
        marker_pub2.publish(marker);
    }
}

void Decision::displayTargetPoint() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "target_point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = 0.3;
    marker.color.r = 1.0f;
    marker.color.a = 0.8f;
    geometry_msgs::Point p;
    p.x = target_position_[0];
    p.y = target_position_[1];
    p.z = target_position_[2];
    marker.points.push_back(p);
    marker_pub.publish(marker);
}//显示发现的目标点

void Decision::displayDroneTasks() {
    for (const auto& decision_tree : decision_trees) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "drone_tasks";
        marker.id = decision_tree->getid();
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = decision_tree->current_position_;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.5;
        marker.color.a = 0.8;
        marker.color.r = (marker.id == 0) ? 1.0 : 0.0;
        marker.color.g = (marker.id == 1) ? 1.0 : 0.0;
        marker.color.b = (marker.id == 2) ? 1.0 : 0.0;
        marker.text = std::to_string(marker.id) + ":" + decision_tree->get_task();
        marker_pub.publish(marker);
    }
}//显示每个无人机当前执行的任务

void Decision::publishDroneTasks() {
    for (const auto& decision_tree : decision_trees) {
        diagnostic_msgs::KeyValue msg;
        msg.key = std::to_string(decision_tree->getid());
        msg.value = decision_tree->get_task();
        drone_task_pub.publish(msg);
        ROS_INFO("Published task for drone %d: %s", decision_tree->getid(), task_status.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "decision_tree");
    ros::NodeHandle nh;
    Decision decision(nh);
    ros::spin();
    return 0;
}

