# 无人机多任务决策系统代码解析

## TASK_NODE 类

### 成员变量

1. `astar`: A*算法指针，用于路径规划。
2. `drone_num`: 无人机数量。
3. `virtual_target_positions`: 存储虚拟目标位置的向量。
4. `true_target_positions`: 存储真实目标位置的向量。
5. `current_position`: 存储不同ID飞机的当前位置。
6. `virtual_target_`: 虚拟目标数量。
7. `true_target_`: 真实目标数量。
8. `current_drone_positions`: 存储每架无人机的当前位置。
9. `current_drone_tasks`: 存储每架无人机的当前任务。
10. `task_cost_`: 存储每个任务的成本。
11. `action_cost_`: 存储每个动作的成本。
12. `task_bonus_`: 存储每个任务的奖励。
13. `selected_targets`: 存储每架无人机选择的目标。
14. `decided_drones_simulations`: 存储已决策无人机的模拟结果。

### 成员函数

1. `TASK_NODE(int virtual_target, int true_target, int drone_num, AStar::Ptr astar_ptr)`: 
   构造函数，初始化任务节点。

2. `void calculate_cost(int drone_id)`: 
   计算指定无人机的任务成本。

3. `bool precon(int drone_id, int task_name, int simulated_virtual_target, int simulated_true_target)`: 
   检查任务的前置条件是否满足。

4. `void update_drone_status(int drone_id, const Eigen::Vector3d& position, const std::string& task)`: 
   更新指定无人机的状态（位置和任务）。

5. `void add_virtual_target(const Eigen::Vector3d& position)`: 
   添加新的虚拟目标。

6. `void remove_virtual_target(const Eigen::Vector3d& position)`: 
   移除指定位置的虚拟目标。

7. `Eigen::Vector3d getNextSearchPosition(int drone_id)`: 
   获取指定无人机的下一个搜索位置。

8. `std::pair<Eigen::Vector3d, double> getBestVirtualTarget(int drone_id)`: 
   获取成本最小的虚拟目标及其成本。

9. `std::pair<Eigen::Vector3d, double> getBestTrueTarget(int drone_id)`: 
   获取成本最小的真实目标及其成本。

10. `SimulationResult get_current_state() const`: 
    获取当前的模拟状态（虚拟目标和真实目标数量）。

## DecisionTree 类

### 成员变量

1. `drone_id`: 无人机ID。
2. `gen`: 随机数生成器。
3. `best_first_layer_task`: 最佳第一层任务。
4. `best_first_layer_fscore`: 最佳第一层得分。
5. `has_decided`: 是否已做出决策。
6. `has_executed_first_layer`: 是否已执行第一层决策。
7. `current_task`: 当前任务。
8. `current_position_`: 当前位置。

### 成员函数

1. `DecisionTree(int id)`: 
   构造函数，初始化决策树。

2. `void three_layer_decision(const TASK_NODE::Ptr& task_node, const SimulationResult& current_state)`: 
   执行三层决策过程。

3. `void first_layer_decision(const TASK_NODE::Ptr& task_node, const SimulationResult& current_state)`: 
   执行第一层决策过程。

4. `double simulate_mission_cost(int task, int drone_id, const TASK_NODE::Ptr& task_node, double current_cost = -1)`: 
   模拟任务成本。

5. `int simulate_virtual_target(int task, int drone_id, int current_virtual_target)`: 
   模拟虚拟目标数量变化。

6. `int simulate_true_target(int task, int drone_id, int current_true_target)`: 
   模拟真实目标数量变化。

7. `void apply_first_layer_decision()`: 
   应用第一层决策结果。

8. `void update_status(Eigen::Vector3d position)`: 
   更新无人机状态。

9. `void reset_decision_state()`: 
   重置决策状态。

10. `int getid()`: 
    获取无人机ID。

11. `std::string get_task()`: 
    获取当前任务。

12. `int get_priority(const TASK_NODE::Ptr& task_node) const`: 
    获取优先级（基于第一层得分）。

13. `FirstDecision backtrack(int depth, double current_cost, const SimulationResult& current_state, const TASK_NODE::Ptr& task_node)`: 
    执行回溯算法，用于多层决策。

## Decision 类

### 成员变量

1. `drone_num`: 无人机数量。
2. `decision_timer_`: ROS定时器。
3. `nh_`: ROS节点句柄。
4. `marker_pub`: 发布目标点的ROS发布器。
5. `marker_pub2`: 发布占用地图的ROS发布器。
6. `idodom_sub`: 订阅无人机位姿的ROS订阅器。
7. `target_sub`: 订阅目标信息的ROS订阅器。
8. `target_pub`: 发布目标位置的ROS发布器。
9. `id_target_pub`: 发布无人机对目标应有位姿的ROS发布器。
10. `drone_task_pub`: 发布每个飞机任务的ROS发布器。
11. `task_result_sub`: 订阅任务完成信息的ROS订阅器。
12. `target_position_`: 目标位置。
13. `target_point`: 目标点。
14. `replanning_needed`: 是否需要重新规划的标志。
15. `decision_trees`: 决策树集合。
16. `decided_drones`: 已决策的无人机集合。
17. `task_node`: 任务节点指针。

### 成员函数

1. `Decision(ros::NodeHandle &nh)`: 
   构造函数，初始化决策系统。

2. `void decision_making(const ros::TimerEvent &e)`: 
   主要决策流程，定期执行。

3. `void initDecision(ros::NodeHandle &nh)`: 
   初始化决策系统，设置ROS订阅器和发布器。

4. `void rviz_display()`: 
   在RViz中显示相关信息。

5. `void update_status(int id, Eigen::Vector3d position)`: 
   更新指定无人机的状态。

6. `void IdOdomCallback(const traj_utils::IdOdomConstPtr &msg)`: 
   处理无人机位置信息的回调函数。

7. `void task_result_callback(const TaskResultMsg::ConstPtr& msg)`: 
   处理任务结果的回调函数。

8. `void targetCallback(const geometry_msgs::PointStamped::ConstPtr& msg)`: 
   处理目标信息的回调函数。

9. `void execute_tasks()`: 
   执行已决策的任务。

10. `bool check_replanning_condition()`: 
    检查是否需要重新规划。

11. `void replan()`: 
    执行重新规划。

12. `void update_task_results(int drone_id, const std::string& task_result, const Eigen::Vector3d& position)`: 
    更新任务结果。

13. `void calculate_priorities()`: 
    计算无人机的优先级。

14. `DecisionTree::Ptr get_highest_priority_drone()`: 
    获取优先级最高的无人机。

15. `void displayOccupancyMap()`: 
    显示占用地图。

16. `void displayTargetPoint()`: 
    显示目标点。

17. `void displayDroneTasks()`: 
    显示每个无人机的任务。

18. `void publishDroneTasks()`: 
    发布每个无人机的任务。

19. `void reset_simulation_results()`: 
    重置模拟结果。

## 设计目的

这个系统的设计目的是为多个无人机在复杂环境中进行任务规划和决策。主要特点包括：

1. 多层决策：使用决策树和回溯算法进行多层决策，考虑了任务成本、奖励和前置条件等因素。
2. 动态调整：能够根据实时情况动态调整决策，处理任务结果，并在必要时进行重新规划。
3. 优先级管理：通过计算优先级来决定无人机的决策顺序，确保资源的有效分配。
4. 任务模拟：使用模拟来预测不同决策的可能结果，以做出更好的决策。
5. 可视化：提供了可视化功能，便于监控和调试整个系统的运行状态。
6. ROS集成：系统与ROS（机器人操作系统）紧密集成，便于与其他机器人系统模块交互。

总的来说，这个系统旨在提供一个灵活、高效和可扩展的解决方案，用于多无人机协同任务的决策和管理。