#include "decision_tree/decision.h"


// sanyuan
// std::vector<Eigen::Vector3d> agent_pos_map_;
double missionallcost = 40.0;//完成最终任务所需要的cost
Eigen::Vector3d map_position[16];
int occ_map[16]; //表示位置是否被覆盖
std::map<int,bool> id_init;
std::map<int,Eigen::Vector3d> last_pos;//记录无人机last pos
// static std::map<int,Eigen::Vector3d> id_init;
std::vector<Eigen::Vector3d> getWaypoints;
static bool covered=false;
void initSearchMap(){
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
}//初始化位置点位置

Eigen::Vector3d choosenextwaypoint(Eigen::Vector3d current_position,int id){
    if(id_init[id]==false){
        last_pos[id] = current_position;   
    }//若无人机还未初始化，则将当前位置当作last pos
    Eigen::Vector3d next_position= last_pos[id];//不懂，如果是初始化为什么不直接设成0
    // static int num =0;
    static int count = 0;
    // std::cout << "current_position: "<<current_position<<std::endl;
    if(id_init[id] == false){
        double distance = 100.0;
        int min = 0;        
        for (int j=0; j<16; j++){
            Eigen::Vector3d pos;
            if(distance > (map_position[j]- current_position).norm()&& occ_map[j] == 0){
                distance = (map_position[j]- current_position).norm();
                
                next_position = map_position[j];
                last_pos[id] = next_position;
                min = j;
            
            }    
        }//找没有被占据的邻近节点
        occ_map[min] = 1;
        id_init[id] = true;

        std::cout << "init_position: "<<next_position<<std::endl;
        return next_position;
    }//未初始化的寻找最近的可被search的位置点
    if((current_position - last_pos[id]).norm() > 0.2){
        return last_pos[id];
    }//若飞机距离上一次完成的位置点相隔过远则判定为任务未完成
    std::cout << "occ_map: "<<occ_map[0]<<";"<<occ_map[1]<<";"<<occ_map[2]<<";"
              <<occ_map[3]<<";"<<occ_map[4]<<";"<<occ_map[5]<<";"<<occ_map[6]<<";"
              <<occ_map[7]<<";"<<occ_map[8]<<std::endl;//不懂
    for(int i=0;i<16;i++){
      if(occ_map[i]==0 && (map_position[i] - current_position).norm() < 5.3){
          next_position = map_position[i];
          occ_map[i] = 1;
          break;
      }  
    }//找邻近点
    for(int i=0;i<16;i++){
      if(occ_map[i]==0){
          covered = false;
      }else{
          covered = true;
      }
    }//感觉有些多余，可删
    last_pos[id] = next_position;
    std::cout << "next_position: "<<next_position<<std::endl;
    return next_position;
}//执行search需要的



std::vector<Eigen::Vector3d> potential_target_tree;

void DecisionTree::update_dis_cur_pt(Eigen::Vector3d target_position){
    static bool discover = false;
    for(int i=0;i<potential_target_tree.size();i++){
        if(target_position == potential_target_tree[i] && discover == true){
            return;
        }
    }//在潜在目标树对该目标进行查找
    double distance = (target_position - current_position_).norm();
    if(distance < 5.0){
        potential_target_tree.push_back(target_position);//将目标点加入target_tree
        virtual_target_++;
        // incre_virtual_target_();
        discover = true;
        std::cout<< "discover: "<< getCounter() <<std::endl;
    }
}


void DecisionTree::decision_making_node(){
    // do something
    task_node.reset(new TASK_NODE(virtual_target_,true_target_));
    task_node->calculate_cost();//计算各个任务的cost，此处需要增添多层决策
    if(task_node->doing_last_task==true){
            task_node = nullptr;
            return;
    }//判断上一次任务是否完成
    for (const auto &task : task_node->fscore)
    {
        if(task_node->c_fscore > task.second){
            task_node->c_fscore = task.second;//更新任务节点中cost最小值
            task_node->task_name = task.first;//更新cost最小的任务
        }
    }
    // std::cout<< "task_name: "<<task_node->task_name<<std::endl;
   
    current_task = task_node->task_name_[task_node->task_name];

    if(task_node->task_name == 1){
        virtual_target_--;
        // des_virtual_target_();
    }//执行check任务，排查掉一个虚拟目标，这个放这合适吗？
    // delete task_node;
    task_node = nullptr;
}

void DecisionTree::print_task_name(){

}


void DecisionTree::update_status(Eigen::Vector3d position){
    current_position_ = position;
}

/*-----------------------------------*/


void TASK_NODE::calculate_cost(){
    for(int i=0;i<3;i++){
        
        task_precon.push_back(precon(i));
        if(precon(i)){
            if( i==0 ){
                // search cost
                task_cost_ = 1;//完成任务本身的所需要的cost
                action_cost_ = 5;//search搜索邻近位置点
                task_bonus_ = 2.5;
                missionallcost -=task_bonus_;
                fscore[SEARCH] = task_cost_ + action_cost_ + missionallcost;
                doing_last_task = false;
            }else if(i == 1){   
                // check cost
                task_cost_ = 2;
                action_cost_ = (current_position_-virtual_target_position_).norm();
                task_bonus_ = 10.0;
                missionallcost -=task_bonus_;
                fscore[CHECK] = task_cost_ + action_cost_ + missionallcost;
                doing_last_task = false;
            }else if(i == 2){
                // attack cost
                task_cost_ = 3;//这个该怎么设置？
                action_cost_ = (current_position_-target_position_).norm();
                task_bonus_ = 10.0;
                missionallcost -=task_bonus_;
                fscore[ATTACK] = task_cost_ + action_cost_ + missionallcost;
                doing_last_task = false;
            }
        }else if(precon(0)==false && precon(1)==false && precon(2)==false){
                // doing the last task
                doing_last_task = true;
        }
    }
    
}//cost部分，需要着重修改

void TASK_NODE::is_finish_last(){
    if(task_name == SEARCH){
            finish_last_ = true;
        else
            finish_last_ = false;
    }else if(task_name == CHECK){
        // finish_last_ = true;
        finish_last_ = false;
    }else if(task_name == ATTACK){
        // finish_last_ = true;
        finish_last_ = false;
    }   
}//判断前一次任务是否完成，这里需要修改,没有想好怎么改

bool TASK_NODE::precon(int task_name){
    
    bool precon = false;
    is_finish_last();
    switch (task_name)
    {
        case SEARCH:
            if(uncover_range_== true && finish_last_ == true){
                precon = true;
            }
            
            break;
        case CHECK:
            /* do something */
            if(virtual_target_> 0 && finish_last_ == true){
                precon = true;

            }
            
            break;
        case ATTACK:
            /* do something */
            if(true_target_>0 && finish_last_ == true){
                precon = true;
            }
            break;
        default:
            break;
    }
    return precon;
}



/*-----------------------------------*/



Decision::Decision(ros::NodeHandle &nh){
    // do something
    initDecision(nh);
}//构造函数

void Decision::decision_making(const ros::TimerEvent &e){
    std::cout << "decision_making" << std::endl;
    target_pub.publish(target_point);

    while(!decision_queue_1.empty()){
        DecisionTree::Ptr decision_tree = decision_queue_1.top();//对优先级最高的决策树节点pop
        /* do something*/
        decision_tree->update_dis_cur_pt(target_position_);//更新无人机当前位置到target的距离
        decision_queue_1.pop();//弹出已经处理过的节点
        decision_queue_2.push(decision_tree);
        decision_tree->decision_making_node();//对当前决策树节点进行任务决策分配
        if(decision_tree->current_task == "search"){
            traj_utils::IdOdom id_target;
            Eigen::Vector3d next_position = choosenextwaypoint(decision_tree->current_position_,decision_tree->getid());
            id_target.drone_id = decision_tree->getid();
            id_target.pose.x = next_position[0];
            id_target.pose.y = next_position[1];
            id_target.pose.z = next_position[2];
            id_target_pub.publish(id_target);
        }//需要添加其他两个任务的发布
    }
    while(!decision_queue_2.empty()){
        DecisionTree::Ptr decision_tree = decision_queue_2.top();
        decision_queue_2.pop();
        decision_queue_1.push(decision_tree);
    }
    rviz_display();
}//周期性回调的

void Decision::initDecision(ros::NodeHandle &nh){
    // do something
    nh_ = nh;
    // init priority queue
    nh.param("/decision_tree/manager/drone_num", drone_num, 1);
    std::cout<< "drone_num: "<<drone_num<<std::endl;
    for(int i=0;i<drone_num;i++){
        decision_queue_1.push(std::make_shared<DecisionTree>(i));
    }   
    /* 20hz */
    decision_timer_ = nh.createTimer(ros::Duration(0.1), &Decision::decision_making, this);//0.1调用一次决策
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    marker_pub2 = nh.advertise<visualization_msgs::Marker>("/visualization_marker2", 10);
    
    idodom_sub = nh.subscribe("/drone_planning/odom", 10, &Decision::IdOdomCallback, this, ros::TransportHints().tcpNoDelay());
    target_position_ = Eigen::Vector3d(10,-2.5,0.1);
    target_pub = nh.advertise<geometry_msgs::Point>("/target_position", 10);
    id_target_pub = nh.advertise<traj_utils::IdOdom>("/id_target", 10);
    target_point.x = target_position_[0];
    target_point.y = target_position_[1];
    target_point.z = target_position_[2];
    initSearchMap();

}

void Decision::IdOdomCallback(const traj_utils::IdOdomConstPtr &msg){
    update_status(msg->drone_id, Eigen::Vector3d(msg->pose.x, msg->pose.y, msg->pose.z));
};

void Decision::update_status(int id, Eigen::Vector3d position){
    while(!decision_queue_1.empty()){
        DecisionTree::Ptr decision_tree = decision_queue_1.top();
        if(decision_tree->getid() == id){
            decision_tree->update_status(position);
        }
        decision_queue_1.pop();
        decision_queue_2.push(decision_tree);
    }
    while(!decision_queue_2.empty()){
        DecisionTree::Ptr decision_tree = decision_queue_2.top();
        decision_queue_2.pop();
        decision_queue_1.push(decision_tree);
    }
}

void Decision::rviz_display(){
    for (int i=0;i<16;i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";     // 命名空间
        marker.id = i; // id
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        // marker.scale = {5.0,5.0,0.1};
        marker.scale.x = 5.0;
        marker.scale.y = 5.0;
        marker.scale.z = 1.0;
        if(occ_map[i] == 1){
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.3f;
        }else{
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.3f;
        
        }
        marker.pose.position.x = map_position[i][0];
        marker.pose.position.y = map_position[i][1];
        marker.pose.position.z = map_position[i][2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1;
        marker.lifetime = ros::Duration();
        std::cout << "map_position: "<<map_position[i]<<std::endl;
        marker_pub2.publish(marker);
    }
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world"; 
    marker.header.stamp = ros::Time::now();
    marker.ns = "point";     // 命名空间
    marker.id = 0; // id
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.3;  // 点的宽度
    marker.scale.y = 0.3;  // 点的高度
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;  
    geometry_msgs::Point p1;
    p1.x = target_position_[0];
    p1.y = target_position_[1];
    p1.z = target_position_[2];
    marker.points.push_back(p1);
    marker_pub.publish(marker);
    for(int i=0;i<drone_num;i++){
        DecisionTree::Ptr decision_tree = decision_queue_1.top();
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";     // 命名空间
        marker.id = decision_tree->getid(); // id
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = decision_tree->current_position_[0]; 
        marker.pose.position.y = decision_tree->current_position_[1];
        marker.pose.position.z = decision_tree->current_position_[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.5;  // 文本的高度
        if(marker.id == 0){
            marker.color.r = 1.0; 
        }else if(marker.id == 1){
            marker.color.g = 1.0;
        }else if(marker.id == 2){
            marker.color.b = 1.0;
        }
        marker.color.a = 0.8;
        marker.text =  std::to_string(marker.id)+":"+ decision_tree->get_task(); 
        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);
        decision_queue_1.pop();
        decision_queue_2.push(decision_tree);
    }
    while(!decision_queue_2.empty()){
        DecisionTree::Ptr decision_tree = decision_queue_2.top();
        decision_queue_2.pop();
        decision_queue_1.push(decision_tree);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "decision_tree");
    ros::NodeHandle nh;
    Decision decision(nh);
    decision.initDecision(nh);
    ros::spin();
    return 0;
}