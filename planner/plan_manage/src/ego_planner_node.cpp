#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/decision_tree.h>

using namespace ego_planner;
// std::mutex global_mutex;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego-planner");
  ros::NodeHandle nh;

  std::shared_ptr<GridMap> grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  Decision decision(nh, grid_map);
  decision.start_decision_making();

  ros::MultiThreadedSpinner spinner(4); // 使用多线程处理回调
  ros::spin();

  return 0;
}