#include <memory>
#include <mutex>
#include <string>
#include <nav2_core/global_planner.hpp>
#include <nav2_core/planner_exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nlohmann/json.hpp>
#include "smart_station_controller/guarded_navfn.hpp"
#include "smart_station_controller/motion_stall.hpp"

namespace smart_station_controller
{
class GuardedPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    node_=parent;costmap_=costmap;
    auto node=node_.lock();
    publisher_=node->create_publisher<std_msgs::msg::String>(
      "/delivery/route_state",rclcpp::QoS(1).best_effort());
    odom_=node->create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        const auto & p=m->pose.pose;
        auto current=node_.lock();
        if (!current) {motion_.reset();return;}
        const double age=(current->now()-rclcpp::Time(m->header.stamp)).seconds();
        if (!std::isfinite(age) || age<-.1 || age>.3) {motion_.reset();return;}
        motion_.odom(p.position.x,p.position.y,tf2::getYaw(p.orientation),steady());
      });
    command_=node->create_subscription<geometry_msgs::msg::Twist>("/chassis_cmd_vel",rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        motion_.command(m->linear.x,m->linear.y,m->angular.z,steady());
      });
  }
  void cleanup() override
  {
    odom_.reset();command_.reset();
    std::lock_guard<std::mutex> lock(motion_mutex_);
    motion_.reset();goal_stamp_=0;engine_.reset();publisher_.reset();costmap_.reset();
  }
  void activate() override {publisher_->on_activate();}
  void deactivate() override
  {
    publisher_->on_deactivate();engine_.reset();
    std::lock_guard<std::mutex> lock(motion_mutex_);motion_.reset();goal_stamp_=0;
  }

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,std::function<bool()> cancel) override
  {
    auto node=node_.lock();
    if (!node || !costmap_ || start.header.frame_id!=costmap_->getGlobalFrameID() ||
      goal.header.frame_id!=costmap_->getGlobalFrameID()) {
      throw nav2_core::PlannerTFError("Guarded route needs finite poses in the costmap frame");
    }
    const RoutePose a{start.pose.position.x,start.pose.position.y,tf2::getYaw(start.pose.orientation)},
      b{goal.pose.position.x,goal.pose.position.y,tf2::getYaw(goal.pose.orientation)};
    for (double v:{a.x,a.y,a.yaw,b.x,b.y,b.yaw}) {
      if (!std::isfinite(v)) {throw nav2_core::PlannerTFError("Nonfinite route pose");}
    }
    RouteGrid grid;
    {
      auto map=costmap_->getCostmap();
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      grid.width=map->getSizeInCellsX();grid.height=map->getSizeInCellsY();
      grid.resolution=map->getResolution();grid.x=map->getOriginX();grid.y=map->getOriginY();
      grid.costs.assign(map->getCharMap(),map->getCharMap()+grid.width*grid.height);
    } // Never hold the live costmap mutex during search/geometry/publication.
    const double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    std::vector<RoutePose> route;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      const auto stamp=rclcpp::Time(goal.header.stamp).nanoseconds();
      if (stamp!=goal_stamp_) {motion_.reset();goal_stamp_=stamp;}
      if (!cancel() && motion_.stalled(now)) {
        engine_.motionStalled(a,now);motion_.reset();++tiny_stalls_;
        RCLCPP_WARN(node->get_logger(),"Tiny command without wheel progress; replanning the corridor");
      }
    }
    try {route=engine_.plan(grid,a,b,now,cancel);}
    catch (const std::runtime_error & e) {
      publish(now,goal,false,0.);
      if (cancel()) {throw nav2_core::PlannerCancelled("Guarded route cancelled");}
      throw nav2_core::NoValidPathCouldBeFound(e.what());
    }
    if (cancel()) {throw nav2_core::PlannerCancelled("Guarded route cancelled");}
    nav_msgs::msg::Path result;result.header.frame_id=goal.header.frame_id;result.header.stamp=node->now();
    for (size_t i=0;i<route.size();++i) {
      geometry_msgs::msg::PoseStamped p;p.header=result.header;
      p.pose.position.x=route[i].x;p.pose.position.y=route[i].y;
      const double yaw=route[i].yaw;
      p.pose.orientation.z=std::sin(yaw/2.);p.pose.orientation.w=std::cos(yaw/2.);
      result.poses.push_back(p);
    }
    result.poses.front()=start;result.poses.back()=goal;
    publish(now,goal,true,GuardedNavfn::length(route));
    return result;
  }
private:
  void publish(double now,const geometry_msgs::msg::PoseStamped & goal,bool valid,double remaining)
  {
    std_msgs::msg::String msg;
    msg.data=nlohmann::json{{"planner","guarded_navfn_dijkstra"},{"valid",valid},
      {"retained",engine_.retained},{"generation",engine_.generation},
      {"repair_attempts",engine_.repairs},{"avoid_regions",engine_.avoid.size()},
      {"heading_policy",engine_.heading_policy},
      {"stalled_routes",engine_.stalls},{"remaining_m",remaining},
      {"tiny_stall_events",tiny_stalls_},{"sample_ros_stamp_ns",node_.lock()->now().nanoseconds()},
      {"no_progress_sec",std::max(0.,now-engine_.last_progress)},
      {"goal_stamp_ns",rclcpp::Time(goal.header.stamp).nanoseconds()}}.dump();
    publisher_->publish(msg);
  }
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  GuardedNavfn engine_;
  static double steady() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  std::mutex motion_mutex_;
  MotionStall motion_;
  int64_t goal_stamp_=0;
  unsigned tiny_stalls_=0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_;
};
}  // namespace smart_station_controller
PLUGINLIB_EXPORT_CLASS(smart_station_controller::GuardedPlanner,nav2_core::GlobalPlanner)
