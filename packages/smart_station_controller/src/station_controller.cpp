#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "smart_station_controller/speed_envelope.hpp"

namespace smart_station_controller
{
class StationMPPI : public nav2_mppi_controller::MPPIController
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    auto node = parent.lock();
    if (!node) {throw std::runtime_error("station controller parent unavailable");}
    const auto parameter = [&](const std::string & suffix, double fallback) {
        const auto key = name+".station."+suffix;
        if (!node->has_parameter(key)) {node->declare_parameter(key, fallback);}
        return node->get_parameter(key).as_double();
      };
    envelope_.maximum = parameter("maximum_speed", .15);
    envelope_.slow = parameter("slow_speed", .06);
    envelope_.inner = parameter("slow_distance", .20);
    envelope_.outer = parameter("ramp_distance", .40);
    envelope_.validate();
    diagnostics_ = node->create_publisher<std_msgs::msg::String>("/delivery/station_controller_state", 10);
    nav2_mppi_controller::MPPIController::configure(parent, name, tf, costmap);
  }
  void activate() override
  {
    nav2_mppi_controller::MPPIController::activate();diagnostics_->on_activate();
  }
  void deactivate() override
  {
    diagnostics_->on_deactivate();nav2_mppi_controller::MPPIController::deactivate();
  }
  void cleanup() override
  {
    diagnostics_.reset();nav2_mppi_controller::MPPIController::cleanup();
  }
  void reset() override
  {
    {std::lock_guard<std::mutex> lock(mutex_);has_goal_ = false;}
    nav2_mppi_controller::MPPIController::reset();
  }
  void setPlan(const nav_msgs::msg::Path & path) override
  {
    {std::lock_guard<std::mutex> lock(mutex_);
      has_goal_ = !path.poses.empty();
      if (has_goal_) {
        goal_ = path.poses.back();
        if (goal_.header.frame_id.empty()) {goal_.header.frame_id = path.header.frame_id;}
      }
    }
    nav2_mppi_controller::MPPIController::setPlan(path);
  }
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    // Cache genuine external limits, but never publish our distance envelope
    // to the shared speed_limit topic or alter ordinary FollowPath settings.
    std::lock_guard<std::mutex> lock(mutex_);
    external_ = speed_limit;percentage_ = percentage;
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * checker) override
  {
    geometry_msgs::msg::PoseStamped goal;
    double external;bool percentage;
    {std::lock_guard<std::mutex> lock(mutex_);
      if (!has_goal_) {throw nav2_core::InvalidPath("station controller has no goal");}
      goal = goal_;external = external_;percentage = percentage_;
    }
    goal.header.stamp = pose.header.stamp;
    geometry_msgs::msg::PoseStamped local_goal;
    try {
      if (goal.header.frame_id == pose.header.frame_id) {local_goal = goal;}
      else {tf_buffer_->transform(goal, local_goal, pose.header.frame_id, tf2::durationFromSec(.1));}
    } catch (const tf2::TransformException & error) {
      throw nav2_core::ControllerTFError(std::string("station goal transform: ")+error.what());
    }
    const double distance = std::hypot(local_goal.pose.position.x-pose.pose.position.x,
      local_goal.pose.position.y-pose.pose.position.y);
    const double cap = envelope_.limit(distance, external, percentage);
    geometry_msgs::msg::TwistStamped command;
    if (cap > 0.) {
      nav2_mppi_controller::MPPIController::setSpeedLimit(100.*cap/envelope_.maximum, true);
      command = nav2_mppi_controller::MPPIController::computeVelocityCommands(pose, speed, checker);
      // MPPI's command history filter can overshoot individual sampled bounds.
      // Keep the published resultant translation inside the local envelope.
      const double norm = std::hypot(command.twist.linear.x, command.twist.linear.y);
      if (norm > cap) {command.twist.linear.x *= cap/norm;command.twist.linear.y *= cap/norm;}
    } else {command.header = pose.header;}
    auto node = parent_.lock();
    if (node && (node->now().seconds()-last_report_ >= 1. || last_report_ == 0.)) {
      last_report_ = node->now().seconds();
      std::ostringstream text;
      text << "{\"controller\":\"" << name_ << "\",\"remaining_m\":" << distance
           << ",\"speed_cap_mps\":" << cap << ",\"command_speed_mps\":"
           << std::hypot(command.twist.linear.x, command.twist.linear.y) << "}";
      std_msgs::msg::String message;message.data = text.str();diagnostics_->publish(message);
    }
    return command;
  }
private:
  SpeedEnvelope envelope_;
  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped goal_;
  bool has_goal_{false}, percentage_{false};
  double external_{0.}, last_report_{0.};
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr diagnostics_;
};
}  // namespace smart_station_controller
PLUGINLIB_EXPORT_CLASS(smart_station_controller::StationMPPI, nav2_core::Controller)
