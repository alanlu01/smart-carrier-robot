#include <mutex>
#include <cmath>
#include "smart_station_controller/alignment_envelope.hpp"
#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace smart_station_controller {
class StationAlignment : public nav2_rotation_shim_controller::RotationShimController {
public:
  void setPlan(const nav_msgs::msg::Path & path) override {
    if (path.poses.empty()) { throw nav2_core::InvalidPath("Empty alignment path"); }
    RotationShimController::setPlan(path);
    std::lock_guard<std::mutex> lock(alignment_mutex_);
    goal_ = path.poses.back();
    envelope_.reset();
  }
  void reset() override {
    RotationShimController::reset();
    std::lock_guard<std::mutex> lock(alignment_mutex_);
    envelope_.reset();
  }
  void setSpeedLimit(const double & limit, const bool & percentage) override {
    RotationShimController::setSpeedLimit(limit, percentage);
    std::lock_guard<std::mutex> lock(alignment_mutex_);
    // ControllerServer's collision/slow-zone limits apply to pure rotation too.
    speed_scale_ = limit == nav2_costmap_2d::NO_SPEED_LIMIT ? 1. :
      std::clamp(percentage ? limit/100. : limit/.20, 0., 1.);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker *) override {
    std::lock_guard<std::mutex> lock(alignment_mutex_);
    if (goal_.header.frame_id.empty()) { throw nav2_core::InvalidPath("No alignment goal"); }
    auto goal = goal_;
    try {
      if (goal.header.frame_id != pose.header.frame_id) {
        goal.header.stamp = pose.header.stamp;
        goal = tf_->transform(goal, pose.header.frame_id);
      }
    } catch (const tf2::TransformException & ex) { throw nav2_core::ControllerTFError(ex.what()); }
    const auto yaw = [](const geometry_msgs::msg::Quaternion & q) {
      return std::atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z));
    };
    double error = yaw(goal.pose.orientation)-yaw(pose.pose.orientation);
    error = std::atan2(std::sin(error), std::cos(error));
    if (!std::isfinite(error) || !std::isfinite(velocity.angular.z)) {
      throw nav2_core::NoValidControl("Nonfinite alignment feedback");
    }
    geometry_msgs::msg::TwistStamped result;
    result.header = pose.header;
    result.twist.angular.z = envelope_.command(error, velocity.angular.z,
      clock_->now().seconds(), .20*speed_scale_);
    // Keep native costmap footprint simulation; zero translation is deliberate.
    if (result.twist.angular.z != 0.) { isCollisionFree(result, error, pose); }
    return result;
  }
private:
  std::mutex alignment_mutex_;
  geometry_msgs::msg::PoseStamped goal_;
  AlignmentEnvelope envelope_;
  double speed_scale_ = 1.;
};
}  // namespace smart_station_controller
PLUGINLIB_EXPORT_CLASS(smart_station_controller::StationAlignment, nav2_core::Controller)
