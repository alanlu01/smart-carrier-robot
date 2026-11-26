// One stable tree; no profile is usable until it matches this exact map goal.
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace smart_station_controller
{
class NavigationPolicy : public BT::StatefulActionNode
{
public:
  NavigationPolicy(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "/delivery/navigation_policy", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        try {policy_ = nlohmann::json::parse(message->data);} catch (...) {policy_ = nullptr;}
      }, options);
    diagnostics_=node_->create_publisher<std_msgs::msg::String>(
      "/delivery/navigation_policy_state",rclcpp::QoS(1).reliable().transient_local());
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::OutputPort<std::string>("controller"), BT::OutputPort<std::string>("goal_checker"),
      BT::OutputPort<bool>("station"), BT::OutputPort<uint16_t>("error_code_id")};
  }

  BT::NodeStatus onStart() override
  {
    started_ = std::chrono::steady_clock::now();
    last_report_=started_-std::chrono::seconds(1);
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    executor_.spin_some();
    geometry_msgs::msg::PoseStamped goal;
    static const std::unordered_map<std::string, std::string> profiles{
      {"FollowPath", "general_goal_checker"}, {"StationX", "station_goal_checker"},
      {"StationY", "station_goal_checker"}, {"StationExitX", "general_goal_checker"},
      {"StationExitY", "general_goal_checker"}, {"StationAlign", "station_alignment_checker"}};
    try {
      if (getInput("goal", goal) && policy_.is_object()) {
        const auto stamp = rclcpp::Time(goal.header.stamp).nanoseconds();
        const auto controller = policy_.at("controller").get<std::string>();
        const auto checker = policy_.at("goal_checker").get<std::string>();
        const auto station = policy_.at("station").get<bool>();
        const auto & p = goal.pose.position;
        const auto & q = goal.pose.orientation;
        const double values[]{p.x,p.y,p.z,q.x,q.y,q.z,q.w};
        const auto position = policy_.at("position").get<std::vector<double>>();
        const auto orientation = policy_.at("orientation").get<std::vector<double>>();
        bool matches = position.size()==3 && orientation.size()==4;
        if (matches) {
          for (unsigned i=0; i<7; ++i) {
            const auto value = i<3 ? position[i] : orientation[i-3];
            matches = matches && std::isfinite(value) && std::isfinite(values[i]) &&
              std::abs(values[i]-value)<1e-9;
          }
        }
        if (matches && goal.header.frame_id=="map" && policy_.at("frame")=="map" &&
          policy_.at("schema_version")==1 && stamp>0 && policy_.at("stamp_ns")==stamp &&
          profiles.count(controller) && profiles.at(controller)==checker &&
          station==(controller!="FollowPath"))
        {
          setOutput("controller", controller);
          setOutput("goal_checker", checker);
          setOutput("station", station);
          setOutput("error_code_id", uint16_t(0));
          report(goal,"matched");
          config().blackboard->set("compute_path_error_code", uint16_t(0));
          config().blackboard->set("follow_path_error_code", uint16_t(0));
          return BT::NodeStatus::SUCCESS;
        }
      }
    } catch (const nlohmann::json::exception &) {}
    if (std::chrono::steady_clock::now()-started_>std::chrono::seconds(2)) {
      setOutput("error_code_id", uint16_t(10001));
      report(goal,"mismatch");
      RCLCPP_ERROR(node_->get_logger(), "Delivery policy did not match the stamped goal; no motion");
      return BT::NodeStatus::FAILURE;
    }
    if (std::chrono::steady_clock::now()-last_report_>std::chrono::milliseconds(500)) {
      report(goal,"waiting");
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  void report(const geometry_msgs::msg::PoseStamped & goal,const char * status)
  {
    last_report_=std::chrono::steady_clock::now();
    const auto & p=goal.pose.position;const auto & q=goal.pose.orientation;
    std_msgs::msg::String message;
    message.data=nlohmann::json{{"status",status},{"goal_stamp_ns",rclcpp::Time(goal.header.stamp).nanoseconds()},
      {"frame",goal.header.frame_id},{"position",{p.x,p.y,p.z}},
      {"orientation",{q.x,q.y,q.z,q.w}},{"received_policy",policy_},
      {"wait_sec",std::chrono::duration<double>(last_report_-started_).count()}}.dump();
    diagnostics_->publish(message);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  nlohmann::json policy_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_;
  std::chrono::steady_clock::time_point last_report_;
  std::chrono::steady_clock::time_point started_;
};
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<smart_station_controller::NavigationPolicy>("DeliveryNavigationPolicy");
}
