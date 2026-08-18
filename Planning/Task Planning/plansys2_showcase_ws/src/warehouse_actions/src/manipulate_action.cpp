// The `pick` and `drop` performers.
//
// Both are the same node with a different `action_name`: this fleet has no
// manipulator, so the physical content of grasping a crate is a dwell of the
// duration the domain promised, and the honest thing is to implement it once
// and say so. What the node does add is the part that is *not* a stand-in —
// it publishes an RViz marker for the crate, so that the symbolic state
// (`holding`, `crate_at`) has a visible consequence in the world, and a viewer
// can see the plan's effects rather than trusting the log.
//
// The dwell is read from the action's own duration in the domain, passed in as
// a parameter, so that changing the domain changes the execution rather than
// silently disagreeing with it.

#include <chrono>
#include <memory>
#include <string>

#include <plansys2_executor/ActionExecutorClient.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>

namespace warehouse_actions
{

class ManipulateAction : public plansys2::ActionExecutorClient
{
public:
  ManipulateAction()
  : plansys2::ActionExecutorClient("manipulate_action", std::chrono::milliseconds(100))
  {
    declare_parameter<std::string>("robot_name", "r1");
    declare_parameter<std::string>("roadmap", "");
    declare_parameter<double>("dwell_seconds", 8.0);
    declare_parameter<bool>("carrying_after", true);   // true for pick, false for drop
  }

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override
  {
    robot_name_ = get_parameter("robot_name").as_string();
    dwell_ = get_parameter("dwell_seconds").as_double();
    carrying_after_ = get_parameter("carrying_after").as_bool();
    loadRoadmap(get_parameter("roadmap").as_string());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/warehouse/crates", 10);
    return plansys2::ActionExecutorClient::on_configure(state);
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state)
  {
    marker_pub_->on_activate();
    started_ = now();
    const auto & args = get_arguments();          // r, crate, waypoint
    crate_ = args.size() > 1 ? args[1] : "crate";
    place_ = args.size() > 2 ? args[2] : "";
    RCLCPP_INFO(
      get_logger(), "%s: %s %s at %s", robot_name_.c_str(),
      carrying_after_ ? "pick" : "drop", crate_.c_str(), place_.c_str());
    return plansys2::ActionExecutorClient::on_activate(state);
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state)
  {
    marker_pub_->on_deactivate();
    return plansys2::ActionExecutorClient::on_deactivate(state);
  }

private:
  void loadRoadmap(const std::string & path)
  {
    if (path.empty()) {
      return;
    }
    const auto root = YAML::LoadFile(path);
    for (const auto & entry : root["waypoints"]) {
      places_[entry.first.as<std::string>()] = {
        entry.second["x"].as<double>(), entry.second["y"].as<double>()};
    }
  }

  void do_work() override
  {
    const double elapsed = (now() - started_).seconds();
    if (elapsed >= dwell_) {
      publishMarker();
      finish(true, 1.0, carrying_after_ ? "picked" : "dropped");
      return;
    }
    send_feedback(elapsed / dwell_, carrying_after_ ? "picking" : "dropping");
  }

  // A crate that has been picked up is removed from the floor; a crate that has
  // been dropped is drawn at the dock. Nothing here reads the knowledge base —
  // the marker follows this node's own action, so a marker in the wrong place
  // is evidence that execution and the plan have diverged.
  void publishMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "crates";
    marker.id = static_cast<int>(std::hash<std::string>{}(crate_) % 10000);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = carrying_after_ ? visualization_msgs::msg::Marker::DELETE
      : visualization_msgs::msg::Marker::ADD;
    if (places_.count(place_)) {
      marker.pose.position.x = places_[place_].first;
      marker.pose.position.y = places_[place_].second;
    }
    marker.pose.position.z = 0.15;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
    marker.color.r = 0.85f;
    marker.color.g = 0.55f;
    marker.color.b = 0.10f;
    marker.color.a = 1.0f;
    marker_pub_->publish(marker);
  }

  std::string robot_name_;
  std::string crate_;
  std::string place_;
  double dwell_{8.0};
  bool carrying_after_{true};
  rclcpp::Time started_;
  std::map<std::string, std::pair<double, double>> places_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace warehouse_actions

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<warehouse_actions::ManipulateAction>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(200),
    [node]() {
      static bool configured = false;
      if (!configured) {
        configured = true;
        node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      }
    });
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
