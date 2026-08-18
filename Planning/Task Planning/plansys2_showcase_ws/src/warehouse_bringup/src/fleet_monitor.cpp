// What the plan cost the fleet, as opposed to what the planner thought it would.
//
// A makespan is a prediction. This node records the outcome: how far each
// vehicle actually drove, how long it actually spent moving, and how much of
// that time it spent stationary while it was supposed to be executing a `move`.
// Those three numbers are what separate "the fleet finished sooner" from "the
// fleet drove further to finish sooner", which is the question the multi-agent
// half of the showcase exists to answer.
//
// It also draws the roadmap in RViz, because a waypoint graph that only exists
// in a YAML file is a graph nobody checks.

#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

namespace warehouse_bringup
{

struct RobotTrack
{
  bool have_previous{false};
  double previous_x{0.0};
  double previous_y{0.0};
  double distance_m{0.0};
  double moving_seconds{0.0};
  double idle_seconds{0.0};
  rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
};

class FleetMonitor : public rclcpp::Node
{
public:
  FleetMonitor()
  : rclcpp::Node("fleet_monitor")
  {
    declare_parameter<std::vector<std::string>>("robots", {"r1"});
    declare_parameter<std::string>("roadmap", "");
    declare_parameter<std::string>("log_dir", "");
    declare_parameter<double>("sample_period", 1.0);
    // Below this speed the vehicle is counted as stationary. Odometry noise on
    // a stopped differential drive is well under a centimetre per second, so
    // 0.02 m/s separates "waiting" from "creeping" without argument.
    declare_parameter<double>("moving_threshold", 0.02);

    robots_ = get_parameter("robots").as_string_array();
    log_dir_ = get_parameter("log_dir").as_string();
    moving_threshold_ = get_parameter("moving_threshold").as_double();

    for (const auto & robot : robots_) {
      tracks_[robot] = RobotTrack{};
      subscriptions_.push_back(
        create_subscription<nav_msgs::msg::Odometry>(
          "/" + robot + "/odom", 10,
          [this, robot](nav_msgs::msg::Odometry::SharedPtr msg) {onOdom(robot, msg);}));
    }

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/warehouse/roadmap", rclcpp::QoS(1).transient_local());
    publishRoadmap(get_parameter("roadmap").as_string());

    if (!log_dir_.empty()) {
      csv_.open(log_dir_ + "/fleet.csv", std::ios::out | std::ios::trunc);
      csv_ << "wall_time_s";
      for (const auto & robot : robots_) {
        csv_ << ',' << robot << "_distance_m," << robot << "_moving_s," << robot << "_idle_s";
      }
      csv_ << '\n';
    }

    started_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(get_parameter("sample_period").as_double()),
      std::bind(&FleetMonitor::sample, this));
    RCLCPP_INFO(get_logger(), "monitoring %zu robot(s)", robots_.size());
  }

  ~FleetMonitor() override
  {
    summarise();
  }

private:
  void onOdom(const std::string & robot, const nav_msgs::msg::Odometry::SharedPtr & msg)
  {
    auto & track = tracks_[robot];
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const rclcpp::Time stamp(msg->header.stamp);

    if (track.have_previous) {
      const double step = std::hypot(x - track.previous_x, y - track.previous_y);
      const double dt = (stamp - track.last_stamp).seconds();
      if (dt > 0.0 && dt < 1.0) {
        track.distance_m += step;
        // Integrating the classification rather than the speed keeps a vehicle
        // that is turning in place — moving, but covering no ground — on the
        // moving side of the ledger, where it belongs.
        if (step / dt >= moving_threshold_ ||
          std::fabs(msg->twist.twist.angular.z) >= 0.05)
        {
          track.moving_seconds += dt;
        } else {
          track.idle_seconds += dt;
        }
      }
    }
    track.previous_x = x;
    track.previous_y = y;
    track.last_stamp = stamp;
    track.have_previous = true;
  }

  void sample()
  {
    if (!csv_.is_open()) {
      return;
    }
    csv_ << (now() - started_).seconds();
    for (const auto & robot : robots_) {
      const auto & track = tracks_[robot];
      csv_ << ',' << track.distance_m << ',' << track.moving_seconds << ','
           << track.idle_seconds;
    }
    csv_ << '\n';
    csv_.flush();
  }

  void publishRoadmap(const std::string & path)
  {
    if (path.empty()) {
      return;
    }
    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "no roadmap to draw: %s", error.what());
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    std::map<std::string, std::pair<double, double>> positions;

    for (const auto & entry : root["waypoints"]) {
      const auto name = entry.first.as<std::string>();
      const double x = entry.second["x"].as<double>();
      const double y = entry.second["y"].as<double>();
      const auto kind = entry.second["kind"].as<std::string>();
      positions[name] = {x, y};

      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = "map";
      sphere.header.stamp = now();
      sphere.ns = "waypoints";
      sphere.id = id++;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = x;
      sphere.pose.position.y = y;
      sphere.pose.position.z = 0.05;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = (kind == "corridor") ? 0.18 : 0.30;
      sphere.color.a = 0.9f;
      if (kind == "dock") {
        sphere.color.r = 0.80f; sphere.color.g = 0.20f; sphere.color.b = 0.15f;
      } else if (kind == "storage") {
        sphere.color.r = 0.10f; sphere.color.g = 0.55f; sphere.color.b = 0.25f;
      } else if (kind == "charger") {
        sphere.color.r = 0.85f; sphere.color.g = 0.65f; sphere.color.b = 0.10f;
      } else {
        sphere.color.r = 0.20f; sphere.color.g = 0.45f; sphere.color.b = 0.75f;
      }
      markers.markers.push_back(sphere);

      if (kind != "corridor") {
        visualization_msgs::msg::Marker label = sphere;
        label.ns = "waypoint_labels";
        label.id = id++;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.pose.position.z = 0.55;
        label.scale.z = 0.35;
        label.text = name;
        label.color.r = label.color.g = label.color.b = 0.05f;
        label.color.a = 1.0f;
        markers.markers.push_back(label);
      }
    }

    visualization_msgs::msg::Marker lines;
    lines.header.frame_id = "map";
    lines.header.stamp = now();
    lines.ns = "roadmap_edges";
    lines.id = id++;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.scale.x = 0.03;
    lines.color.r = 0.20f;
    lines.color.g = 0.45f;
    lines.color.b = 0.75f;
    lines.color.a = 0.5f;
    lines.pose.orientation.w = 1.0;
    for (const auto & edge : root["edges"]) {
      const auto from = edge["from"].as<std::string>();
      const auto to = edge["to"].as<std::string>();
      if (!positions.count(from) || !positions.count(to)) {
        continue;
      }
      geometry_msgs::msg::Point a, b;
      a.x = positions[from].first;
      a.y = positions[from].second;
      a.z = 0.02;
      b.x = positions[to].first;
      b.y = positions[to].second;
      b.z = 0.02;
      lines.points.push_back(a);
      lines.points.push_back(b);
    }
    markers.markers.push_back(lines);

    marker_pub_->publish(markers);
    RCLCPP_INFO(get_logger(), "drew %zu roadmap markers", markers.markers.size());
  }

  void summarise()
  {
    if (log_dir_.empty()) {
      return;
    }
    std::ofstream out(log_dir_ + "/fleet_summary.csv", std::ios::out | std::ios::trunc);
    out << "robot,distance_m,moving_s,idle_s\n";
    for (const auto & robot : robots_) {
      const auto & track = tracks_[robot];
      out << robot << ',' << track.distance_m << ',' << track.moving_seconds << ','
          << track.idle_seconds << '\n';
    }
  }

  std::vector<std::string> robots_;
  std::string log_dir_;
  double moving_threshold_{0.02};
  std::map<std::string, RobotTrack> tracks_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subscriptions_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time started_;
  std::ofstream csv_;
};

}  // namespace warehouse_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<warehouse_bringup::FleetMonitor>());
  rclcpp::shutdown();
  return 0;
}
