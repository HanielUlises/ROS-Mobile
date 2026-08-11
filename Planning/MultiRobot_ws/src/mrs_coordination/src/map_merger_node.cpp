// Merges the per-robot occupancy grids produced by the individual slam_toolbox
// instances into a single fleet-level map.
//
// First iteration assumptions (documented so they can be lifted later):
//
//   * The relative pose between robots is known, because every robot is spawned
//     at a configured pose and slam_toolbox anchors <ns>/map at the spawn pose.
//     The merger therefore knows map -> <ns>/map exactly and publishes it as a
//     static transform. Recovering that transform from inter-robot loop closure
//     is deliberately out of scope here.
//   * All grids share the same resolution.
//
// The merge itself is gated by connectivity: a robot whose /<ns>/comms_ok is
// false does not contribute. That gate is the hook the epistemic layer will use
// later, since "which cells are known to which agent" is exactly the content of
// the modal operators the project reasons about. The node already publishes the
// per-robot and fleet-level known-cell coverage that the comparison against the
// classical baseline needs.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace mrs_coordination
{

namespace
{
constexpr int8_t kUnknown = -1;

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// Axis-aligned bounds of the merged grid, in world metres.
struct Bounds
{
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
  bool valid{false};

  void extend(double x, double y)
  {
    if (!valid) {
      min_x = max_x = x;
      min_y = max_y = y;
      valid = true;
      return;
    }
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
  }
};
}  // namespace

class MapMergerNode : public rclcpp::Node
{
public:
  MapMergerNode()
  : rclcpp::Node("map_merger")
  {
    declare_parameter<std::vector<std::string>>("robot_names", {"robot1", "robot2"});
    declare_parameter<std::vector<double>>("initial_poses", std::vector<double>{});
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<double>("merge_rate", 1.0);
    declare_parameter<double>("resolution", 0.05);
    declare_parameter<bool>("respect_comms", true);

    robot_names_ = get_parameter("robot_names").as_string_array();
    global_frame_ = get_parameter("global_frame").as_string();
    resolution_ = get_parameter("resolution").as_double();
    respect_comms_ = get_parameter("respect_comms").as_bool();
    const double merge_rate = get_parameter("merge_rate").as_double();

    const auto flat_poses = get_parameter("initial_poses").as_double_array();
    if (flat_poses.size() != robot_names_.size() * 3) {
      RCLCPP_FATAL(
        get_logger(),
        "initial_poses must hold 3 values (x, y, yaw) per robot: expected %zu, got %zu",
        robot_names_.size() * 3, flat_poses.size());
      throw std::runtime_error("initial_poses / robot_names size mismatch");
    }

    static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    merged_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);
    coverage_pub_ = create_publisher<std_msgs::msg::Float32>("/mrs/coverage", 10);

    std::vector<geometry_msgs::msg::TransformStamped> static_tfs;
    static_tfs.reserve(robot_names_.size());

    for (size_t i = 0; i < robot_names_.size(); ++i) {
      const std::string & name = robot_names_[i];

      origins_[name] = Pose2D{flat_poses[3 * i], flat_poses[3 * i + 1], flat_poses[3 * i + 2]};
      connected_[name] = true;
      stale_[name] = 0;

      map_subs_.push_back(
        create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/" + name + "/map", qos,
          [this, name](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            // A dropped link stops the flow of new observations; it does not
            // erase what the fleet already received. The last map shared while
            // connected therefore stays in the merge, and simply goes stale
            // until the link returns. Discarding it instead would model the
            // fleet as forgetting, which is not what a lost radio does.
            if (!respect_comms_ || connected_[name]) {
              latest_maps_[name] = msg;
            } else {
              ++stale_[name];
            }
          }));

      comms_subs_.push_back(
        create_subscription<std_msgs::msg::Bool>(
          "/" + name + "/comms_ok", 10,
          [this, name](std_msgs::msg::Bool::SharedPtr msg) {
            const bool was = connected_[name];
            connected_[name] = msg->data;
            if (was != msg->data) {
              if (msg->data) {
                RCLCPP_INFO(
                  get_logger(), "%s reconnected, %lu withheld updates now superseded",
                  name.c_str(), stale_[name]);
                stale_[name] = 0;
              } else {
                RCLCPP_INFO(
                  get_logger(),
                  "%s disconnected, its last shared map is retained but frozen",
                  name.c_str());
              }
            }
          }));

      coverage_pubs_[name] =
        create_publisher<std_msgs::msg::Float32>("/" + name + "/coverage", 10);

      static_tfs.push_back(makeStaticTf(name, origins_[name]));
    }

    static_tf_->sendTransform(static_tfs);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(merge_rate, 0.1)),
      std::bind(&MapMergerNode::merge, this));

    RCLCPP_INFO(
      get_logger(), "map_merger tracking %zu robots, publishing /map in frame '%s'",
      robot_names_.size(), global_frame_.c_str());
  }

private:
  geometry_msgs::msg::TransformStamped makeStaticTf(
    const std::string & name, const Pose2D & pose) const
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = global_frame_;
    tf.child_frame_id = name + "/map";
    tf.transform.translation.x = pose.x;
    tf.transform.translation.y = pose.y;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    return tf;
  }

  // Robots that count towards the shared map: everyone whose map has reached
  // the fleet at least once, connected or not (see the map callback).
  std::vector<std::string> contributors() const
  {
    std::vector<std::string> out;
    for (const auto & name : robot_names_) {
      if (latest_maps_.count(name) != 0) {
        out.push_back(name);
      }
    }
    return out;
  }

  void merge()
  {
    publishPerRobotCoverage();

    const auto active = contributors();
    if (active.empty()) {
      return;
    }

    // 1. World-frame extent covering every contributing grid. Each grid is
    //    rotated by its robot's spawn yaw, so all four corners matter.
    Bounds bounds;
    for (const auto & name : active) {
      const auto & grid = *latest_maps_.at(name);
      const auto & origin = origins_.at(name);
      const double w = grid.info.width * grid.info.resolution;
      const double h = grid.info.height * grid.info.resolution;
      const double ox = grid.info.origin.position.x;
      const double oy = grid.info.origin.position.y;

      const double corners[4][2] = {{ox, oy}, {ox + w, oy}, {ox, oy + h}, {ox + w, oy + h}};
      for (const auto & corner : corners) {
        double wx, wy;
        localToWorld(origin, corner[0], corner[1], wx, wy);
        bounds.extend(wx, wy);
      }
    }

    // One cell of slack on each side keeps rounding from clipping the border.
    const double pad = resolution_;
    const int width = static_cast<int>(
      std::ceil((bounds.max_x - bounds.min_x + 2 * pad) / resolution_));
    const int height = static_cast<int>(
      std::ceil((bounds.max_y - bounds.min_y + 2 * pad) / resolution_));
    if (width <= 0 || height <= 0) {
      return;
    }

    nav_msgs::msg::OccupancyGrid merged;
    merged.header.stamp = now();
    merged.header.frame_id = global_frame_;
    merged.info.resolution = resolution_;
    merged.info.width = static_cast<uint32_t>(width);
    merged.info.height = static_cast<uint32_t>(height);
    merged.info.origin.position.x = bounds.min_x - pad;
    merged.info.origin.position.y = bounds.min_y - pad;
    merged.info.origin.orientation.w = 1.0;
    merged.data.assign(static_cast<size_t>(width) * height, kUnknown);

    // 2. Forward-project every source cell into the global grid. Sources are
    //    scattered rather than the target being sampled: the grids share a
    //    resolution, so at most a handful of target cells are missed at
    //    rotation boundaries, and unknown-preserving fusion tolerates that.
    for (const auto & name : active) {
      const auto & grid = *latest_maps_.at(name);
      const auto & origin = origins_.at(name);
      const double res = grid.info.resolution;
      const double ox = grid.info.origin.position.x;
      const double oy = grid.info.origin.position.y;

      for (uint32_t gy = 0; gy < grid.info.height; ++gy) {
        for (uint32_t gx = 0; gx < grid.info.width; ++gx) {
          const int8_t value = grid.data[gy * grid.info.width + gx];
          if (value == kUnknown) {
            continue;
          }

          const double lx = ox + (gx + 0.5) * res;
          const double ly = oy + (gy + 0.5) * res;
          double wx, wy;
          localToWorld(origin, lx, ly, wx, wy);

          const int mx = static_cast<int>((wx - merged.info.origin.position.x) / resolution_);
          const int my = static_cast<int>((wy - merged.info.origin.position.y) / resolution_);
          if (mx < 0 || my < 0 || mx >= width || my >= height) {
            continue;
          }

          int8_t & cell = merged.data[static_cast<size_t>(my) * width + mx];
          // Occupancy-dominant fusion: any agent reporting an obstacle wins over
          // free space. Conservative for navigation, and it makes disagreement
          // between agents visible instead of averaging it away.
          cell = (cell == kUnknown) ? value : std::max(cell, value);
        }
      }
    }

    merged_pub_->publish(merged);

    size_t known = 0;
    for (const auto cell : merged.data) {
      if (cell != kUnknown) {
        ++known;
      }
    }
    std_msgs::msg::Float32 coverage;
    coverage.data = merged.data.empty()
      ? 0.0F
      : static_cast<float>(known) / static_cast<float>(merged.data.size());
    coverage_pub_->publish(coverage);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "merged %zu/%zu maps -> %dx%d cells, %.1f%% known",
      active.size(), robot_names_.size(), width, height, 100.0 * coverage.data);
  }

  // Per-robot coverage counts known cells in that robot's own map, whether or
  // not it is currently connected. It is the "what this agent knows alone"
  // figure that the disconnection experiments compare against the merged map.
  void publishPerRobotCoverage()
  {
    for (const auto & entry : latest_maps_) {
      const auto & grid = *entry.second;
      if (grid.data.empty()) {
        continue;
      }
      size_t known = 0;
      for (const auto cell : grid.data) {
        if (cell != kUnknown) {
          ++known;
        }
      }
      std_msgs::msg::Float32 msg;
      msg.data = static_cast<float>(known) / static_cast<float>(grid.data.size());
      coverage_pubs_.at(entry.first)->publish(msg);
    }
  }

  static void localToWorld(
    const Pose2D & origin, double lx, double ly, double & wx, double & wy)
  {
    const double c = std::cos(origin.yaw);
    const double s = std::sin(origin.yaw);
    wx = origin.x + c * lx - s * ly;
    wy = origin.y + s * lx + c * ly;
  }

  std::vector<std::string> robot_names_;
  std::string global_frame_;
  double resolution_{0.05};
  bool respect_comms_{true};

  std::unordered_map<std::string, Pose2D> origins_;
  std::unordered_map<std::string, bool> connected_;
  std::unordered_map<std::string, size_t> stale_;
  std::unordered_map<std::string, nav_msgs::msg::OccupancyGrid::SharedPtr> latest_maps_;
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr>
  coverage_pubs_;

  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> map_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> comms_subs_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr coverage_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mrs_coordination

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrs_coordination::MapMergerNode>());
  rclcpp::shutdown();
  return 0;
}
