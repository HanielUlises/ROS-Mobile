// Records a run for offline analysis.
//
// Two products:
//   * A sequence of merged-grid snapshots (`frame_%05d.grid`), each carrying its
//     own origin and resolution so the growing map can be rendered on a common
//     canvas afterwards.
//   * `coverage.csv`, one row per snapshot with simulation time, the merged
//     known-cell fraction, and each robot's individual coverage and link state.
//
// Coverage is reported as explored area in square metres rather than a fraction
// of the canvas: the canvas grows as the map does, so a fraction is not
// comparable across time, whereas area is. The merged/individual gap under
// intermittent connectivity is the quantity the project's evaluation targets.

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

namespace mrs_coordination
{

namespace
{
constexpr int8_t kUnknown = -1;

// Custom container rather than PGM: PGM cannot carry the grid origin, and the
// origin is what lets every frame be composited onto one canvas.
//   header: "MRSGRID <width> <height> <resolution> <origin_x> <origin_y> <stamp>
//            <tx> <ty> <tyaw>\n"
//   body:   width*height raw int8 cells, row-major, y increasing
//
// The trailing pose is the SE(2) transform carrying this grid's frame into the
// global frame. It is identity for the fused grid, which is already global, and
// the agent's deployment pose for a per-agent grid. Carrying it in the snapshot
// means the renderer can place per-agent grids on the same canvas as the fused
// one without needing the fleet configuration.
constexpr char kMagic[] = "MRSGRID";

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};
}  // namespace

class MapRecorderNode : public rclcpp::Node
{
public:
  MapRecorderNode()
  : rclcpp::Node("map_recorder")
  {
    declare_parameter<std::vector<std::string>>("robot_names", {"robot1", "robot2"});
    declare_parameter<std::vector<double>>("initial_poses", std::vector<double>{});
    declare_parameter<std::string>("output_dir", "/tmp/mrs_run");
    declare_parameter<double>("sample_period", 2.0);

    robot_names_ = get_parameter("robot_names").as_string_array();
    output_dir_ = get_parameter("output_dir").as_string();
    const double period = get_parameter("sample_period").as_double();

    const auto flat_poses = get_parameter("initial_poses").as_double_array();
    if (flat_poses.size() != robot_names_.size() * 3) {
      RCLCPP_FATAL(
        get_logger(),
        "initial_poses must hold 3 values (x, y, yaw) per robot: expected %zu, got %zu",
        robot_names_.size() * 3, flat_poses.size());
      throw std::runtime_error("initial_poses / robot_names size mismatch");
    }
    for (size_t i = 0; i < robot_names_.size(); ++i) {
      origins_[robot_names_[i]] =
        Pose2D{flat_poses[3 * i], flat_poses[3 * i + 1], flat_poses[3 * i + 2]};
    }

    std::filesystem::create_directories(output_dir_);

    csv_.open(output_dir_ + "/coverage.csv", std::ios::out | std::ios::trunc);
    csv_ << "frame,sim_time,merged_area_m2";
    for (const auto & name : robot_names_) {
      csv_ << "," << name << "_area_m2," << name << "_linked";
    }
    csv_ << "\n";

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    merged_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {merged_ = msg;});

    for (const auto & name : robot_names_) {
      areas_[name] = 0.0;
      linked_[name] = true;

      robot_map_subs_.push_back(
        create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/" + name + "/map", qos,
          [this, name](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            areas_[name] = knownArea(*msg);
            robot_maps_[name] = msg;
          }));

      comms_subs_.push_back(
        create_subscription<std_msgs::msg::Bool>(
          "/" + name + "/comms_ok", 10,
          [this, name](std_msgs::msg::Bool::SharedPtr msg) {linked_[name] = msg->data;}));
    }

    start_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(period, 0.2)),
      std::bind(&MapRecorderNode::sample, this));

    RCLCPP_INFO(get_logger(), "recording run to %s", output_dir_.c_str());
  }

  ~MapRecorderNode() override
  {
    if (csv_.is_open()) {
      csv_.close();
    }
  }

private:
  static double knownArea(const nav_msgs::msg::OccupancyGrid & grid)
  {
    size_t known = 0;
    for (const auto cell : grid.data) {
      if (cell != kUnknown) {
        ++known;
      }
    }
    const double cell_area = grid.info.resolution * grid.info.resolution;
    return static_cast<double>(known) * cell_area;
  }

  static void writeGrid(
    const std::string & path, const nav_msgs::msg::OccupancyGrid & grid, double t,
    const Pose2D & pose)
  {
    std::ofstream out(path, std::ios::out | std::ios::binary);
    out << kMagic << ' ' << grid.info.width << ' ' << grid.info.height << ' '
        << grid.info.resolution << ' ' << grid.info.origin.position.x << ' '
        << grid.info.origin.position.y << ' ' << t << ' '
        << pose.x << ' ' << pose.y << ' ' << pose.yaw << '\n';
    out.write(reinterpret_cast<const char *>(grid.data.data()),
      static_cast<std::streamsize>(grid.data.size()));
  }

  void sample()
  {
    if (!merged_) {
      return;
    }

    const double t = (now() - start_).seconds();
    const auto & grid = *merged_;

    char filename[96];
    std::snprintf(filename, sizeof(filename), "/frame_%05d.grid", frame_index_);
    writeGrid(output_dir_ + filename, grid, t, Pose2D{});

    // Per-agent snapshots as well, so the individual and fused estimates can be
    // compared side by side rather than only through their scalar areas. These
    // are each agent's own map, ungated by connectivity.
    for (const auto & entry : robot_maps_) {
      std::snprintf(filename, sizeof(filename), "/frame_%05d_%s.grid",
        frame_index_, entry.first.c_str());
      writeGrid(output_dir_ + filename, *entry.second, t, origins_.at(entry.first));
    }

    csv_ << frame_index_ << ',' << t << ',' << knownArea(grid);
    for (const auto & name : robot_names_) {
      csv_ << ',' << areas_[name] << ',' << (linked_[name] ? 1 : 0);
    }
    csv_ << '\n';
    csv_.flush();

    ++frame_index_;
  }

  std::vector<std::string> robot_names_;
  std::string output_dir_;
  int frame_index_{0};
  rclcpp::Time start_;
  std::ofstream csv_;

  nav_msgs::msg::OccupancyGrid::SharedPtr merged_;
  std::unordered_map<std::string, double> areas_;
  std::unordered_map<std::string, bool> linked_;
  std::unordered_map<std::string, Pose2D> origins_;
  std::unordered_map<std::string, nav_msgs::msg::OccupancyGrid::SharedPtr> robot_maps_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> robot_map_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> comms_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mrs_coordination

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrs_coordination::MapRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
