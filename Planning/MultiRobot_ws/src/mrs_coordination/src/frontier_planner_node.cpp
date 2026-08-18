// Deliberative frontier exploration over what the *fleet* has delivered.
//
// The reactive explorer of the first two iterations drives from the laser
// alone: it cannot represent where it has been, let alone where anyone else
// has been, and the second iteration measured what that costs — two of three
// agents entrained onto the same corridor and 27 % of all observation spent on
// ground another agent had already covered.
//
// This node is the classical answer to that, and the baseline against which
// the project's epistemic planner is to be compared. Each agent, once per
// planning cycle:
//
//   1. assembles its *belief* B_i = M_delivered (+) m_i — the fused fleet grid
//      as last received while its own link was up, joined with its own current
//      grid under the same occupancy-dominant operator the merger uses;
//   2. runs one Dijkstra wavefront from its own cell over the free, inflated
//      belief, which yields simultaneously the reachable set, the cost-to-go of
//      every frontier, and (by gradient backtracking) the optimal path to any
//      of them;
//   3. scores every frontier cluster by width minus travel, discounted near the
//      goals other agents have announced, and commits to the best;
//   4. follows the smoothed path with pure pursuit under a laser safety stop.
//
// The point of interest is step 1. An agent whose link is down keeps exploring
// on a *stale* fleet map and stale claims: it plans against what the fleet knew
// when it was last in contact, not against what the fleet knows now. Nothing
// here estimates that gap or reasons about it — this planner cannot represent
// "agent j does not know that I have taken this room". Making that gap explicit
// is the content of the epistemic layer; making it measurable is the point of
// this node.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mrs_coordination
{

namespace
{
constexpr int8_t kUnknown = -1;
constexpr float kInf = std::numeric_limits<float>::infinity();

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// A belief grid in the global frame. Deliberately not a nav_msgs message: the
// planner needs random access by cell index far more than it needs to publish.
struct Grid
{
  double res{0.05};
  double ox{0.0};
  double oy{0.0};
  int w{0};
  int h{0};
  std::vector<int8_t> data;

  bool valid() const {return w > 0 && h > 0;}
  size_t index(int cx, int cy) const {return static_cast<size_t>(cy) * w + cx;}
  bool inside(int cx, int cy) const {return cx >= 0 && cy >= 0 && cx < w && cy < h;}
  int8_t at(int cx, int cy) const {return data[index(cx, cy)];}

  int colOf(double x) const {return static_cast<int>(std::floor((x - ox) / res));}
  int rowOf(double y) const {return static_cast<int>(std::floor((y - oy) / res));}
  double xOf(int cx) const {return ox + (cx + 0.5) * res;}
  double yOf(int cy) const {return oy + (cy + 0.5) * res;}
};

struct Frontier
{
  Point2D centroid;
  int cell{0};        // representative cell, reachable, nearest the centroid
  int size{0};        // frontier cells in the cluster
  double distance{0.0};
  double value{0.0};
};

double normalizeAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}
}  // namespace

class FrontierPlannerNode : public rclcpp::Node
{
public:
  FrontierPlannerNode()
  : rclcpp::Node("frontier_planner")
  {
    declare_parameter<std::string>("robot_name", "robot1");
    declare_parameter<std::vector<std::string>>("robot_names", {"robot1"});
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("base_frame", "robot1/base_footprint");

    // Locomotion. Identical to the reactive explorer's, so that a difference
    // between the two policies is a difference in *decisions* and not in speed.
    declare_parameter<double>("linear_speed", 0.22);
    declare_parameter<double>("angular_speed", 0.5);
    declare_parameter<double>("front_clearance", 0.35);
    declare_parameter<double>("front_half_angle", 0.35);
    declare_parameter<double>("control_rate", 20.0);
    declare_parameter<double>("lookahead", 0.55);
    declare_parameter<double>("goal_tolerance", 0.45);

    // Planning.
    declare_parameter<double>("plan_period", 2.0);
    // The *inscribed* radius of the 0.35 x 0.28 m chassis, not the
    // circumscribed one. Inflating by the circumscribed radius is the
    // conservative choice on a clean costmap, but this grid is scan-matched:
    // walls are smeared by up to a cell or two and a 0.9 m doorway shows up
    // narrower than it is. At 0.28 m the inflation seals most of the building's
    // doorways, the wavefront cannot leave the room the agent is in, and the
    // planner reports the building explored when two thirds of it has never
    // been seen. The laser safety stop, not the inflation, is what keeps the
    // vehicle off the walls.
    declare_parameter<double>("robot_radius", 0.16);
    declare_parameter<int>("occupied_threshold", 60);
    declare_parameter<int>("min_frontier_cells", 6);
    // A frontier closer than this is not worth committing to: the agent would
    // arrive inside its own goal tolerance before the SLAM front end has
    // published a map in which that frontier has moved, re-select it, and
    // oscillate on the spot. Early in a run, when the whole belief is a disc a
    // metre across, that is the difference between exploring and standing
    // still.
    declare_parameter<double>("min_goal_distance", 1.0);
    declare_parameter<double>("gain_weight", 12.0);
    declare_parameter<double>("distance_weight", 1.0);
    declare_parameter<double>("hysteresis", 1.15);
    // Coordination: a frontier within this radius of another agent's announced
    // goal is discounted towards zero. Roughly the diameter of a room here, so
    // two agents are pushed apart by about one work unit and no more.
    declare_parameter<double>("coordination_radius", 6.0);
    declare_parameter<double>("stuck_time", 12.0);
    declare_parameter<double>("stuck_distance", 0.35);
    declare_parameter<double>("blacklist_radius", 1.2);
    declare_parameter<double>("blacklist_time", 90.0);
    declare_parameter<std::string>("log_path", "");

    robot_name_ = get_parameter("robot_name").as_string();
    robot_names_ = get_parameter("robot_names").as_string_array();
    global_frame_ = get_parameter("global_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    linear_speed_ = get_parameter("linear_speed").as_double();
    angular_speed_ = get_parameter("angular_speed").as_double();
    front_clearance_ = get_parameter("front_clearance").as_double();
    front_half_angle_ = get_parameter("front_half_angle").as_double();
    lookahead_ = get_parameter("lookahead").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();

    robot_radius_ = get_parameter("robot_radius").as_double();
    occupied_threshold_ = static_cast<int8_t>(get_parameter("occupied_threshold").as_int());
    min_frontier_cells_ = static_cast<int>(get_parameter("min_frontier_cells").as_int());
    min_goal_distance_ = get_parameter("min_goal_distance").as_double();
    gain_weight_ = get_parameter("gain_weight").as_double();
    distance_weight_ = get_parameter("distance_weight").as_double();
    hysteresis_ = get_parameter("hysteresis").as_double();
    coordination_radius_ = get_parameter("coordination_radius").as_double();
    stuck_time_ = get_parameter("stuck_time").as_double();
    stuck_distance_ = get_parameter("stuck_distance").as_double();
    blacklist_radius_ = get_parameter("blacklist_radius").as_double();
    blacklist_time_ = get_parameter("blacklist_time").as_double();

    const double control_rate = get_parameter("control_rate").as_double();
    const double plan_period = get_parameter("plan_period").as_double();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // Bound to this node: a listener-owned node would not inherit use_sim_time,
    // and a buffer on the wall clock has no common time with sim-stamped
    // transforms, so every lookup fails as an extrapolation.
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("plan", 1);
    goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/mrs/claims", 10);

    own_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", map_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {own_map_ = msg;});

    // The fleet map is *received* only while this agent's link is up. Latching
    // it unconditionally would hand the planner knowledge the radio never
    // delivered, which is exactly the fiction this project exists to remove.
    fleet_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (linked_) {
          fleet_map_ = msg;
          ++deliveries_;
        } else {
          ++withheld_;
        }
      });

    comms_sub_ = create_subscription<std_msgs::msg::Bool>(
      "comms_ok", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {linked_ = msg->data;});

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {scan_ = msg;});

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "explore_enabled", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {enabled_ = msg->data;});

    // Claims travel over the same radio as the maps: an agent out of contact
    // neither announces its goal nor hears anyone else's, and keeps acting on
    // whatever it last heard.
    claim_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/mrs/claims", 20,
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (!linked_ || msg->header.frame_id == robot_name_) {
          return;
        }
        claims_[msg->header.frame_id] = Point2D{msg->point.x, msg->point.y};
      });

    const auto log_path = get_parameter("log_path").as_string();
    if (!log_path.empty()) {
      // The recorder creates the run directory too, but the two nodes start in
      // an arbitrary order and the log must not depend on who won.
      std::filesystem::create_directories(
        std::filesystem::path(log_path).parent_path());
      log_.open(log_path, std::ios::out | std::ios::trunc);
      log_ << "sim_time,linked,state,n_frontiers,n_claims,goal_x,goal_y,"
              "goal_cells,goal_distance,goal_value,path_length,replans,"
              "reached,abandoned,deliveries,withheld\n";
    }

    start_ = now();
    plan_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(plan_period, 0.5)),
      std::bind(&FrontierPlannerNode::plan, this));
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(control_rate, 1.0)),
      std::bind(&FrontierPlannerNode::control, this));

    RCLCPP_INFO(get_logger(), "frontier planner active for %s", robot_name_.c_str());
  }

  ~FrontierPlannerNode() override
  {
    if (log_.is_open()) {
      log_.close();
    }
  }

private:
  // ----------------------------------------------------------------------
  // belief assembly
  // ----------------------------------------------------------------------

  // B_i = M_delivered (+) m_i, on a canvas covering both, under the merger's
  // occupancy-dominant join: unknown is absorbed by anything known, and any
  // agent reporting an obstacle beats free space.
  bool buildBelief()
  {
    if (!own_map_ && !fleet_map_) {
      return false;
    }

    double min_x = kInf, min_y = kInf, max_x = -kInf, max_y = -kInf;
    double res = 0.05;

    if (fleet_map_) {
      const auto & info = fleet_map_->info;
      res = info.resolution;
      min_x = std::min(min_x, info.origin.position.x);
      min_y = std::min(min_y, info.origin.position.y);
      max_x = std::max(max_x, info.origin.position.x + info.width * info.resolution);
      max_y = std::max(max_y, info.origin.position.y + info.height * info.resolution);
    }

    // The agent's own grid lives in <ns>/map, whose pose in the global frame is
    // the static transform the merger publishes from the deployment
    // configuration. Looking it up rather than re-deriving it keeps the one
    // known-initialisation assumption in one place.
    bool own_ok = false;
    double tx = 0.0, ty = 0.0, tyaw = 0.0;
    if (own_map_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(
          global_frame_, own_map_->header.frame_id, tf2::TimePointZero);
        tx = tf.transform.translation.x;
        ty = tf.transform.translation.y;
        tyaw = tf2::getYaw(tf.transform.rotation);
        own_ok = true;
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000, "no transform to own map: %s", error.what());
      }
    }

    if (own_ok) {
      const auto & info = own_map_->info;
      res = info.resolution;
      const double w = info.width * info.resolution;
      const double h = info.height * info.resolution;
      const double corners[4][2] = {
        {info.origin.position.x, info.origin.position.y},
        {info.origin.position.x + w, info.origin.position.y},
        {info.origin.position.x, info.origin.position.y + h},
        {info.origin.position.x + w, info.origin.position.y + h}};
      const double c = std::cos(tyaw), s = std::sin(tyaw);
      for (const auto & corner : corners) {
        const double wx = tx + c * corner[0] - s * corner[1];
        const double wy = ty + s * corner[0] + c * corner[1];
        min_x = std::min(min_x, wx);
        min_y = std::min(min_y, wy);
        max_x = std::max(max_x, wx);
        max_y = std::max(max_y, wy);
      }
    }

    if (!(min_x < max_x) || !(min_y < max_y)) {
      return false;
    }

    belief_.res = res;
    belief_.ox = min_x - res;
    belief_.oy = min_y - res;
    belief_.w = static_cast<int>(std::ceil((max_x - min_x + 2 * res) / res));
    belief_.h = static_cast<int>(std::ceil((max_y - min_y + 2 * res) / res));
    belief_.data.assign(static_cast<size_t>(belief_.w) * belief_.h, kUnknown);

    if (fleet_map_) {
      paint(*fleet_map_, 0.0, 0.0, 0.0);
    }
    if (own_ok) {
      paint(*own_map_, tx, ty, tyaw);
    }
    return true;
  }

  // Forward-project one source grid onto the belief canvas, as the merger does:
  // scattering source cells rather than sampling the target keeps rotated grids
  // from acquiring interpolation artefacts, and the join tolerates the few
  // target cells a rotation misses.
  void paint(const nav_msgs::msg::OccupancyGrid & grid, double tx, double ty, double tyaw)
  {
    const double c = std::cos(tyaw), s = std::sin(tyaw);
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
        const double wx = tx + c * lx - s * ly;
        const double wy = ty + s * lx + c * ly;

        const int cx = belief_.colOf(wx);
        const int cy = belief_.rowOf(wy);
        if (!belief_.inside(cx, cy)) {
          continue;
        }
        int8_t & cell = belief_.data[belief_.index(cx, cy)];
        cell = (cell == kUnknown) ? value : std::max(cell, value);
      }
    }
  }

  // ----------------------------------------------------------------------
  // costmap, wavefront, frontiers
  // ----------------------------------------------------------------------

  // Lethal = occupied, or within the robot's radius of an occupied cell. The
  // inflation is a multi-source BFS in cells rather than a Euclidean distance
  // transform: at this radius the two differ by less than one cell and the BFS
  // costs nothing.
  void inflate(double robot_radius)
  {
    const size_t n = belief_.data.size();
    lethal_.assign(n, 0);

    const int radius = std::max(1, static_cast<int>(std::round(robot_radius / belief_.res)));
    std::vector<int16_t> depth(n, -1);
    std::deque<int> queue;

    for (size_t i = 0; i < n; ++i) {
      if (belief_.data[i] >= occupied_threshold_) {
        lethal_[i] = 1;
        depth[i] = 0;
        queue.push_back(static_cast<int>(i));
      }
    }

    while (!queue.empty()) {
      const int index = queue.front();
      queue.pop_front();
      if (depth[index] >= radius) {
        continue;
      }
      const int cx = index % belief_.w;
      const int cy = index / belief_.w;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = cx + dx, ny = cy + dy;
          if ((dx == 0 && dy == 0) || !belief_.inside(nx, ny)) {
            continue;
          }
          const size_t ni = belief_.index(nx, ny);
          if (depth[ni] >= 0) {
            continue;
          }
          depth[ni] = static_cast<int16_t>(depth[index] + 1);
          lethal_[ni] = 1;
          queue.push_back(static_cast<int>(ni));
        }
      }
    }
  }

  bool traversable(size_t i) const
  {
    return belief_.data[i] >= 0 && belief_.data[i] < occupied_threshold_ && !lethal_[i];
  }

  // One Dijkstra wavefront from the agent's own cell over the traversable
  // belief. It answers three questions at once — what is reachable, how far
  // every frontier is, and which way to walk — which is why the planner runs a
  // single search per cycle instead of one A* per candidate goal.
  void wavefront(int start)
  {
    const size_t n = belief_.data.size();
    cost_.assign(n, kInf);
    parent_.assign(n, -1);

    using Item = std::pair<float, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    cost_[start] = 0.0f;
    open.emplace(0.0f, start);

    const float step = static_cast<float>(belief_.res);
    const float diag = step * static_cast<float>(std::sqrt(2.0));

    while (!open.empty()) {
      const auto [c, index] = open.top();
      open.pop();
      if (c > cost_[index]) {
        continue;
      }
      const int cx = index % belief_.w;
      const int cy = index / belief_.w;

      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int nx = cx + dx, ny = cy + dy;
          if (!belief_.inside(nx, ny)) {
            continue;
          }
          const size_t ni = belief_.index(nx, ny);
          if (!traversable(ni)) {
            continue;
          }
          const float nc = c + ((dx == 0 || dy == 0) ? step : diag);
          if (nc < cost_[ni]) {
            cost_[ni] = nc;
            parent_[ni] = index;
            open.emplace(nc, static_cast<int>(ni));
          }
        }
      }
    }
  }

  // A frontier cell is reachable free space with an unobserved 4-neighbour:
  // the boundary between what the agent believes and what nobody has looked at.
  std::vector<Frontier> findFrontiers()
  {
    const size_t n = belief_.data.size();
    std::vector<uint8_t> is_frontier(n, 0);

    for (int cy = 1; cy < belief_.h - 1; ++cy) {
      for (int cx = 1; cx < belief_.w - 1; ++cx) {
        const size_t i = belief_.index(cx, cy);
        if (!std::isfinite(cost_[i])) {
          continue;
        }
        if (belief_.at(cx - 1, cy) == kUnknown || belief_.at(cx + 1, cy) == kUnknown ||
          belief_.at(cx, cy - 1) == kUnknown || belief_.at(cx, cy + 1) == kUnknown)
        {
          is_frontier[i] = 1;
        }
      }
    }

    std::vector<Frontier> out;
    std::vector<uint8_t> seen(n, 0);
    std::vector<int> cluster;

    for (size_t seed = 0; seed < n; ++seed) {
      if (!is_frontier[seed] || seen[seed]) {
        continue;
      }
      cluster.clear();
      std::deque<int> queue{static_cast<int>(seed)};
      seen[seed] = 1;

      while (!queue.empty()) {
        const int index = queue.front();
        queue.pop_front();
        cluster.push_back(index);
        const int cx = index % belief_.w;
        const int cy = index / belief_.w;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = cx + dx, ny = cy + dy;
            if (!belief_.inside(nx, ny)) {
              continue;
            }
            const size_t ni = belief_.index(nx, ny);
            if (is_frontier[ni] && !seen[ni]) {
              seen[ni] = 1;
              queue.push_back(static_cast<int>(ni));
            }
          }
        }
      }

      if (static_cast<int>(cluster.size()) < min_frontier_cells_) {
        continue;
      }

      Frontier frontier;
      frontier.size = static_cast<int>(cluster.size());
      double sx = 0.0, sy = 0.0;
      for (const int index : cluster) {
        sx += belief_.xOf(index % belief_.w);
        sy += belief_.yOf(index / belief_.w);
      }
      frontier.centroid = Point2D{sx / cluster.size(), sy / cluster.size()};

      // The centroid of a curved frontier need not lie on the frontier, so the
      // goal is the cluster member nearest to it — a cell the wavefront has
      // already proved reachable.
      double best = kInf;
      for (const int index : cluster) {
        const double dx = belief_.xOf(index % belief_.w) - frontier.centroid.x;
        const double dy = belief_.yOf(index / belief_.w) - frontier.centroid.y;
        const double d = dx * dx + dy * dy;
        if (d < best) {
          best = d;
          frontier.cell = index;
        }
      }
      frontier.distance = cost_[frontier.cell];
      out.push_back(frontier);
    }

    return out;
  }

  // Cost-utility with a coordination discount. Utility is the frontier's width
  // in metres, a proxy for the information behind it; cost is the travel the
  // wavefront measured; the discount removes the utility of frontiers another
  // agent has already announced it is driving to, which is the whole of the
  // collaboration and is only as good as the last claim that arrived.
  double score(const Frontier & frontier) const
  {
    double discount = 1.0;
    for (const auto & [name, claim] : claims_) {
      (void)name;
      const double dx = frontier.centroid.x - claim.x;
      const double dy = frontier.centroid.y - claim.y;
      const double d = std::hypot(dx, dy);
      discount = std::min(discount, std::min(1.0, d / std::max(coordination_radius_, 1e-3)));
    }
    const double gain = gain_weight_ * frontier.size * belief_.res * discount;
    return gain - distance_weight_ * frontier.distance;
  }

  bool blacklisted(const Point2D & p) const
  {
    const double t = elapsed();
    for (const auto & entry : blacklist_) {
      if (t - entry.second > blacklist_time_) {
        continue;
      }
      if (std::hypot(p.x - entry.first.x, p.y - entry.first.y) < blacklist_radius_) {
        return true;
      }
    }
    return false;
  }

  // ----------------------------------------------------------------------
  // planning cycle
  // ----------------------------------------------------------------------

  void plan()
  {
    if (!enabled_) {
      return;
    }
    const auto pose = lookupPose();
    if (!pose) {
      return;
    }
    if (!buildBelief()) {
      return;
    }

    inflate(robot_radius_);

    int start = startCell(pose->x, pose->y);
    if (start < 0) {
      // The agent's own cell can read as lethal while it sits close to a wall,
      // in which case there is nothing to search from. Reporting it and waiting
      // is right: the next scan usually clears it.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no free cell under the robot; holding");
      state_ = "no_start";
      writeLog(0, 0.0);
      return;
    }

    wavefront(start);
    auto frontiers = findFrontiers();
    bool relaxed = false;

    // Relaxed retry. The inflation is a preference, not a constraint — the
    // laser safety stop is what keeps the vehicle off the walls — and on a
    // scan-matched grid of a building with 0.9 m doorways it can seal an agent
    // into a pocket of its own map. When that happens the agent reads its
    // situation as 'nothing left to explore', which is indistinguishable in the
    // log from having genuinely finished, and it stops for the rest of the run.
    // Measured in this building: at the nominal 0.16 m an agent parked in the
    // east corridor had a 6.3 m2 reachable component and no frontier at all,
    // while one cell less of inflation gave it 23.6 m2 and 380 frontier cells.
    // So when the nominal radius yields nothing, the search is repeated once at
    // one cell of clearance before the agent is allowed to conclude it is done.
    if (frontiers.empty()) {
      inflate(belief_.res);
      const int relaxed_start = startCell(pose->x, pose->y);
      if (relaxed_start >= 0) {
        wavefront(relaxed_start);
        frontiers = findFrontiers();
        relaxed = !frontiers.empty();
        if (relaxed) {
          start = relaxed_start;
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "%s sealed at %.2f m inflation; planning at %.2f m",
            robot_name_.c_str(), robot_radius_, belief_.res);
        }
      }
      if (!relaxed) {
        // Restore the nominal costmap, so that a genuinely finished agent is
        // not left holding a permissive one.
        inflate(robot_radius_);
        wavefront(start);
      }
    }
    n_frontiers_ = static_cast<int>(frontiers.size());

    Frontier * best = nullptr;
    double best_value = -kInf;
    int usable = 0;  // reachable and not blacklisted, for the log
    for (auto & frontier : frontiers) {
      if (!std::isfinite(frontier.distance) || blacklisted(frontier.centroid)) {
        continue;
      }
      ++usable;
      if (frontier.distance < min_goal_distance_) {
        continue;
      }
      frontier.value = score(frontier);
      if (frontier.value > best_value) {
        best_value = frontier.value;
        best = &frontier;
      }
    }

    if (!best && !frontiers.empty()) {
      // There is a frontier but none the planner will commit to: either they
      // are all within arm's reach, which is the state of the world in the
      // first seconds of a run when the belief is a disc a metre across, or
      // they are all blacklisted after failed approaches. Drive forward and let
      // the next scan enlarge the belief and the blacklist expire. One reactive
      // primitive, used to bootstrap the deliberative one and to get out from
      // under it — standing still in either case ends the agent's run, since
      // nothing about a stationary agent's situation changes.
      state_ = "probe";
      goal_valid_ = false;
      path_.clear();
      writeLog(0, 0.0);
      return;
    }

    if (!best) {
      // No reachable frontier: either the agent's own component of the building
      // is exhausted, or everything left is blacklisted. Stop, and keep
      // planning — a fleet map arriving on the next link-up can reopen the
      // problem, which is precisely the case worth logging.
      state_ = "idle";
      goal_valid_ = false;
      path_.clear();
      stop();
      writeLog(0, 0.0);
      return;
    }

    // Hysteresis: hold the current goal unless a candidate is clearly better.
    // Without it the discount from an arriving claim and the shrinking of a
    // frontier as it is observed make the argmax oscillate between two rooms,
    // and the agent spends the run in the corridor between them.
    if (goal_valid_) {
      const Frontier * incumbent = nullptr;
      double best_d = kInf;
      for (auto & frontier : frontiers) {
        const double d = std::hypot(
          frontier.centroid.x - goal_.x, frontier.centroid.y - goal_.y);
        if (d < best_d && d < 2.0 && std::isfinite(frontier.distance)) {
          best_d = d;
          incumbent = &frontier;
        }
      }
      if (incumbent && score(*incumbent) * hysteresis_ >= best_value) {
        best = const_cast<Frontier *>(incumbent);
        best_value = score(*incumbent);
      }
    }

    if (std::hypot(best->centroid.x - goal_.x, best->centroid.y - goal_.y) > 0.5) {
      ++replans_;
    }
    goal_ = best->centroid;
    goal_valid_ = true;
    goal_cells_ = best->size;
    goal_distance_ = best->distance;
    goal_value_ = best_value;
    // Logged distinctly, so that a run can be audited for how much of it was
    // driven on the nominal costmap and how much on the relaxed one.
    state_ = relaxed ? "driving_relaxed" : "driving";

    buildPath(best->cell);
    publishClaim();
    writeLog(best->size, pathLength());
  }

  int startCell(double x, double y) const
  {
    const int cx = belief_.colOf(x);
    const int cy = belief_.rowOf(y);
    if (!belief_.inside(cx, cy)) {
      return -1;
    }
    if (traversable(belief_.index(cx, cy))) {
      return static_cast<int>(belief_.index(cx, cy));
    }
    // Nearest traversable cell within half a metre, so that standing next to a
    // wall — where the inflation swallows the robot's own cell — does not abort
    // the search.
    const int radius = static_cast<int>(std::round(0.5 / belief_.res));
    int best = -1;
    double best_d = kInf;
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const int nx = cx + dx, ny = cy + dy;
        if (!belief_.inside(nx, ny)) {
          continue;
        }
        const size_t ni = belief_.index(nx, ny);
        if (!traversable(ni)) {
          continue;
        }
        const double d = dx * dx + dy * dy;
        if (d < best_d) {
          best_d = d;
          best = static_cast<int>(ni);
        }
      }
    }
    return best;
  }

  // Gradient backtracking down the wavefront gives the optimal path; the
  // shortcut pass then replaces its staircase with the straight segments a
  // differential drive can actually follow without weaving.
  void buildPath(int goal_cell)
  {
    std::vector<Point2D> raw;
    for (int index = goal_cell; index >= 0; index = parent_[index]) {
      raw.push_back(Point2D{belief_.xOf(index % belief_.w), belief_.yOf(index / belief_.w)});
      if (parent_[index] < 0) {
        break;
      }
    }
    std::reverse(raw.begin(), raw.end());

    path_.clear();
    if (raw.empty()) {
      return;
    }
    size_t i = 0;
    path_.push_back(raw.front());
    while (i + 1 < raw.size()) {
      size_t j = raw.size() - 1;
      while (j > i + 1 && !lineClear(raw[i], raw[j])) {
        --j;
      }
      path_.push_back(raw[j]);
      i = j;
    }
    path_index_ = 0;
    publishPath();
  }

  bool lineClear(const Point2D & a, const Point2D & b) const
  {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const int steps = static_cast<int>(std::ceil(std::hypot(dx, dy) / (belief_.res * 0.5)));
    for (int k = 0; k <= steps; ++k) {
      const double t = steps ? static_cast<double>(k) / steps : 0.0;
      const int cx = belief_.colOf(a.x + t * dx);
      const int cy = belief_.rowOf(a.y + t * dy);
      if (!belief_.inside(cx, cy) || !traversable(belief_.index(cx, cy))) {
        return false;
      }
    }
    return true;
  }

  double pathLength() const
  {
    double total = 0.0;
    for (size_t i = 1; i < path_.size(); ++i) {
      total += std::hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y);
    }
    return total;
  }

  // ----------------------------------------------------------------------
  // control
  // ----------------------------------------------------------------------

  void control()
  {
    if (!enabled_) {
      return;
    }
    const auto pose = lookupPose();
    if (!pose) {
      stop();
      return;
    }

    checkStuck(*pose);

    if (elapsed() < recover_until_) {
      geometry_msgs::msg::Twist cmd;
      if (rearBlocked()) {
        cmd.angular.z = angular_speed_;
      } else {
        cmd.linear.x = -0.5 * linear_speed_;
      }
      cmd_pub_->publish(cmd);
      return;
    }

    if (state_ == "probe" && !goal_valid_) {
      geometry_msgs::msg::Twist cmd;
      if (frontBlocked()) {
        cmd.angular.z = angular_speed_;
      } else {
        cmd.linear.x = linear_speed_;
      }
      cmd_pub_->publish(cmd);
      return;
    }

    if (!goal_valid_ || path_.size() < 2) {
      stop();
      return;
    }

    if (std::hypot(goal_.x - pose->x, goal_.y - pose->y) < goal_tolerance_) {
      ++reached_;
      goal_valid_ = false;
      path_.clear();
      stop();
      return;
    }

    // Advance along the path, then take the first point beyond the lookahead
    // distance as the carrot.
    while (path_index_ + 1 < path_.size() &&
      std::hypot(path_[path_index_].x - pose->x, path_[path_index_].y - pose->y) < lookahead_)
    {
      ++path_index_;
    }
    const Point2D & carrot = path_[std::min(path_index_, path_.size() - 1)];

    const double heading = std::atan2(carrot.y - pose->y, carrot.x - pose->x);
    const double error = normalizeAngle(heading - pose->yaw);

    geometry_msgs::msg::Twist cmd;
    if (std::fabs(error) > 0.7) {
      // Turn in place rather than arc: the arc a differential drive would take
      // at this heading error leaves the corridor.
      cmd.angular.z = std::copysign(angular_speed_, error);
    } else {
      cmd.linear.x = linear_speed_ * std::max(0.25, 1.0 - std::fabs(error) / 0.7);
      cmd.angular.z = std::clamp(1.6 * error, -angular_speed_, angular_speed_);
    }

    // Safety stop. The plan is computed on a belief that is up to a planning
    // period old and coarser than the laser; the scan is the authority on what
    // is in front of the vehicle right now, including the other agents, which
    // the static-world belief never contains.
    if (frontBlocked()) {
      blocked_since_ = std::isfinite(blocked_since_) ? blocked_since_ : elapsed();
      cmd.linear.x = 0.0;
      cmd.angular.z = std::copysign(angular_speed_, error != 0.0 ? error : 1.0);
      if (elapsed() - blocked_since_ > 4.0) {
        abandon(pose->x, pose->y);
      }
    } else {
      blocked_since_ = kInf;
    }

    cmd_pub_->publish(cmd);
  }

  bool rearBlocked() const
  {
    return sectorBlocked(M_PI, 0.5, 0.30);
  }

  bool frontBlocked() const
  {
    return sectorBlocked(0.0, front_half_angle_, front_clearance_);
  }

  bool sectorBlocked(double centre, double half_width, double clearance) const
  {
    if (!scan_) {
      return false;
    }
    const auto & scan = *scan_;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double angle = normalizeAngle(scan.angle_min + i * scan.angle_increment);
      if (std::fabs(normalizeAngle(angle - centre)) > half_width) {
        continue;
      }
      const double range = scan.ranges[i];
      if (std::isfinite(range) && range >= scan.range_min && range < clearance) {
        return true;
      }
    }
    return false;
  }

  void checkStuck(const Pose2D & pose)
  {
    const double t = elapsed();
    const Point2D here{pose.x, pose.y};
    if (!goal_valid_) {
      last_progress_ = t;
      progress_ref_ = here;
      return;
    }
    if (std::hypot(here.x - progress_ref_.x, here.y - progress_ref_.y) > stuck_distance_) {
      last_progress_ = t;
      progress_ref_ = here;
      return;
    }
    if (t - last_progress_ > stuck_time_) {
      abandon(pose.x, pose.y);
      last_progress_ = t;
      progress_ref_ = here;
    }
  }

  // Give up on the current goal and refuse it for a while. A frontier the
  // planner believes is reachable but the vehicle cannot actually reach — a
  // doorway narrower than the map says, a chair the grid smeared into free
  // space — would otherwise be re-selected on every cycle for the rest of the
  // run, because nothing about it changes when the attempt fails.
  void abandon(double, double)
  {
    if (!goal_valid_) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "%s abandoning frontier at (%.1f, %.1f)",
      robot_name_.c_str(), goal_.x, goal_.y);
    blacklist_.emplace_back(goal_, elapsed());
    ++abandoned_;
    // Back out of whatever the vehicle failed to get through before planning
    // again: replanning from a pose wedged in a doorway usually produces the
    // same approach and the same failure.
    recover_until_ = elapsed() + 2.5;
    goal_valid_ = false;
    path_.clear();
    blocked_since_ = kInf;
    stop();
  }

  void stop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  // ----------------------------------------------------------------------
  // plumbing
  // ----------------------------------------------------------------------

  std::optional<Pose2D> lookupPose()
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        global_frame_, base_frame_, tf2::TimePointZero);
      return Pose2D{tf.transform.translation.x, tf.transform.translation.y,
        tf2::getYaw(tf.transform.rotation)};
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000, "no pose: %s", error.what());
      return std::nullopt;
    }
  }

  double elapsed() const {return (now() - start_).seconds();}

  void publishClaim()
  {
    if (!linked_) {
      return;
    }
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = robot_name_;
    msg.point.x = goal_.x;
    msg.point.y = goal_.y;
    goal_pub_->publish(msg);
  }

  void publishPath()
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = global_frame_;
    msg.poses.reserve(path_.size());
    for (const auto & point : path_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }
    path_pub_->publish(msg);
  }

  void writeLog(int goal_cells, double path_length)
  {
    if (!log_.is_open()) {
      return;
    }
    log_ << elapsed() << ',' << (linked_ ? 1 : 0) << ',' << state_ << ','
         << n_frontiers_ << ',' << claims_.size() << ',';
    if (goal_valid_) {
      log_ << goal_.x << ',' << goal_.y;
    } else {
      log_ << ',';
    }
    log_ << ',' << goal_cells << ',' << goal_distance_ << ',' << goal_value_ << ','
         << path_length << ',' << replans_ << ',' << reached_ << ',' << abandoned_ << ','
         << deliveries_ << ',' << withheld_ << '\n';
    log_.flush();
  }

  // configuration
  std::string robot_name_;
  std::vector<std::string> robot_names_;
  std::string global_frame_;
  std::string base_frame_;
  double linear_speed_{0.22};
  double angular_speed_{0.5};
  double front_clearance_{0.35};
  double front_half_angle_{0.35};
  double lookahead_{0.55};
  double goal_tolerance_{0.45};
  double robot_radius_{0.16};
  int8_t occupied_threshold_{60};
  int min_frontier_cells_{6};
  double min_goal_distance_{1.0};
  double gain_weight_{12.0};
  double distance_weight_{1.0};
  double hysteresis_{1.15};
  double coordination_radius_{6.0};
  double stuck_time_{12.0};
  double stuck_distance_{0.35};
  double blacklist_radius_{1.2};
  double blacklist_time_{90.0};

  // state
  Grid belief_;
  std::vector<uint8_t> lethal_;
  std::vector<float> cost_;
  std::vector<int> parent_;
  std::vector<Point2D> path_;
  size_t path_index_{0};
  Point2D goal_;
  bool goal_valid_{false};
  int goal_cells_{0};
  double goal_distance_{0.0};
  double goal_value_{0.0};
  int n_frontiers_{0};
  int replans_{0};
  int reached_{0};
  int abandoned_{0};
  int deliveries_{0};
  int withheld_{0};
  std::string state_{"init"};
  bool linked_{true};
  bool enabled_{true};
  double blocked_since_{kInf};
  double recover_until_{-1.0};
  double last_progress_{0.0};
  Point2D progress_ref_;
  std::vector<std::pair<Point2D, double>> blacklist_;
  std::unordered_map<std::string, Point2D> claims_;
  rclcpp::Time start_;
  std::ofstream log_;

  nav_msgs::msg::OccupancyGrid::SharedPtr own_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr fleet_map_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr own_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr fleet_map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr comms_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr claim_sub_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace mrs_coordination

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrs_coordination::FrontierPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
