// The roadmap: the object that both halves of this workspace agree on.
//
// The planner sees it as `(connected a b)` and `(= (travel_time a b) t)`; the
// `move` performer sees it as a pair of coordinates to drive to. Both read the
// same YAML file, produced by `roadmap_builder` from the warehouse's own map,
// so there is exactly one place where a symbol is bound to a position and
// exactly one place where an edge is proved drivable.

#ifndef WAREHOUSE_TOOLS__ROADMAP_HPP_
#define WAREHOUSE_TOOLS__ROADMAP_HPP_

#include <map>
#include <string>
#include <vector>

#include "warehouse_tools/occupancy_map.hpp"

namespace warehouse_tools
{

// What a waypoint is *for*. The kinds are not decoration: the problem
// generator deals crates over the storage bays, sends deliveries to the docks
// and starts robots on corridor nodes, so the scenario's structure is carried
// by this field rather than by naming conventions.
enum class WaypointKind
{
  Corridor,
  Storage,
  Dock,
  Charger,
};

std::string toString(WaypointKind kind);
WaypointKind kindFromString(const std::string & text);

struct Waypoint
{
  std::string name;
  Point2D position;
  WaypointKind kind{WaypointKind::Corridor};
  double clearance_m{0.0};      // measured, not assumed
};

struct Edge
{
  std::string from;
  std::string to;
  double length_m{0.0};
  double travel_time_s{0.0};
};

class Roadmap
{
public:
  // Keeps every waypoint whose footprint fits, and connects two of them when
  // the straight segment between them is free at the same clearance and no
  // longer than `max_edge_m`. Throws if a named waypoint does not fit, because
  // the alternative — dropping it quietly — produces a problem file that is
  // subtly not the scenario that was asked for.
  static Roadmap build(
    const OccupancyMap & map,
    const std::vector<Waypoint> & requested,
    double max_edge_m,
    double speed_mps,
    double turn_penalty_s);

  static Roadmap load(const std::string & yaml_path);
  void save(const std::string & yaml_path, const std::string & map_path) const;

  // A roadmap in two pieces is a planner that answers "no plan" for reasons
  // that have nothing to do with the domain, so the builder refuses one.
  bool isConnected(std::vector<std::string> * unreachable = nullptr) const;

  const std::vector<Waypoint> & waypoints() const {return waypoints_;}
  const std::vector<Edge> & edges() const {return edges_;}
  std::vector<std::string> namesOfKind(WaypointKind kind) const;
  const Waypoint * find(const std::string & name) const;

  double meanDegree() const;

  // Shortest travel time between two waypoints over the graph, by Dijkstra.
  // Used to report what a plan *could* have cost, and to spread the robots'
  // start positions over the building rather than over the coordinate frame.
  double shortestTime(const std::string & from, const std::string & to) const;

  struct Metadata
  {
    double speed_mps{0.22};
    double turn_penalty_s{3.0};
    double radius_m{0.32};
    double max_edge_m{6.0};
    std::string map_path;
  };
  const Metadata & metadata() const {return meta_;}

private:
  std::vector<Waypoint> waypoints_;
  std::vector<Edge> edges_;
  Metadata meta_;
};

}  // namespace warehouse_tools

#endif  // WAREHOUSE_TOOLS__ROADMAP_HPP_
