// Derive the planner's waypoint graph from the warehouse's own occupancy map.
//
// A task planner's symbols are only as good as their correspondence with the
// floor underneath them, and the usual way to break that correspondence is to
// write the waypoints and the `connected` relation by hand: a waypoint ends up
// inside a shelf, or two waypoints are declared adjacent through a wall, and
// the plan is valid while the execution is impossible.
//
// The station positions below are hand-placed — they are the semantic content
// of the scenario and no map can supply them — but not one of them is trusted:
// each is checked against the map at the vehicle's own clearance, each edge is
// proved to be a drivable straight segment, and the tool refuses to write a
// roadmap that fails either test.
//
//   ros2 run warehouse_tools roadmap_builder
//       --map install/warehouse_planning/share/warehouse_planning/maps/warehouse.yaml
//       --output src/warehouse_planning/config/roadmap.yaml

#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "warehouse_tools/occupancy_map.hpp"
#include "warehouse_tools/roadmap.hpp"

namespace
{

using warehouse_tools::Waypoint;
using warehouse_tools::WaypointKind;

// The scenario: two shipping docks and a charger in the south-west, storage
// bays around the perimeter of the open floor, and a corridor skeleton joining
// them. Storage bays sit at the ends of the aisles rather than inside them,
// because a crate deep inside a rack is a manipulation problem and this
// showcase is about allocation.
std::vector<Waypoint> requestedWaypoints()
{
  return {
    {"dock_a", {-5.30, -8.60}, WaypointKind::Dock, 0.0},
    {"dock_b", {-5.30, -6.90}, WaypointKind::Dock, 0.0},
    {"charger", {-5.30, -4.20}, WaypointKind::Charger, 0.0},

    {"bay_w1", {-5.30, -2.60}, WaypointKind::Storage, 0.0},
    {"bay_w2", {-5.30, 2.80}, WaypointKind::Storage, 0.0},
    {"bay_w3", {-5.30, 7.30}, WaypointKind::Storage, 0.0},
    {"bay_n1", {-0.60, 7.20}, WaypointKind::Storage, 0.0},
    {"bay_n2", {1.30, 6.90}, WaypointKind::Storage, 0.0},
    {"bay_e1", {1.30, 1.40}, WaypointKind::Storage, 0.0},
    {"bay_e2", {1.30, -2.60}, WaypointKind::Storage, 0.0},
    {"bay_e3", {1.30, -7.40}, WaypointKind::Storage, 0.0},

    {"c_sw", {-4.20, -8.60}, WaypointKind::Corridor, 0.0},
    {"c_w1", {-4.20, -4.20}, WaypointKind::Corridor, 0.0},
    {"c_w2", {-4.20, -0.60}, WaypointKind::Corridor, 0.0},
    {"c_w3", {-4.20, 3.40}, WaypointKind::Corridor, 0.0},
    {"c_nw", {-4.20, 7.30}, WaypointKind::Corridor, 0.0},
    {"c_n", {-3.00, 8.60}, WaypointKind::Corridor, 0.0},
    {"c_ne", {0.40, 7.00}, WaypointKind::Corridor, 0.0},
    {"c_e1", {0.40, 4.60}, WaypointKind::Corridor, 0.0},
    {"c_e2", {0.40, 0.40}, WaypointKind::Corridor, 0.0},
    {"c_e3", {0.40, -4.20}, WaypointKind::Corridor, 0.0},
    {"c_se", {0.40, -8.60}, WaypointKind::Corridor, 0.0},
    {"c_s", {-2.00, -9.30}, WaypointKind::Corridor, 0.0},
    {"c_mid", {-2.00, -2.20}, WaypointKind::Corridor, 0.0},
  };
}

std::string argument(int argc, char ** argv, const std::string & flag, const std::string & fallback)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) {
      return argv[i + 1];
    }
  }
  return fallback;
}

double numeric(int argc, char ** argv, const std::string & flag, double fallback)
{
  const auto text = argument(argc, argv, flag, "");
  return text.empty() ? fallback : std::stod(text);
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string map_path = argument(argc, argv, "--map", "");
  const std::string output = argument(argc, argv, "--output", "");
  if (map_path.empty() || output.empty()) {
    std::cerr << "usage: roadmap_builder --map <map.yaml> --output <roadmap.yaml>\n"
              << "                       [--radius 0.32] [--speed 0.22]\n"
              << "                       [--turn-penalty 3.0] [--max-edge 6.0]\n";
    return 2;
  }

  // Footprint radius plus margin. The TurtleBot3 Waffle is 0.281 m across its
  // widest point, so 0.32 m is the half-width plus roughly one map cell — tight
  // enough to keep the aisles usable, wide enough that a waypoint never sits
  // where the vehicle would clip a shelf.
  const double radius = numeric(argc, argv, "--radius", 0.32);
  const double speed = numeric(argc, argv, "--speed", 0.22);
  const double turn_penalty = numeric(argc, argv, "--turn-penalty", 3.0);
  const double max_edge = numeric(argc, argv, "--max-edge", 6.0);

  try {
    auto map = warehouse_tools::OccupancyMap::load(map_path);
    map.computeClearance(radius);
    std::cout << "map " << map_path << ": " << map.width() << " x " << map.height()
              << " cells at " << map.resolution() << " m, origin ("
              << map.originX() << ", " << map.originY() << ")\n";

    const auto roadmap = warehouse_tools::Roadmap::build(
      map, requestedWaypoints(), max_edge, speed, turn_penalty);
    roadmap.save(output, map_path);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "wrote " << output << ": " << roadmap.waypoints().size() << " waypoints, "
              << roadmap.edges().size() << " undirected edges, mean degree "
              << roadmap.meanDegree() << "\n";

    // The tightest waypoint is worth naming: it is the one that will fail first
    // if the map, the footprint or the margin ever changes.
    const warehouse_tools::Waypoint * tightest = nullptr;
    for (const auto & wp : roadmap.waypoints()) {
      if (!tightest || wp.clearance_m < tightest->clearance_m) {
        tightest = &wp;
      }
    }
    if (tightest) {
      std::cout << "tightest waypoint: " << tightest->name << " with "
                << tightest->clearance_m << " m of clearance (margin required "
                << radius << " m)\n";
    }

    const auto docks = roadmap.namesOfKind(WaypointKind::Dock);
    const auto bays = roadmap.namesOfKind(WaypointKind::Storage);
    double worst = 0.0;
    std::string worst_pair;
    for (const auto & bay : bays) {
      for (const auto & dock : docks) {
        const double t = roadmap.shortestTime(bay, dock);
        if (t > worst) {
          worst = t;
          worst_pair = bay + " -> " + dock;
        }
      }
    }
    std::cout << "longest bay-to-dock run: " << worst_pair << " at " << worst << " s\n";
  } catch (const std::exception & error) {
    std::cerr << "roadmap_builder: " << error.what() << "\n";
    return 1;
  }
  return 0;
}
