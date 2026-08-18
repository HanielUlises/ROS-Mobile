#include "warehouse_tools/roadmap.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace warehouse_tools
{

std::string toString(WaypointKind kind)
{
  switch (kind) {
    case WaypointKind::Storage: return "storage";
    case WaypointKind::Dock: return "dock";
    case WaypointKind::Charger: return "charger";
    case WaypointKind::Corridor: default: return "corridor";
  }
}

WaypointKind kindFromString(const std::string & text)
{
  if (text == "storage") {return WaypointKind::Storage;}
  if (text == "dock") {return WaypointKind::Dock;}
  if (text == "charger") {return WaypointKind::Charger;}
  if (text == "corridor") {return WaypointKind::Corridor;}
  throw std::runtime_error("unknown waypoint kind: " + text);
}

Roadmap Roadmap::build(
  const OccupancyMap & map,
  const std::vector<Waypoint> & requested,
  double max_edge_m,
  double speed_mps,
  double turn_penalty_s)
{
  Roadmap roadmap;
  roadmap.meta_.speed_mps = speed_mps;
  roadmap.meta_.turn_penalty_s = turn_penalty_s;
  roadmap.meta_.radius_m = map.radius();
  roadmap.meta_.max_edge_m = max_edge_m;

  std::vector<std::string> rejected;
  for (const auto & candidate : requested) {
    if (!map.isClear(candidate.position)) {
      rejected.push_back(candidate.name);
      continue;
    }
    Waypoint accepted = candidate;
    accepted.clearance_m = map.clearanceAt(candidate.position);
    roadmap.waypoints_.push_back(accepted);
  }
  if (!rejected.empty()) {
    std::string message = "waypoints without clearance on the map:";
    for (const auto & name : rejected) {
      message += " " + name;
    }
    throw std::runtime_error(message);
  }

  for (size_t i = 0; i < roadmap.waypoints_.size(); ++i) {
    for (size_t j = i + 1; j < roadmap.waypoints_.size(); ++j) {
      const auto & a = roadmap.waypoints_[i];
      const auto & b = roadmap.waypoints_[j];
      const double length = std::hypot(b.position.x - a.position.x, b.position.y - a.position.y);
      if (length > max_edge_m) {
        continue;
      }
      if (!map.isSegmentClear(a.position, b.position)) {
        continue;
      }
      // The turn penalty is charged per edge rather than per turn angle: the
      // planner has no heading state, so a per-angle cost would be a fiction
      // dressed as precision. A flat cost keeps the durations honest — they
      // over-estimate a straight-through transit and under-estimate a reversal.
      roadmap.edges_.push_back(
        Edge{a.name, b.name, length, length / speed_mps + turn_penalty_s});
    }
  }

  std::vector<std::string> unreachable;
  if (!roadmap.isConnected(&unreachable)) {
    std::string message = "roadmap is disconnected; unreachable from the first waypoint:";
    for (const auto & name : unreachable) {
      message += " " + name;
    }
    throw std::runtime_error(message);
  }
  return roadmap;
}

bool Roadmap::isConnected(std::vector<std::string> * unreachable) const
{
  if (waypoints_.empty()) {
    return true;
  }
  std::map<std::string, std::vector<std::string>> adjacency;
  for (const auto & wp : waypoints_) {
    adjacency[wp.name];
  }
  for (const auto & edge : edges_) {
    adjacency[edge.from].push_back(edge.to);
    adjacency[edge.to].push_back(edge.from);
  }

  std::set<std::string> seen{waypoints_.front().name};
  std::vector<std::string> stack{waypoints_.front().name};
  while (!stack.empty()) {
    const auto current = stack.back();
    stack.pop_back();
    for (const auto & next : adjacency[current]) {
      if (seen.insert(next).second) {
        stack.push_back(next);
      }
    }
  }
  if (seen.size() == waypoints_.size()) {
    return true;
  }
  if (unreachable) {
    for (const auto & wp : waypoints_) {
      if (!seen.count(wp.name)) {
        unreachable->push_back(wp.name);
      }
    }
  }
  return false;
}

std::vector<std::string> Roadmap::namesOfKind(WaypointKind kind) const
{
  std::vector<std::string> names;
  for (const auto & wp : waypoints_) {
    if (wp.kind == kind) {
      names.push_back(wp.name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

const Waypoint * Roadmap::find(const std::string & name) const
{
  for (const auto & wp : waypoints_) {
    if (wp.name == name) {
      return &wp;
    }
  }
  return nullptr;
}

double Roadmap::meanDegree() const
{
  if (waypoints_.empty()) {
    return 0.0;
  }
  return 2.0 * static_cast<double>(edges_.size()) / static_cast<double>(waypoints_.size());
}

double Roadmap::shortestTime(const std::string & from, const std::string & to) const
{
  const double infinity = std::numeric_limits<double>::infinity();
  std::map<std::string, std::vector<std::pair<std::string, double>>> adjacency;
  for (const auto & edge : edges_) {
    adjacency[edge.from].emplace_back(edge.to, edge.travel_time_s);
    adjacency[edge.to].emplace_back(edge.from, edge.travel_time_s);
  }

  std::map<std::string, double> cost;
  for (const auto & wp : waypoints_) {
    cost[wp.name] = infinity;
  }
  if (!cost.count(from) || !cost.count(to)) {
    return infinity;
  }

  using Item = std::pair<double, std::string>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  cost[from] = 0.0;
  open.emplace(0.0, from);
  while (!open.empty()) {
    const auto [c, node] = open.top();
    open.pop();
    if (c > cost[node]) {
      continue;
    }
    if (node == to) {
      return c;
    }
    for (const auto & [next, weight] : adjacency[node]) {
      if (c + weight < cost[next]) {
        cost[next] = c + weight;
        open.emplace(cost[next], next);
      }
    }
  }
  return cost[to];
}

void Roadmap::save(const std::string & yaml_path, const std::string & map_path) const
{
  std::ofstream out(yaml_path);
  if (!out) {
    throw std::runtime_error("cannot write " + yaml_path);
  }
  out << "# GENERATED by warehouse_tools/roadmap_builder - do not edit by hand.\n"
      << "#\n"
      << "# Every waypoint below fits the vehicle's footprint on the map named in\n"
      << "# `meta.map`, and every edge is a straight segment that is free at the same\n"
      << "# clearance. `travel_time_s` is the segment length over the cruise speed plus\n"
      << "# a flat turn penalty, which is what makes the planner's makespan a number of\n"
      << "# seconds rather than a number of actions.\n"
      << "meta:\n"
      << "  map: " << map_path << "\n"
      << "  radius_m: " << meta_.radius_m << "\n"
      << "  speed_mps: " << meta_.speed_mps << "\n"
      << "  turn_penalty_s: " << meta_.turn_penalty_s << "\n"
      << "  max_edge_m: " << meta_.max_edge_m << "\n"
      << "waypoints:\n";
  out.setf(std::ios::fixed);
  out.precision(3);
  for (const auto & wp : waypoints_) {
    out << "  " << wp.name << ": {x: " << wp.position.x << ", y: " << wp.position.y
        << ", kind: " << toString(wp.kind) << ", clearance_m: " << wp.clearance_m << "}\n";
  }
  out << "edges:\n";
  for (const auto & edge : edges_) {
    out << "  - {from: " << edge.from << ", to: " << edge.to
        << ", length_m: " << edge.length_m
        << ", travel_time_s: " << edge.travel_time_s << "}\n";
  }
}

Roadmap Roadmap::load(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception & error) {
    throw std::runtime_error("cannot read roadmap " + yaml_path + ": " + error.what());
  }

  Roadmap roadmap;
  if (root["meta"]) {
    const auto & meta = root["meta"];
    if (meta["speed_mps"]) {roadmap.meta_.speed_mps = meta["speed_mps"].as<double>();}
    if (meta["turn_penalty_s"]) {
      roadmap.meta_.turn_penalty_s = meta["turn_penalty_s"].as<double>();
    }
    if (meta["radius_m"]) {roadmap.meta_.radius_m = meta["radius_m"].as<double>();}
    if (meta["max_edge_m"]) {roadmap.meta_.max_edge_m = meta["max_edge_m"].as<double>();}
    if (meta["map"]) {roadmap.meta_.map_path = meta["map"].as<std::string>();}
  }
  for (const auto & entry : root["waypoints"]) {
    Waypoint wp;
    wp.name = entry.first.as<std::string>();
    wp.position.x = entry.second["x"].as<double>();
    wp.position.y = entry.second["y"].as<double>();
    wp.kind = kindFromString(entry.second["kind"].as<std::string>());
    if (entry.second["clearance_m"]) {
      wp.clearance_m = entry.second["clearance_m"].as<double>();
    }
    roadmap.waypoints_.push_back(wp);
  }
  for (const auto & entry : root["edges"]) {
    roadmap.edges_.push_back(
      Edge{
        entry["from"].as<std::string>(),
        entry["to"].as<std::string>(),
        entry["length_m"].as<double>(),
        entry["travel_time_s"].as<double>()});
  }
  return roadmap;
}

}  // namespace warehouse_tools
