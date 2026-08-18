// Turning a roadmap plus a task specification into a PDDL problem.
//
// The scaling study needs dozens of problems that differ in exactly one
// variable, so they are generated rather than written by hand. Generation also
// removes the most common way a PDDL problem lies: a `connected` relation that
// disagrees with the geometry the executor will actually drive.
//
// The same code serves two consumers. `problem_generator` writes the problem to
// a file for POPF to read directly, and the scenario description it returns is
// what the launch file passes to the mission controller so that the knowledge
// base it builds is the same problem, asserted through PlanSys2's API instead
// of through a file.

#ifndef WAREHOUSE_TOOLS__PROBLEM_WRITER_HPP_
#define WAREHOUSE_TOOLS__PROBLEM_WRITER_HPP_

#include <string>
#include <vector>

#include "warehouse_tools/roadmap.hpp"

namespace warehouse_tools
{

struct Scenario
{
  std::vector<std::string> robots;
  std::vector<std::string> robot_start;    // parallel to robots
  std::vector<std::string> crates;
  std::vector<std::string> crate_bay;      // parallel to crates
  std::vector<std::string> docks;
};

// Robots start on corridor nodes chosen to be as far apart as the roadmap
// allows (greedy farthest-point over graph travel time, not over the coordinate
// frame, so a wall between two nodes counts against them). Crates are dealt
// round-robin over the storage bays, so that adding crates adds distance rather
// than repeating one bay.
Scenario buildScenario(const Roadmap & roadmap, int n_robots, int n_crates);

std::string writeProblem(
  const Roadmap & roadmap,
  const Scenario & scenario,
  const std::string & problem_name = "warehouse_task");

}  // namespace warehouse_tools

#endif  // WAREHOUSE_TOOLS__PROBLEM_WRITER_HPP_
