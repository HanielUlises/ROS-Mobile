// Write one PDDL problem, and print the scenario it encodes.
//
// Two consumers, one definition. POPF reads the problem file directly, which is
// how the benchmark measures planning; the launch file reads the printed
// scenario and passes it to the mission controller, which asserts the same
// facts through PlanSys2's problem-expert API. Generating both from one
// function is what keeps the benchmark measuring the problem the fleet actually
// runs.
//
//   ros2 run warehouse_tools problem_generator --roadmap <roadmap.yaml>
//       --robots 3 --crates 6 --output /tmp/problem.pddl

#include <fstream>
#include <iostream>
#include <string>

#include "warehouse_tools/problem_writer.hpp"
#include "warehouse_tools/roadmap.hpp"

namespace
{

std::string argument(int argc, char ** argv, const std::string & flag, const std::string & fallback)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) {
      return argv[i + 1];
    }
  }
  return fallback;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string roadmap_path = argument(argc, argv, "--roadmap", "");
  const std::string output = argument(argc, argv, "--output", "");
  const int robots = std::stoi(argument(argc, argv, "--robots", "1"));
  const int crates = std::stoi(argument(argc, argv, "--crates", "4"));
  const std::string name = argument(argc, argv, "--name", "warehouse_task");

  if (roadmap_path.empty()) {
    std::cerr << "usage: problem_generator --roadmap <roadmap.yaml> [--robots N] "
              << "[--crates K] [--output <problem.pddl>]\n";
    return 2;
  }

  try {
    const auto roadmap = warehouse_tools::Roadmap::load(roadmap_path);
    const auto scenario = warehouse_tools::buildScenario(roadmap, robots, crates);
    const auto problem = warehouse_tools::writeProblem(roadmap, scenario, name);

    if (output.empty()) {
      std::cout << problem;
    } else {
      std::ofstream out(output);
      if (!out) {
        std::cerr << "cannot write " << output << "\n";
        return 1;
      }
      out << problem;
      std::cerr << "wrote " << output << ": " << robots << " robot(s), "
                << crates << " crate(s)\n";
    }

    // Printed on stderr as YAML so that a launch file can capture it while the
    // problem itself goes to stdout or to a file.
    std::cerr << "scenario:\n  robots: [";
    for (size_t i = 0; i < scenario.robots.size(); ++i) {
      std::cerr << (i ? ", " : "") << scenario.robots[i];
    }
    std::cerr << "]\n  robot_start: [";
    for (size_t i = 0; i < scenario.robot_start.size(); ++i) {
      std::cerr << (i ? ", " : "") << scenario.robot_start[i];
    }
    std::cerr << "]\n  crate_bay: [";
    for (size_t i = 0; i < scenario.crate_bay.size(); ++i) {
      std::cerr << (i ? ", " : "") << scenario.crate_bay[i];
    }
    std::cerr << "]\n  dock: " << (scenario.docks.empty() ? "none" : scenario.docks.front())
              << "\n";
  } catch (const std::exception & error) {
    std::cerr << "problem_generator: " << error.what() << "\n";
    return 1;
  }
  return 0;
}
