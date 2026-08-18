// Sweep the fleet size and the workload, and record what the planner does.
//
// The embodied runs in Gazebo answer "does this work?" for a handful of
// configurations; they are too slow to answer "what happens as the fleet
// grows?" This tool answers the second question on the same domain, the same
// roadmap and the same planner PlanSys2 uses, by running POPF directly over a
// grid of generated problems.
//
// What it records per cell of the grid:
//
//   * planning wall time, which is the cost of deliberation and grows with the
//     number of ground actions, i.e. with robots x waypoints;
//   * makespan, the plan's own predicted completion time in seconds;
//   * plan length, total action time and total drive time, which separate "the
//     fleet finished sooner" from "the fleet drove less";
//   * peak concurrency and per-robot busy time, which say whether an added
//     robot was actually used or merely stood in the aisle.
//
// A cell that times out is written down as a timeout rather than dropped: where
// the planner stops being able to answer is a result about classical planning,
// and hiding it would make the curves look better than the method is.
//
//   ros2 run warehouse_tools plan_benchmark --roadmap <roadmap.yaml>
//       --domain <warehouse_domain.pddl> --robots 1,2,3,4 --crates 2,4,6,8
//       --output docs/data/planner_scaling.csv

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

#include "warehouse_tools/plan_parser.hpp"
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

std::vector<int> intList(const std::string & text)
{
  std::vector<int> values;
  std::istringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (!token.empty()) {
      values.push_back(std::stoi(token));
    }
  }
  return values;
}

struct RunResult
{
  bool timed_out{false};
  double seconds{0.0};
  std::string output;
};

// Runs the planner as a child process with a wall-clock limit. `popen` would be
// shorter, but it gives no way to kill a planner that has stopped making
// progress, and an unbounded POPF on a large instance will happily sit there
// for hours — which is exactly the case this benchmark needs to record and move
// past.
RunResult runPlanner(
  const std::string & planner,
  const std::string & domain,
  const std::string & problem,
  double timeout_s)
{
  RunResult result;
  std::array<int, 2> pipe_fd{};
  if (pipe(pipe_fd.data()) != 0) {
    throw std::runtime_error("cannot create a pipe for the planner");
  }

  const auto started = std::chrono::steady_clock::now();
  const pid_t pid = fork();
  if (pid < 0) {
    throw std::runtime_error("cannot fork the planner");
  }
  if (pid == 0) {
    close(pipe_fd[0]);
    dup2(pipe_fd[1], STDOUT_FILENO);
    dup2(pipe_fd[1], STDERR_FILENO);
    close(pipe_fd[1]);
    execl(planner.c_str(), planner.c_str(), domain.c_str(), problem.c_str(),
      static_cast<char *>(nullptr));
    _exit(127);
  }

  close(pipe_fd[1]);
  // Read while the child runs, so that a planner that fills the pipe buffer
  // does not deadlock against a parent that is only waiting for it to exit.
  std::string output;
  std::array<char, 4096> buffer{};
  int status = 0;
  bool exited = false;
  while (true) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(pipe_fd[0], &set);
    timeval tv{0, 200000};
    if (select(pipe_fd[0] + 1, &set, nullptr, nullptr, &tv) > 0) {
      const ssize_t got = read(pipe_fd[0], buffer.data(), buffer.size());
      if (got > 0) {
        output.append(buffer.data(), static_cast<size_t>(got));
        continue;
      }
      if (got == 0) {
        break;                      // planner closed its stdout
      }
    }
    if (waitpid(pid, &status, WNOHANG) == pid) {
      exited = true;
      break;
    }
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    if (elapsed > timeout_s) {
      kill(pid, SIGKILL);
      waitpid(pid, &status, 0);
      result.timed_out = true;
      exited = true;
      break;
    }
  }
  // Drain whatever is left in the pipe before closing it, or a plan printed
  // just before exit is lost and the run is misreported as unsolved.
  while (true) {
    const ssize_t got = read(pipe_fd[0], buffer.data(), buffer.size());
    if (got <= 0) {
      break;
    }
    output.append(buffer.data(), static_cast<size_t>(got));
  }
  close(pipe_fd[0]);
  if (!exited) {
    waitpid(pid, &status, 0);
  }

  result.seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  result.output = std::move(output);
  return result;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string roadmap_path = argument(argc, argv, "--roadmap", "");
  const std::string domain = argument(argc, argv, "--domain", "");
  const std::string output_path = argument(argc, argv, "--output", "");
  const std::string planner = argument(argc, argv, "--planner", "/opt/ros/humble/lib/popf/popf");
  const std::string workdir = argument(argc, argv, "--workdir", "/tmp");
  const auto robot_counts = intList(argument(argc, argv, "--robots", "1,2,3,4"));
  const auto crate_counts = intList(argument(argc, argv, "--crates", "2,4,6,8"));
  const double timeout_s = std::stod(argument(argc, argv, "--timeout", "120"));

  if (roadmap_path.empty() || domain.empty() || output_path.empty()) {
    std::cerr << "usage: plan_benchmark --roadmap <roadmap.yaml> --domain <domain.pddl> "
              << "--output <csv> [--robots 1,2,3,4] [--crates 2,4,6,8] [--timeout 120]\n";
    return 2;
  }

  try {
    const auto roadmap = warehouse_tools::Roadmap::load(roadmap_path);
    std::ofstream csv(output_path);
    if (!csv) {
      std::cerr << "cannot write " << output_path << "\n";
      return 1;
    }
    csv << "robots,crates,solved,timed_out,planning_seconds,actions,makespan_s,"
        << "action_time_s,drive_time_s,peak_concurrency,min_robot_busy_s,max_robot_busy_s\n";

    for (const int crates : crate_counts) {
      for (const int robots : robot_counts) {
        const auto scenario = warehouse_tools::buildScenario(roadmap, robots, crates);
        const auto problem_text = warehouse_tools::writeProblem(roadmap, scenario);
        const std::string problem_path =
          workdir + "/bench_r" + std::to_string(robots) + "_c" + std::to_string(crates) + ".pddl";
        {
          std::ofstream problem_file(problem_path);
          problem_file << problem_text;
        }

        const auto run = runPlanner(planner, domain, problem_path, timeout_s);
        const auto plan = warehouse_tools::parsePopfOutput(run.output);

        double min_busy = 0.0, max_busy = 0.0;
        if (plan.solved) {
          const auto busy = plan.busyTimePerRobot();
          bool first = true;
          for (const auto & robot : scenario.robots) {
            const double value = busy.count(robot) ? busy.at(robot) : 0.0;
            min_busy = first ? value : std::min(min_busy, value);
            max_busy = first ? value : std::max(max_busy, value);
            first = false;
          }
        }

        csv << robots << ',' << crates << ',' << (plan.solved ? 1 : 0) << ','
            << (run.timed_out ? 1 : 0) << ',' << run.seconds << ',' << plan.size() << ','
            << plan.makespan() << ',' << plan.totalActionTime() << ','
            << plan.totalDriveTime() << ',' << plan.peakConcurrency() << ','
            << min_busy << ',' << max_busy << '\n';
        csv.flush();

        std::cout << robots << " robot(s), " << crates << " crate(s): ";
        if (run.timed_out) {
          std::cout << "TIMEOUT after " << timeout_s << " s\n";
        } else if (!plan.solved) {
          std::cout << "no plan (" << run.seconds << " s)\n";
        } else {
          std::cout << plan.size() << " actions, makespan " << plan.makespan()
                    << " s, planned in " << run.seconds << " s, peak concurrency "
                    << plan.peakConcurrency() << "\n";
        }
      }
    }
    std::cout << "wrote " << output_path << "\n";
  } catch (const std::exception & error) {
    std::cerr << "plan_benchmark: " << error.what() << "\n";
    return 1;
  }
  return 0;
}
