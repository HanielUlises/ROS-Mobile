// Reading POPF's output back in.
//
// The benchmark runs the same planner PlanSys2 runs, on the same domain, over a
// grid of problems, and has to turn its stdout into numbers. POPF prints a
// solution as lines of
//
//     115.304: (move r1 dock_a c_s)  [18.300]
//
// preceded by search diagnostics and, when it fails, by nothing at all. This
// parser is deliberately strict about that: a run that produced no solution
// line is reported as unsolved rather than as a plan of length zero, because
// the two are the same number and opposite facts.

#ifndef WAREHOUSE_TOOLS__PLAN_PARSER_HPP_
#define WAREHOUSE_TOOLS__PLAN_PARSER_HPP_

#include <map>
#include <string>
#include <vector>

namespace warehouse_tools
{

struct PlanAction
{
  double start_s{0.0};
  double duration_s{0.0};
  std::string name;                    // "move", "pick", "drop"
  std::vector<std::string> arguments;  // ["r1", "dock_a", "c_s"]

  double end_s() const {return start_s + duration_s;}
  std::string text() const;
};

struct Plan
{
  bool solved{false};
  std::vector<PlanAction> actions;

  double makespan() const;
  size_t size() const {return actions.size();}

  // Seconds of driving per robot: the quantity that decides whether adding a
  // vehicle bought anything, since makespan alone hides a fleet that halved the
  // clock by doubling the distance.
  std::map<std::string, double> busyTimePerRobot() const;
  double totalActionTime() const;
  double totalDriveTime() const;

  // The largest number of actions in flight at any instant. One means the
  // planner serialised the fleet; n means it found n things to do at once.
  int peakConcurrency() const;
};

Plan parsePopfOutput(const std::string & text);

}  // namespace warehouse_tools

#endif  // WAREHOUSE_TOOLS__PLAN_PARSER_HPP_
