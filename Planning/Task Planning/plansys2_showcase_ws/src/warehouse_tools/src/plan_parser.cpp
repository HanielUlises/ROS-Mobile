#include "warehouse_tools/plan_parser.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace warehouse_tools
{

std::string PlanAction::text() const
{
  std::string out = "(" + name;
  for (const auto & argument : arguments) {
    out += " " + argument;
  }
  return out + ")";
}

double Plan::makespan() const
{
  double end = 0.0;
  for (const auto & action : actions) {
    end = std::max(end, action.end_s());
  }
  return end;
}

std::map<std::string, double> Plan::busyTimePerRobot() const
{
  std::map<std::string, double> busy;
  for (const auto & action : actions) {
    if (!action.arguments.empty()) {
      busy[action.arguments.front()] += action.duration_s;
    }
  }
  return busy;
}

double Plan::totalActionTime() const
{
  double total = 0.0;
  for (const auto & action : actions) {
    total += action.duration_s;
  }
  return total;
}

double Plan::totalDriveTime() const
{
  double total = 0.0;
  for (const auto & action : actions) {
    if (action.name == "move") {
      total += action.duration_s;
    }
  }
  return total;
}

int Plan::peakConcurrency() const
{
  // Sweep the action intervals as +1/-1 events. Ends are processed before
  // starts at equal times, so an action that begins exactly when another ends
  // is not counted as concurrent with it — POPF's own epsilon separation makes
  // exact ties rare, but the convention should be stated rather than discovered.
  std::vector<std::pair<double, int>> events;
  events.reserve(actions.size() * 2);
  for (const auto & action : actions) {
    events.emplace_back(action.start_s, +1);
    events.emplace_back(action.end_s(), -1);
  }
  std::sort(
    events.begin(), events.end(),
    [](const auto & a, const auto & b) {
      if (a.first == b.first) {
        return a.second < b.second;
      }
      return a.first < b.first;
    });

  int current = 0, peak = 0;
  for (const auto & [time, delta] : events) {
    (void)time;
    current += delta;
    peak = std::max(peak, current);
  }
  return peak;
}

Plan parsePopfOutput(const std::string & text)
{
  Plan plan;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    // POPF prints its search progress and the solution on the same stream, and
    // the progress lines start with 'b (' rather than a number, so the shape of
    // the line is what separates them.
    const auto colon = line.find(':');
    const auto open = line.find('(');
    const auto close = line.find(')', open == std::string::npos ? 0 : open);
    if (colon == std::string::npos || open == std::string::npos ||
      close == std::string::npos || open < colon)
    {
      continue;
    }

    double start = 0.0;
    try {
      size_t consumed = 0;
      start = std::stod(line.substr(0, colon), &consumed);
      if (consumed != colon) {
        continue;               // the text before ':' was not purely a number
      }
    } catch (const std::exception &) {
      continue;
    }

    PlanAction action;
    action.start_s = start;

    std::istringstream body(line.substr(open + 1, close - open - 1));
    std::string token;
    if (!(body >> action.name)) {
      continue;
    }
    while (body >> token) {
      action.arguments.push_back(token);
    }

    const auto bracket = line.find('[', close);
    const auto bracket_end = line.find(']', bracket == std::string::npos ? 0 : bracket);
    if (bracket != std::string::npos && bracket_end != std::string::npos) {
      try {
        action.duration_s = std::stod(line.substr(bracket + 1, bracket_end - bracket - 1));
      } catch (const std::exception &) {
        action.duration_s = 0.0;
      }
    }

    plan.actions.push_back(action);
  }

  plan.solved = !plan.actions.empty();
  std::sort(
    plan.actions.begin(), plan.actions.end(),
    [](const PlanAction & a, const PlanAction & b) {return a.start_s < b.start_s;});
  return plan;
}

}  // namespace warehouse_tools
