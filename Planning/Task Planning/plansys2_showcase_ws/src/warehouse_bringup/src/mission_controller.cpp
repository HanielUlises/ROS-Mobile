// The mission controller: everything that happens between "here is a warehouse"
// and "the crates are on the dock".
//
// It does four things, in order, and records all of them:
//
//   1. writes the problem into PlanSys2's knowledge base — instances for the
//      robots, the waypoints and the crates, predicates for where everything
//      is, and the `travel_time` function values read from the same
//      `waypoints.yaml` the roadmap tool produced;
//   2. asks the domain expert and the planner for a plan, timing the call;
//   3. hands the plan to the executor and follows its feedback until every
//      action has finished or one has failed;
//   4. writes a CSV of the plan and a CSV of execution progress, which is what
//      the reports are built from.
//
// Steps 1 and 2 are the classical planning problem; step 3 is where a classical
// plan meets a world that does not stop to be planned about. Keeping the two
// timings separate — `planning_seconds` against `execution_seconds` — is the
// whole point of the instrumentation, because they scale with fleet size in
// opposite directions.

#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <plansys2_domain_expert/DomainExpertClient.hpp>
#include <plansys2_executor/ExecutorClient.hpp>
#include <plansys2_msgs/msg/action_execution_info.hpp>
#include <plansys2_planner/PlannerClient.hpp>
#include <plansys2_problem_expert/ProblemExpertClient.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace warehouse_bringup
{

class MissionController : public rclcpp::Node
{
public:
  MissionController()
  : rclcpp::Node("mission_controller")
  {
    declare_parameter<std::string>("roadmap", "");
    declare_parameter<std::vector<std::string>>("robots", {"r1"});
    // Where each robot starts, in the same order as `robots`.
    declare_parameter<std::vector<std::string>>("robot_start", {"c_e1"});
    declare_parameter<std::vector<std::string>>("crates", {"crate1"});
    // Which bay each crate starts in, in the same order as `crates`.
    declare_parameter<std::vector<std::string>>("crate_bay", {"bay_e1"});
    declare_parameter<std::string>("dock", "dock_a");
    declare_parameter<std::string>("log_dir", "");
    declare_parameter<double>("startup_delay", 5.0);
  }

  void run()
  {
    domain_ = std::make_shared<plansys2::DomainExpertClient>();
    problem_ = std::make_shared<plansys2::ProblemExpertClient>();
    planner_ = std::make_shared<plansys2::PlannerClient>();
    executor_ = std::make_shared<plansys2::ExecutorClient>();

    robots_ = get_parameter("robots").as_string_array();
    starts_ = get_parameter("robot_start").as_string_array();
    crates_ = get_parameter("crates").as_string_array();
    bays_ = get_parameter("crate_bay").as_string_array();
    dock_ = get_parameter("dock").as_string();
    log_dir_ = get_parameter("log_dir").as_string();

    if (robots_.size() != starts_.size() || crates_.size() != bays_.size()) {
      RCLCPP_FATAL(get_logger(), "robots/starts and crates/bays must line up");
      rclcpp::shutdown();
      return;
    }

    // The knowledge base lives in another process that has to have finished
    // its own lifecycle transitions first; asking it to hold instances before
    // then loses them silently.
    rclcpp::sleep_for(std::chrono::milliseconds(
        static_cast<int>(1000 * get_parameter("startup_delay").as_double())));

    if (!populate()) {
      rclcpp::shutdown();
      return;
    }

    const auto goal = buildGoal();
    problem_->setGoal(plansys2::Goal(goal));
    RCLCPP_INFO(get_logger(), "goal: %s", goal.c_str());

    const auto domain = domain_->getDomain();
    const auto problem = problem_->getProblem();

    const auto t0 = std::chrono::steady_clock::now();
    const auto plan = planner_->getPlan(domain, problem);
    const auto t1 = std::chrono::steady_clock::now();
    planning_seconds_ = std::chrono::duration<double>(t1 - t0).count();

    if (!plan.has_value()) {
      RCLCPP_ERROR(get_logger(), "no plan found after %.3f s", planning_seconds_);
      writeSummary(false, 0.0, 0);
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(
      get_logger(), "plan: %zu actions, makespan %.1f s, found in %.3f s",
      plan.value().items.size(), makespan(plan.value()), planning_seconds_);
    writePlan(plan.value());

    if (!executor_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "the executor refused the plan");
      writeSummary(false, 0.0, plan.value().items.size());
      rclcpp::shutdown();
      return;
    }

    execute(plan.value());
    rclcpp::shutdown();
  }

private:
  bool populate()
  {
    const auto roadmap_path = get_parameter("roadmap").as_string();
    YAML::Node roadmap;
    try {
      roadmap = YAML::LoadFile(roadmap_path);
    } catch (const std::exception & error) {
      RCLCPP_FATAL(get_logger(), "cannot read roadmap %s: %s",
        roadmap_path.c_str(), error.what());
      return false;
    }

    for (const auto & entry : roadmap["waypoints"]) {
      problem_->addInstance(
        plansys2::Instance{entry.first.as<std::string>(), "waypoint"});
    }
    for (const auto & r : robots_) {
      problem_->addInstance(plansys2::Instance{r, "robot"});
    }
    for (const auto & c : crates_) {
      problem_->addInstance(plansys2::Instance{c, "crate"});
    }

    for (size_t i = 0; i < robots_.size(); ++i) {
      problem_->addPredicate(plansys2::Predicate("(robot_at " + robots_[i] + " " + starts_[i] + ")"));
      problem_->addPredicate(plansys2::Predicate("(gripper_free " + robots_[i] + ")"));
    }
    for (size_t i = 0; i < crates_.size(); ++i) {
      problem_->addPredicate(plansys2::Predicate("(crate_at " + crates_[i] + " " + bays_[i] + ")"));
    }
    problem_->addPredicate(plansys2::Predicate("(is_dock " + dock_ + ")"));
    // Every waypoint starts clear except the ones the fleet is standing on:
    // single occupancy is a property of the place, so a start position is a
    // place already taken.
    for (const auto & entry : roadmap["waypoints"]) {
      const auto name = entry.first.as<std::string>();
      if (std::find(starts_.begin(), starts_.end(), name) == starts_.end()) {
        problem_->addPredicate(plansys2::Predicate("(wp_clear " + name + ")"));
      }
    }

    // The roadmap is undirected and the domain is directed, so each edge is
    // asserted both ways with the same duration. This is where the metric map
    // enters the symbolic problem, and it is the only place it does.
    size_t edges = 0;
    for (const auto & edge : roadmap["edges"]) {
      const auto a = edge["from"].as<std::string>();
      const auto b = edge["to"].as<std::string>();
      const auto t = edge["travel_time_s"].as<double>();
      problem_->addPredicate(plansys2::Predicate("(connected " + a + " " + b + ")"));
      problem_->addPredicate(plansys2::Predicate("(connected " + b + " " + a + ")"));
      problem_->addFunction(plansys2::Function(
          "(= (travel_time " + a + " " + b + ") " + std::to_string(t) + ")"));
      problem_->addFunction(plansys2::Function(
          "(= (travel_time " + b + " " + a + ") " + std::to_string(t) + ")"));
      ++edges;
    }
    RCLCPP_INFO(
      get_logger(), "knowledge base: %zu robots, %zu crates, %zu edges",
      robots_.size(), crates_.size(), edges);
    return true;
  }

  std::string buildGoal() const
  {
    std::string goal = "(and";
    for (const auto & c : crates_) {
      goal += " (delivered " + c + ")";
    }
    return goal + ")";
  }

  static double makespan(const plansys2_msgs::msg::Plan & plan)
  {
    double end = 0.0;
    for (const auto & item : plan.items) {
      end = std::max(end, static_cast<double>(item.time + item.duration));
    }
    return end;
  }

  void execute(const plansys2_msgs::msg::Plan & plan)
  {
    const auto started = std::chrono::steady_clock::now();
    std::ofstream progress;
    if (!log_dir_.empty()) {
      progress.open(log_dir_ + "/execution.csv", std::ios::out | std::ios::trunc);
      progress << "wall_time_s,completed_actions,total_actions,running\n";
    }

    rclcpp::Rate rate(2.0);
    bool success = false;
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());

      const auto feedback = executor_->getFeedBack();
      size_t done = 0, running = 0;
      for (const auto & action : feedback.action_execution_status) {
        if (action.status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED) {
          ++done;
        } else if (action.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
          ++running;
        }
      }
      const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      if (progress.is_open()) {
        progress << elapsed << ',' << done << ',' << plan.items.size() << ',' << running << '\n';
        progress.flush();
      }

      if (!executor_->execute_and_check_plan()) {
        const auto result = executor_->getResult();
        success = result.has_value() && result.value().success;
        // A failed action is worth naming: with several robots the interesting
        // failures are the ones where two of them wanted the same corridor.
        if (result.has_value()) {
          for (const auto & action : result.value().action_execution_status) {
            if (action.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
              RCLCPP_ERROR(
                get_logger(), "failed: %s (%s)", action.action.c_str(),
                action.message_status.c_str());
            }
          }
        }
        execution_seconds_ = elapsed;
        RCLCPP_INFO(
          get_logger(), "execution %s after %.1f s (%zu/%zu actions)",
          success ? "succeeded" : "failed", elapsed, done, plan.items.size());
        writeSummary(success, elapsed, plan.items.size());
        return;
      }
      rate.sleep();
    }
  }

  void writePlan(const plansys2_msgs::msg::Plan & plan)
  {
    if (log_dir_.empty()) {
      return;
    }
    std::ofstream out(log_dir_ + "/plan.csv", std::ios::out | std::ios::trunc);
    out << "index,start_s,duration_s,action\n";
    for (size_t i = 0; i < plan.items.size(); ++i) {
      const auto & item = plan.items[i];
      out << i << ',' << item.time << ',' << item.duration << ",\"" << item.action << "\"\n";
    }
  }

  void writeSummary(bool success, double execution_seconds, size_t actions)
  {
    if (log_dir_.empty()) {
      return;
    }
    std::ofstream out(log_dir_ + "/summary.csv", std::ios::out | std::ios::trunc);
    out << "robots,crates,actions,planning_seconds,execution_seconds,success\n";
    out << robots_.size() << ',' << crates_.size() << ',' << actions << ','
        << planning_seconds_ << ',' << execution_seconds << ',' << (success ? 1 : 0) << '\n';
  }

  std::shared_ptr<plansys2::DomainExpertClient> domain_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_;
  std::shared_ptr<plansys2::PlannerClient> planner_;
  std::shared_ptr<plansys2::ExecutorClient> executor_;

  std::vector<std::string> robots_, starts_, crates_, bays_;
  std::string dock_;
  std::string log_dir_;
  double planning_seconds_{0.0};
  double execution_seconds_{0.0};
};

}  // namespace warehouse_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<warehouse_bringup::MissionController>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
