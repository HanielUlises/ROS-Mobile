#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"

using namespace std::chrono_literals;

class PatrolMissionController : public rclcpp::Node
{
public:
  PatrolMissionController()
  : rclcpp::Node("planner_client"), state_(INIT), knowledge_loaded_(false)
  {
    domain_expert_   = std::make_shared<plansys2::DomainExpertClient>();
    problem_expert_  = std::make_shared<plansys2::ProblemExpertClient>();
    planner_client_  = std::make_shared<plansys2::PlannerClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    timer_ = create_wall_timer(500ms, std::bind(&PatrolMissionController::step, this));
  }

private:
  enum State { INIT, PLAN, EXECUTE, FINISHED };

  void step()
  {
    switch (state_) {
      case INIT:
        if (!knowledge_loaded_) {
          init_knowledge();
          knowledge_loaded_ = true;
        }
        state_ = PLAN;
        break;
      case PLAN:
        if (plan_and_execute()) {
          state_ = EXECUTE;
        } else {
          RCLCPP_WARN(get_logger(), "Planning failed, will retry...");
          rclcpp::sleep_for(2s);
        }
        break;
      case EXECUTE:
        monitor_execution();
        break;
      case FINISHED:
        timer_->cancel();
        RCLCPP_INFO(get_logger(), "Patrol mission complete.");
        break;
    }
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"tb3", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"wp0", "location"});
    problem_expert_->addInstance(plansys2::Instance{"wp1", "location"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "location"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "location"});

    problem_expert_->addPredicate(plansys2::Predicate{"(at tb3 wp0)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(path wp0 wp1)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(path wp1 wp0)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(path wp1 wp2)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(path wp2 wp1)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(path wp2 wp3)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(path wp3 wp2)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(path wp3 wp0)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(path wp0 wp3)"});

    problem_expert_->setGoal(plansys2::Goal{
      "(and (inspected wp1) (inspected wp2) (inspected wp3)"
      " (reported wp1) (reported wp2) (reported wp3)"
      " (at tb3 wp0))"
    });

    RCLCPP_INFO(get_logger(), "Knowledge base initialised.");
  }

  bool plan_and_execute()
  {
    auto domain  = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan    = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      RCLCPP_ERROR(get_logger(), "No plan found.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Plan found: %zu actions.", plan.value().items.size());

    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Executor rejected plan.");
      return false;
    }

    return true;
  }

  void monitor_execution()
  {
    auto feedback = executor_client_->getFeedBack();
    for (const auto & af : feedback.action_execution_status) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "  %-25s  %5.1f%%", af.action.c_str(), af.completion * 100.0);
    }

    if (!executor_client_->execute_and_check_plan()) {
      auto result = executor_client_->getResult();
      if (result.has_value()) {
        if (result.value().success) {
          RCLCPP_INFO(get_logger(), "Mission succeeded.");
        } else {
          RCLCPP_ERROR(get_logger(), "Mission failed.");
        }
      }
      state_ = FINISHED;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<plansys2::DomainExpertClient>  domain_expert_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::PlannerClient>       planner_client_;
  std::shared_ptr<plansys2::ExecutorClient>      executor_client_;
  State state_;
  bool knowledge_loaded_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolMissionController>());
  rclcpp::shutdown();
  return 0;
}