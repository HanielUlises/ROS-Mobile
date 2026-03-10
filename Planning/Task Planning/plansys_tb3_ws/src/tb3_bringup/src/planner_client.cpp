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

class PlannerClientNode : public rclcpp::Node
{
public:
  PlannerClientNode()
  : rclcpp::Node("planner_client")
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    timer_ = create_wall_timer(
      500ms, std::bind(&PlannerClientNode::step, this));

    state_ = INIT;
  }

private:
  enum State { INIT, PLAN, EXECUTE, FINISHED };

  void step()
  {
    switch (state_) {
      case INIT:
        init_knowledge();
        state_ = PLAN;
        break;

      case PLAN:
        if (!plan_and_execute()) {
          state_ = INIT;
        } else {
          state_ = EXECUTE;
        }
        break;

      case EXECUTE:
        monitor_execution();
        break;

      case FINISHED:
        timer_->cancel();
        RCLCPP_INFO(get_logger(), "Planning and execution complete.");
        break;
    }
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"tb3", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"wp1", "location"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "location"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "location"});

    problem_expert_->addPredicate(plansys2::Predicate{"(at tb3 wp1)"});

    problem_expert_->setGoal(plansys2::Goal{"(and (at tb3 wp3))"});

    RCLCPP_INFO(get_logger(), "Knowledge base initialized.");
  }

  bool plan_and_execute()
  {
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      RCLCPP_ERROR(get_logger(), "Could not compute plan. Retrying...");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Plan found with %zu actions.", plan.value().items.size());

    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Could not start plan execution.");
      return false;
    }

    return true;
  }

  void monitor_execution()
  {
    auto feedback = executor_client_->getFeedBack();

    for (const auto & action_feedback : feedback.action_execution_status) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[%s] completion: %.1f%%",
        action_feedback.action.c_str(),
        action_feedback.completion * 100.0);
    }

    if (!executor_client_->execute_and_check_plan()) {
      auto result = executor_client_->getResult();
      if (result.has_value()) {
        if (result.value().success) {
          RCLCPP_INFO(get_logger(), "Plan execution succeeded!");
        } else {
          RCLCPP_ERROR(get_logger(), "Plan execution failed.");
        }
      }
      state_ = FINISHED;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  State state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
