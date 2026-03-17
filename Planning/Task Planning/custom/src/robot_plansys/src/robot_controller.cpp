#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

class RobotController : public rclcpp::Node
{
public:
  RobotController()
  : rclcpp::Node("robot_controller"),
    state_(STARTING)
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RobotController::step, this));
  }

private:
  enum State { STARTING, PLANNING, EXECUTING, FINISHED, FAILED };
  State state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void init_problem()
  {
    // Instances
    problem_expert_->addInstance(plansys2::Instance{"robot1", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"warehouse", "location"});
    problem_expert_->addInstance(plansys2::Instance{"office", "location"});
    problem_expert_->addInstance(plansys2::Instance{"kitchen", "location"});
    problem_expert_->addInstance(plansys2::Instance{"package1", "object"});
    problem_expert_->addInstance(plansys2::Instance{"package2", "object"});

    // Initial state predicates
    problem_expert_->addPredicate(plansys2::Predicate{"(robot_at robot1 warehouse)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(robot_free robot1)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(object_at package1 warehouse)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(object_at package2 warehouse)"});

    // Connectivity
    problem_expert_->addPredicate(plansys2::Predicate{"(connected warehouse office)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(connected office warehouse)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(connected office kitchen)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(connected kitchen office)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(connected warehouse kitchen)"});
    problem_expert_->addPredicate(plansys2::Predicate{"(connected kitchen warehouse)"});

    // Goal
    problem_expert_->setGoal(
      plansys2::Goal{
        "(and (object_delivered package1 office) (object_delivered package2 kitchen))"});

    RCLCPP_INFO(get_logger(), "Problem initialized successfully.");
  }

  void step()
  {
    switch (state_) {
      case STARTING:
        {
          RCLCPP_INFO(get_logger(), "[STARTING] Setting up the problem...");
          init_problem();
          state_ = PLANNING;
        }
        break;

      case PLANNING:
        {
          RCLCPP_INFO(get_logger(), "[PLANNING] Requesting plan...");
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            RCLCPP_ERROR(get_logger(), "Could not find plan. Retrying...");
            return;
          }

          RCLCPP_INFO(get_logger(), "[PLANNING] Plan found! Executing...");
          if (!executor_client_->start_plan_execution(plan.value())) {
            RCLCPP_ERROR(get_logger(), "Error starting plan execution!");
            state_ = FAILED;
            return;
          }
          state_ = EXECUTING;
        }
        break;

      case EXECUTING:
        {
          auto feedback = executor_client_->getFeedBack();
          for (const auto & action_feedback : feedback.action_execution_status) {
            // status is int8: NOT_EXECUTED=0, EXECUTING=1, SUCCEEDED=2, FAILED=3, CANCELLED=4
            RCLCPP_INFO(
              get_logger(), "[ACTION] %s - %.0f%% - status:%d",
              action_feedback.action.c_str(),
              action_feedback.completion * 100.0f,
              static_cast<int>(action_feedback.status));
          }

          if (!executor_client_->execute_and_check_plan()) {
            auto result = executor_client_->getResult();
            if (result.has_value()) {
              if (result.value().success) {
                RCLCPP_INFO(get_logger(), "[SUCCESS] All packages delivered!");
                state_ = FINISHED;
              } else {
                RCLCPP_ERROR(get_logger(), "[FAILED] Plan execution failed.");
                state_ = FAILED;
              }
            }
          }
        }
        break;

      case FINISHED:
        RCLCPP_INFO_ONCE(get_logger(), "Mission complete! Robot logistics finished.");
        timer_->cancel();
        break;

      case FAILED:
        RCLCPP_ERROR_ONCE(get_logger(), "Mission failed.");
        timer_->cancel();
        break;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
