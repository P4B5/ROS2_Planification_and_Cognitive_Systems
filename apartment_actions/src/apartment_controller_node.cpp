// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ApartmentController : public rclcpp::Node
{
public:
  ApartmentController()
  : rclcpp::Node("apartment_controller"), state_(STARTING)
  {
  }

  void init()
  {
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(shared_from_this());
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(shared_from_this());
    init_knowledge();
  }

  void init_knowledge()
  {

    problem_expert_->addInstance(plansys2::Instance{"wp_control", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp4", "waypoint"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp1 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp3 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp_control)"));

    problem_expert_->addInstance(plansys2::Instance{"kitchen room"});
    problem_expert_->addInstance(plansys2::Instance{"living_room room"});
    problem_expert_->addInstance(plansys2::Instance{"main_bedroom room"});
    problem_expert_->addInstance(plansys2::Instance{"big_bathroom room"});
    problem_expert_->addInstance(plansys2::Instance{"small_bathroom room"});
    problem_expert_->addInstance(plansys2::Instance{"computer_bedroom room"});
    problem_expert_->addInstance(plansys2::Instance{"downstairs room"});
    problem_expert_->addInstance(plansys2::Instance{"corridor corridor"});
    problem_expert_->addInstance(plansys2::Instance{"tv_zone zone"});
    problem_expert_->addInstance(plansys2::Instance{"dining_zone zone"});
    problem_expert_->addInstance(plansys2::Instance{"fridge_zone zone"});
    problem_expert_->addInstance(plansys2::Instance{"dishwasher_zone zone"});
    problem_expert_->addInstance(plansys2::Instance{"computer_zone zone"});
    problem_expert_->addInstance(plansys2::Instance{"bathtub_zone zone"});
    problem_expert_->addInstance(plansys2::Instance{"laptop object"});
    problem_expert_->addInstance(plansys2::Instance{"paper_boat object"});
    problem_expert_->addInstance(plansys2::Instance{"biscuits object"});
    problem_expert_->addInstance(plansys2::Instance{"gray_pillow object"});
    problem_expert_->addInstance(plansys2::Instance{"rubber_duck object"});
    problem_expert_->addInstance(plansys2::Instance{"computer object"});
    problem_expert_->addInstance(plansys2::Instance{"tiago robot"});

    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether kitchen living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether living_room kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether living_room corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether corridor living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether corridor downstairs)"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether downstairs corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether corridor main_bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether main_bedroom corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether corridor big_bathroom"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether big_bathroom corridor"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether corridor computer_bedroom"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether computer_bedroom corridor"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether corridor small_bathroom"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether small_bathroom corridor"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether kitchen fridge_zone"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether fridge_zone kitchen"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether kitchen dishwasher_zone"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether dishwasher_zone kitchen"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether living_room tv_zone"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether tv_zone living_room"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether living_room dining_zone"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether dining_zone living_room"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether computer_bedroom computer_zone"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether computer_zone computer_bedroom"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether big_bathroom bathtub_zone"));
    problem_expert_->addPredicate(plansys2::Predicate("(placesTogether bathtub_zone big_bathroom"));
    problem_expert_->addPredicate(plansys2::Predicate("(robotAt tiago living_room"));
    problem_expert_->addPredicate(plansys2::Predicate("(robotIdle tiago"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectAt laptop tv_zone"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectAt paper_boat living_room"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectAt biscuits dishwasher_zone"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectAt gray_pillow main_bedroom"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectAt rubber_duck bathtub_zone"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectAt computer computer_zone"));

  }

  void step()
  {
    switch (state_) {
      case STARTING:
        // Set the goal for next state, and execute plan
        problem_expert_->setGoal(plansys2::Goal("(and(apartment wp1))"));

        if (executor_client_->start_plan_execution()) {
          state_ = APARTMENT_WP1;
        }
        break;
      case APARTMENT_WP1:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(apartment wp1)"));

              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(apartment wp2))"));

              if (executor_client_->start_plan_execution()) {
                state_ = APARTMENT_WP2;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }
              executor_client_->start_plan_execution();  // replan and execute
            }
          }
        }
        break;
      case APARTMENT_WP2:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(apartment wp2)"));

              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(apartment wp3))"));

              if (executor_client_->start_plan_execution()) {
                state_ = APARTMENT_WP3;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }
              executor_client_->start_plan_execution();  // replan and execute
            }
          }
        }
        break;
      case APARTMENT_WP3:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(apartment wp3)"));

              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(apartment wp4))"));

              if (executor_client_->start_plan_execution()) {
                state_ = APARTMENT_WP4;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }
              executor_client_->start_plan_execution();  // replan and execute
            }
          }
        }
        break;
      case APARTMENT_WP4:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(apartment wp4)"));

              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(apartment wp1))"));

              if (executor_client_->start_plan_execution()) {
                // Loop to WP1
                state_ = APARTMENT_WP1;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }
              executor_client_->start_plan_execution();  // replan and execute
            }
          }
        }
        break;
      default:
        break;
    }
  }

private:
  typedef enum {STARTING, APARTMENT_WP1, APARTMENT_WP2, APARTMENT_WP3, APARTMENT_WP4} StateType;
  StateType state_;

  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApartmentController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
