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
  : rclcpp::Node("apartment_controller")
  {
  }

  void init()
  {
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(shared_from_this());
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(shared_from_this());

    init_knowledge();

    if (!executor_client_->start_plan_execution()) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"kitchen", "room"});
    problem_expert_->addInstance(plansys2::Instance{"living_room", "room"});
    problem_expert_->addInstance(plansys2::Instance{"main_bedroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"big_bathroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"small_bathroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"computer_bedroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"downstairs", "room"});
    problem_expert_->addInstance(plansys2::Instance{"corridor", "corridor"});
    problem_expert_->addInstance(plansys2::Instance{"tv_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"dining_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"fridge_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"dishwasher_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"computer_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"bathtub_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"laptop", "object"});
    problem_expert_->addInstance(plansys2::Instance{"paper_boat", "object"});
    problem_expert_->addInstance(plansys2::Instance{"biscuits", "object"});
    problem_expert_->addInstance(plansys2::Instance{"gray_pillow", "object"});
    problem_expert_->addInstance(plansys2::Instance{"rubber_duck", "object"});
    problem_expert_->addInstance(plansys2::Instance{"computer", "object"});
    problem_expert_->addInstance(plansys2::Instance{"tiago", "robot"});

    problem_expert_->addPredicate(plansys2::Predicate("(places_together kitchen living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together living_room kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together living_room corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together corridor living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together corridor downstairs)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together downstairs corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together corridor main_bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together main_bedroom corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together corridor big_bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together big_bathroom corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together corridor computer_bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together computer_bedroom corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together corridor small_bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together small_bathroom corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together kitchen fridge_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together fridge_zone kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together kitchen dishwasher_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together dishwasher_zone kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together living_room tv_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together tv_zone living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together living_room dining_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together dining_zone living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together computer_bedroom computer_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together computer_zone computer_bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together big_bathroom bathtub_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(places_together bathtub_zone big_bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at tiago living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_idle tiago)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at laptop tv_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at paper_boat living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at biscuits dishwasher_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at gray_pillow main_bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at rubber_duck bathtub_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at computer computer_zone)"));

    problem_expert_->setGoal(
      plansys2::Goal(
        "(and(robot_at tiago fridge_zone)(object_at rubber_duck small_bathroom))"));
  }

  void step()
  {
    auto feedback = executor_client_->getFeedBack();
    for (const auto & action_feedback : feedback.action_execution_status) {
      if(action_feedback.completion != 0 && action_feedback.completion != 1){
        std::cout << "[" << action_feedback.action << " " <<
                action_feedback.completion * 100.0 << "%]";
        std::cout << std::endl;
      }
    }

    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
      }
    }
  }

private:
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApartmentController>();

  node->init();
  std::cout << "Planning..." << std::endl;

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}