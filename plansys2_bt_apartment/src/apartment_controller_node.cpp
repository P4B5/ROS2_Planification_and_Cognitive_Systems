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

class Apartment : public rclcpp::Node
{
public:
  Apartment()
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
    
    
      //CHANGE THE PLAN HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});

//     problem_expert_->addInstance(plansys2::Instance{"car_1", "car"});
//     problem_expert_->addInstance(plansys2::Instance{"car_2", "car"});
//     problem_expert_->addInstance(plansys2::Instance{"car_3", "car"});

//     problem_expert_->addInstance(plansys2::Instance{"wheels_zone", "zone"});
//     problem_expert_->addInstance(plansys2::Instance{"steering_wheels_zone", "zone"});
//     problem_expert_->addInstance(plansys2::Instance{"body_car_zone", "zone"});
//     problem_expert_->addInstance(plansys2::Instance{"assembly_zone", "zone"});
//     problem_expert_->addInstance(plansys2::Instance{"recharge_zone", "zone"});


//     problem_expert_->addInstance(plansys2::Instance{"wheel_1", "piece"});
//     problem_expert_->addInstance(plansys2::Instance{"wheel_2", "piece"});
//     problem_expert_->addInstance(plansys2::Instance{"wheel_3", "piece"});

//     problem_expert_->addInstance(plansys2::Instance{"body_car_1", "piece"});
//     problem_expert_->addInstance(plansys2::Instance{"body_car_2", "piece"});
//     problem_expert_->addInstance(plansys2::Instance{"body_car_3", "piece"});

//     problem_expert_->addInstance(plansys2::Instance{"steering_wheel_1", "piece"});
//     problem_expert_->addInstance(plansys2::Instance{"steering_wheel_2", "piece"});
//     problem_expert_->addInstance(plansys2::Instance{"steering_wheel_3", "piece"});

//     problem_expert_->addPredicate(plansys2::Predicate("(is_assembly_zone assembly_zone)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(is_recharge_zone recharge_zone)"));

//     problem_expert_->addPredicate(plansys2::Predicate("(piece_is_wheel wheel_1)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_is_wheel wheel_2)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_is_wheel wheel_3)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_at wheel_1 wheels_zone)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_at wheel_2 wheels_zone)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_at wheel_3 wheels_zone)"));

//     problem_expert_->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_1)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_2)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_3)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_at body_car_1 body_car_zone)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_at body_car_2 body_car_zone)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_at body_car_3 body_car_zone)"));

//     problem_expert_->addPredicate(plansys2::Predicate("(piece_is_sterwheel steering_wheel_1)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_is_sterwheel steering_wheel_2)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_is_sterwheel steering_wheel_3)"));
//     problem_expert_->addPredicate(
//       plansys2::Predicate("(piece_at steering_wheel_1 steering_wheels_zone)"));
//     problem_expert_->addPredicate(
//       plansys2::Predicate("(piece_at steering_wheel_2 steering_wheels_zone)"));
//     problem_expert_->addPredicate(
//       plansys2::Predicate("(piece_at steering_wheel_3 steering_wheels_zone)"));

//     problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wheels_zone)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(battery_full r2d2)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(robot_available r2d2)"));

//     problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used wheel_1)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used wheel_2)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used wheel_3)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used body_car_1)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used body_car_2)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used body_car_3)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used steering_wheel_1)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used steering_wheel_2)"));
//     problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used steering_wheel_3)"));

//     problem_expert_->setGoal(
//       plansys2::Goal(
//         "(and(car_assembled car_1) (car_assembled car_2) (car_assembled car_3))"));
  }

  void step()
  {
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
  auto node = std::make_shared<Apartment>();

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
