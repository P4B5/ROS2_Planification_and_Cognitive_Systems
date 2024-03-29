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
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "sync_node.hpp"

#include "hfsm_cognitive_apartment.hpp"

#include "blackboard/BlackBoard.hpp"


using std::placeholders::_1;


class TestComp : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
  public:
    TestComp() : CascadeLifecycleNode("TestComp")
    {

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state)
    {

      RCLCPP_INFO(get_logger(), "Activado TestComp");
      return CascadeLifecycleNode::on_activate(previous_state);
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {

      RCLCPP_INFO(get_logger(), "Desactivado TestComp");
      return CascadeLifecycleNode::on_deactivate(previous_state);
    }
};


class  hsfm_imp_cognitive_apartment: public cascade_hfsm::hfsm_cognitive_apartment
{
public:
   hsfm_imp_cognitive_apartment()
  : hfsm_cognitive_apartment()
  {
     
  }

   bool get_plan_state(){
     auto feedback = executor_client_->getFeedBack();
      if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
         if (executor_client_->getResult().value().success) {
            std::cout << "=======================\n";
               return true;
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
      return false;
   }

   bool corridor_2_bathroom() {
      return get_plan_state();
   }

   bool bathroom_2_bedroom() {
      return get_plan_state();
   }

   bool kitchen_2_living_room() {
      return get_plan_state();
   }

   bool living_room_2_corridor() {
      return get_plan_state();
   }

   bool init_2_kitchen(){
      if (executor_client_->start_plan_execution()) {
         return true;
      }
      return false;
   }

   bool bedroom_2_tidy_apartment(){
      return get_plan_state();
   }

   bool tidy_apartment_2_finish(){
      return get_plan_state();
   }

   //----- ONCE -----

   void init_code_once()
   {
      RCLCPP_INFO(get_logger(), "INIT STATE");    
      auto node = std::make_shared<rclcpp::Node>("node_aux");
      problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(node);
      executor_client_ = std::make_shared<plansys2::ExecutorClient>(node);
      place_pub_ = node->create_publisher<std_msgs::msg::String>("/blackboard/info/places_explored", 100);
      tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
      init_knowledge();
   }

   void kitchen_code_once() {
      RCLCPP_INFO(get_logger(), "KITCHEN STATE");    
   }

   void living_room_code_once() {

      //set new goal
      problem_expert_->removePredicate(plansys2::Predicate("(place_explored kitchen)"));
      problem_expert_->setGoal(plansys2::Goal("(and(place_explored dining_zone))"));
 
      auto kitchen_explored = blackboard::Entry<bool>::make_shared(true);   
      client_state->add_entry("kitchen", kitchen_explored->to_base());

      auto message = std_msgs::msg::String();
      message.data = "kitchen";
      place_pub_->publish(message);
      tf_broadcaster_->sendTransform(blackboard::as<geometry_msgs::msg::TransformStamped>(client_state->get_entry("kitchen_tf"))->data_);

      RCLCPP_INFO(get_logger(), "LIVING ROOM STATE");

      executor_client_->start_plan_execution();

      
   }

   void corridor_code_once() {

      problem_expert_->removePredicate(plansys2::Predicate("(place_explored dining_zone)"));
      problem_expert_->setGoal(plansys2::Goal("(and(place_explored corridor))"));
        
      auto living_room_explored = blackboard::Entry<bool>::make_shared(true);   
      client_state->add_entry("living_room", living_room_explored->to_base());

      auto message = std_msgs::msg::String();
      message.data = "living room";
      place_pub_->publish(message);
      tf_broadcaster_->sendTransform(blackboard::as<geometry_msgs::msg::TransformStamped>(client_state->get_entry("dining_zone_tf"))->data_);

      RCLCPP_INFO(get_logger(), "CORRIDOR STATE");

      executor_client_->start_plan_execution();
   }

   void bathroom_code_once() {
      problem_expert_->removePredicate(plansys2::Predicate("(place_explored corridor)"));
      problem_expert_->setGoal(plansys2::Goal("(and(place_explored big_bathroom))"));

      auto corridor_explored = blackboard::Entry<bool>::make_shared(true);   
      client_state->add_entry("corridor", corridor_explored->to_base());

      auto message = std_msgs::msg::String();
      message.data = "corridor";
      place_pub_->publish(message);
      tf_broadcaster_->sendTransform(blackboard::as<geometry_msgs::msg::TransformStamped>(client_state->get_entry("corridor_tf"))->data_);

      RCLCPP_INFO(get_logger(), "BATHROOM STATE");

      executor_client_->start_plan_execution();
   }

   void bedroom_code_once() {
      problem_expert_->removePredicate(plansys2::Predicate("(place_explored big_bathroom)"));
      problem_expert_->setGoal(plansys2::Goal("(and(place_explored computer_zone))"));

      auto bathroom_explored = blackboard::Entry<bool>::make_shared(true);   
      blackboard->add_entry("bathroom", bathroom_explored->to_base());

      auto message = std_msgs::msg::String();
      message.data = "bathroom";
      place_pub_->publish(message);
      tf_broadcaster_->sendTransform(blackboard::as<geometry_msgs::msg::TransformStamped>(client_state->get_entry("big_bathroom_tf"))->data_);


   
      RCLCPP_INFO(get_logger(), "BEDROOM STATE");

      executor_client_->start_plan_execution();
   }

   void tidy_apartment_code_once() {
      problem_expert_->removePredicate(plansys2::Predicate("(place_explored computer_zone)"));
      problem_expert_->setGoal(plansys2::Goal("(and(object_at rubber_duck kitchen))"));

      auto bedroom_explored = blackboard::Entry<bool>::make_shared(true);   
      client_state->add_entry("bedroom", bedroom_explored->to_base());

      auto message = std_msgs::msg::String();
      message.data = "bedroom";
      place_pub_->publish(message);
      tf_broadcaster_->sendTransform(blackboard::as<geometry_msgs::msg::TransformStamped>(client_state->get_entry("computer_zone_tf"))->data_);

      RCLCPP_INFO(get_logger(), "TIDY ROOM");

      executor_client_->start_plan_execution();

   }

   void finish_code_once() {
      if(blackboard::as<bool>(client_state->get_entry("kitchen"))->data_){std::cout << "Kitchen was explored" << std::endl;}
      if(blackboard::as<bool>(client_state->get_entry("living_room"))->data_){std::cout << "Living_room was explored" << std::endl;}
      if(blackboard::as<bool>(client_state->get_entry("corridor"))->data_){std::cout << "Corridor was explored" << std::endl;} 
      if(blackboard::as<bool>(client_state->get_entry("bathroom"))->data_){std::cout << "Bathroom was explored" << std::endl;}
      if(blackboard::as<bool>(client_state->get_entry("bedroom"))->data_){std::cout << "Bedroom was explored" << std::endl;}

      RCLCPP_INFO(get_logger(), "FINISH STATE");

      executor_client_->start_plan_execution();

   }


   // ----- ITERATIVE -----

   void show_planer_state(){
      auto feedback = executor_client_->getFeedBack();
      for (const auto & action_feedback : feedback.action_execution_status) {
         if(action_feedback.completion != 0 && action_feedback.completion != 1){
            std::cout << "["  <<
               action_feedback.completion * 100.0 << "%]";
            std::cout << std::endl;
         }
      }   
   }

   void corridor_code_iterative() {
      show_planer_state();
   }

   void kitchen_code_iterative() {
      show_planer_state();
   }

   void bedroom_code_iterative() { 
      show_planer_state();
   }

   void bathroom_code_iterative() {
      show_planer_state();
   }
   
   void living_room_code_iterative() {
      show_planer_state();
   }

   void tidy_apartment_code_iterative() {
      show_planer_state();
   }

   void finsish_code_iterative() {
      show_planer_state();
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
      // problem_expert_->addInstance(plansys2::Instance{"laptop", "object"});
      // problem_expert_->addInstance(plansys2::Instance{"paper_boat", "object"});
      // problem_expert_->addInstance(plansys2::Instance{"biscuits", "object"});
      // problem_expert_->addInstance(plansys2::Instance{"gray_pillow", "object"});
     
      //objects to detect

      problem_expert_->addInstance(plansys2::Instance{"rubber_duck", "object"});
      problem_expert_->addInstance(plansys2::Instance{"computer", "object"});
      problem_expert_->addInstance(plansys2::Instance{"orange", "object"});
      problem_expert_->addInstance(plansys2::Instance{"bowl", "object"});
      problem_expert_->addInstance(plansys2::Instance{"fire_extinguisher", "object"});

      problem_expert_->addInstance(plansys2::Instance{"tiago", "robot"});

      /////
      problem_expert_->addInstance(plansys2::Instance{"rubber_duck", "object"});
      problem_expert_->addInstance(plansys2::Instance{"computer", "object"});
      problem_expert_->addInstance(plansys2::Instance{"orange", "object"});
      problem_expert_->addInstance(plansys2::Instance{"bowl", "object"});
      problem_expert_->addInstance(plansys2::Instance{"fire_extinguisher", "object"});

      problem_expert_->addPredicate(plansys2::Predicate("(object_at computer computer_zone)"));
      problem_expert_->addPredicate(plansys2::Predicate("(object_at bowl living_room)"));
      problem_expert_->addPredicate(plansys2::Predicate("(object_at fire_extinguisher corridor)"));
      problem_expert_->addPredicate(plansys2::Predicate("(object_at rubber_duck big_bathroom)"));
      problem_expert_->addPredicate(plansys2::Predicate("(object_at orange kitchen)"));
      /////

      problem_expert_->addPredicate(plansys2::Predicate("(robot_at tiago living_room)"));
      problem_expert_->addPredicate(plansys2::Predicate("(robot_idle tiago)"));
      
      problem_expert_->setGoal(plansys2::Goal("(and(place_explored kitchen))"));
   }


   private:
      typedef enum {INIT, KITCHEN, BEDROOM, BATHROOM, LIVING_ROOM, FINISH, CORRIDOR, TIDY_APARTMENT} StateType;
      StateType state_;
      std::string place_;
      std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
      std::shared_ptr<plansys2::ExecutorClient> executor_client_;
   
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr place_pub_;
      std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

      std::shared_ptr<blackboard::BlackBoardClient> client_state = blackboard::BlackBoardClient::make_shared();
};


int main(int argc, char ** argv)
{
   rclcpp::init(argc, argv);
   
   auto my_hfsm = std::make_shared<hsfm_imp_cognitive_apartment>();
   
   auto test_node = std::make_shared<TestComp>();
   rclcpp::executors::SingleThreadedExecutor executor;
   
   executor.add_node(my_hfsm->get_node_base_interface());
   executor.add_node(test_node->get_node_base_interface());
   my_hfsm->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
   test_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
   executor.spin_some();
   my_hfsm->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

   executor.spin();
   rclcpp::shutdown();
   
   return 0;
}



