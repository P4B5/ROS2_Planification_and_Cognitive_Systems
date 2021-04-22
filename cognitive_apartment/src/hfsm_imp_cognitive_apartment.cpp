#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "hfsm_cognitive_apartment.hpp"
// #include "blackboard/BlackBoard.hpp"


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
      if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
         if (executor_client_->getResult().value().success) {
               std::cout << "Successful finished " << std::endl;
               return true;
         } else {
            //CHANGE HERE THE PRINT TO VISUALIZE CORRECTLY THE STATUS
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
      if(get_plan_state() == true){
         problem_expert_->removePredicate(plansys2::Predicate("(place_explored corridor)"));
         problem_expert_->setGoal(plansys2::Goal("(and(place_explored big_bathroom))"));

         if (executor_client_->start_plan_execution()) {
            state_ = BATHROOM;
            return true;
         }
      }
      return false;
   }

   bool bathroom_2_bedroom() {
      if(get_plan_state() == true){
         problem_expert_->removePredicate(plansys2::Predicate("(place_explored big_bathroom)"));
         problem_expert_->setGoal(plansys2::Goal("(and(place_explored computer_zone))"));
         if (executor_client_->start_plan_execution()) {
            state_ = BEDROOM;
            return true;
         }
      }
   return false;
   }

    bool kitchen_2_living_room() {
      if(get_plan_state() == true){
         problem_expert_->removePredicate(plansys2::Predicate("(place_explored kitchen)"));
         problem_expert_->setGoal(plansys2::Goal("(and(place_explored dining_zone))"));
         if (executor_client_->start_plan_execution()) {
            state_ = LIVING_ROOM;
            return true;
         }
      }
      return false;
    }


   bool living_room_2_corridor() {
      if(get_plan_state() == true){
         problem_expert_->removePredicate(plansys2::Predicate("(place_explored dining_zone)"));
         problem_expert_->setGoal(plansys2::Goal("(and(place_explored corridor))"));
         if (executor_client_->start_plan_execution()) {
            state_ = CORRIDOR;
            return true;
         }
      }
      return false;
   }

    bool init_2_kitchen(){
      if (executor_client_->start_plan_execution()) {
         state_ = KITCHEN;
         return true;
      }
      return false;
    }

      //----- ONCE -----

     void kitchen_code_once() {
      //   RCLCPP_INFO(get_logger(), "Robot at KITCHEN state once");
      
     }
     void corridor_code_once() {
      //   RCLCPP_INFO(get_logger(), "Robot at CORRIDOR");
     }

     void bedroom_code_once() {
      //   RCLCPP_INFO(get_logger(), "Robot at BEDROOM");
     }

     void living_room_code_once() {
      //   RCLCPP_INFO(get_logger(), "Robot at LIVING ROOM");
     }

     void bathroom_code_once() {
      
     }
   

   // ----- ITERATIVE -----

   void show_planer_state(){
       auto feedback = executor_client_->getFeedBack();
         for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
               action_feedback.completion * 100.0 << "%]";
         }
         std::cout << std::endl;
   }

     void corridor_code_iterative() {
         RCLCPP_INFO(get_logger(), "CORRIDOR STATE");
         show_planer_state();
     }
   
     void kitchen_code_iterative() {
         RCLCPP_INFO(get_logger(), "KITCHEN STATE");
         show_planer_state();
     }

     void bedroom_code_iterative() {
         RCLCPP_INFO(get_logger(), "BEDROOM STATE");
         show_planer_state();
     }
   
     void bathroom_code_iterative() {
         RCLCPP_INFO(get_logger(), "BATHROOM STATE");
         show_planer_state();
     }
     
     void living_room_code_iterative() {
        RCLCPP_INFO(get_logger(), "LIVING ROOM STATE");
        show_planer_state();
     }
    
     void init_code_iterative() {
        // RCLCPP_INFO(get_logger(), "Robot at ");
     }

    
   void init_code_once()
   {
      auto node = std::make_shared<rclcpp::Node>("node_aux");
      problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(node);
      executor_client_ = std::make_shared<plansys2::ExecutorClient>(node);
      RCLCPP_INFO(get_logger(), "Predicates successful stablished");
      init_knowledge();
   }

  void init_knowledge()
  {
      // RCLCPP_INFO(get_logger(), "Stablishing predicates");
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

      

      problem_expert_->setGoal(plansys2::Goal("(and(place_explored kitchen))"));
  }


   private:
      typedef enum {INIT, KITCHEN, BEDROOM, BATHROOM, LIVING_ROOM, FINISH, CORRIDOR} StateType;
      StateType state_;
      std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
      std::shared_ptr<plansys2::ExecutorClient> executor_client_;
      

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
   std::cout << "finish configuration\n";
   rclcpp::shutdown();

  return 0;
}



