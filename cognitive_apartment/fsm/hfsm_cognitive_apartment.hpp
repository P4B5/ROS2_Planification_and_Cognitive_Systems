// NOLINT (legal/copyright) 

#ifndef HFSM_COGNITIVE_APARTMENT_H_
#define HFSM_COGNITIVE_APARTMENT_H_

#include <string>

#include "std_msgs/msg/string.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace cascade_hfsm
{
class hfsm_cognitive_apartment : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  hfsm_cognitive_apartment();
  virtual ~hfsm_cognitive_apartment();

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  virtual void init_code_iterative() {}
  virtual void init_code_once() {}
  virtual void living_room_code_iterative() {}
  virtual void living_room_code_once() {}
  virtual void bathroom_code_iterative() {}
  virtual void bathroom_code_once() {}
  virtual void corridor_code_iterative() {}
  virtual void corridor_code_once() {}
  virtual void kitchen_code_iterative() {}
  virtual void kitchen_code_once() {}
  virtual void finish_code_iterative() {}
  virtual void finish_code_once() {}
  virtual void bedroom_code_iterative() {}
  virtual void bedroom_code_once() {}

  virtual bool init_2_kitchen() {return false;}
  virtual bool kitchen_2_living_room() {return false;}
  virtual bool living_room_2_corridor() {return false;}
  virtual bool corridor_2_bathroom() {return false;}
  virtual bool bedroom_2_finish() {return false;}
  virtual bool bathroom_2_bedroom() {return false;}


  void tick();

protected:
  rclcpp::Time state_ts_;

private:
  void deactivateAllDeps();
  void init_activateDeps();
  void living_room_activateDeps();
  void bathroom_activateDeps();
  void corridor_activateDeps();
  void kitchen_activateDeps();
  void finish_activateDeps();
  void bedroom_activateDeps();


  static const int INIT = 0;
  static const int LIVING_ROOM = 1;
  static const int BATHROOM = 2;
  static const int CORRIDOR = 3;
  static const int KITCHEN = 4;
  static const int FINISH = 5;
  static const int BEDROOM = 6;


  int state_;

  std::string myBaseId_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr place_pub_;
};

}  // namespace cascade_hfsm

#endif  // HFSM_COGNITIVE_APARTMENT_H_
