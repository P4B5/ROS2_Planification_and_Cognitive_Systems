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
  virtual void move_code_iterative() {}
  virtual void move_code_once() {}
  virtual void place_code_iterative() {}
  virtual void place_code_once() {}
  virtual void pick_code_iterative() {}
  virtual void pick_code_once() {}

  virtual bool pick_2_move() {return false;}
  virtual bool init_2_move() {return false;}
  virtual bool move_2_pick() {return false;}
  virtual bool place_2_move() {return false;}
  virtual bool move_2_place() {return false;}


  void tick();

protected:
  rclcpp::Time state_ts_;

private:
  void deactivateAllDeps();
  void init_activateDeps();
  void move_activateDeps();
  void place_activateDeps();
  void pick_activateDeps();


  static const int INIT = 0;
  static const int MOVE = 1;
  static const int PLACE = 2;
  static const int PICK = 3;


  int state_;

  std::string myBaseId_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
};

}  // namespace cascade_hfsm

#endif  // HFSM_COGNITIVE_APARTMENT_H_
