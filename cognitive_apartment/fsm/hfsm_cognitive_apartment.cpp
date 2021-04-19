// NOLINT (legal/copyright)

#include "hfsm_cognitive_apartment.hpp"

namespace cascade_hfsm
{
hfsm_cognitive_apartment::hfsm_cognitive_apartment()
: CascadeLifecycleNode("hfsm_cognitive_apartment"), state_(INIT), myBaseId_("hfsm_cognitive_apartment")
{
  declare_parameter("frequency");

  state_ts_ = now();
  state_pub_ = create_publisher<std_msgs::msg::String>("/" + myBaseId_ + "/state", 1);
}

hfsm_cognitive_apartment::~hfsm_cognitive_apartment()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
hfsm_cognitive_apartment::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  deactivateAllDeps();

  state_ = INIT;
  state_ts_ = now();

  init_activateDeps();
  init_code_once();


  double frequency = 5.0;
  get_parameter_or<double>("frequency", frequency, 5.0);

  loop_timer_ = create_wall_timer(
    std::chrono::duration<double, std::ratio<1>>(1.0 / frequency),
    std::bind(&hfsm_cognitive_apartment::tick, this));

  state_pub_->on_activate();
  return CascadeLifecycleNode::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
hfsm_cognitive_apartment::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  loop_timer_ = nullptr;

  return CascadeLifecycleNode::on_deactivate(previous_state);
}

void hfsm_cognitive_apartment::tick()
{
  std_msgs::msg::String msg;

  switch (state_) {
    case INIT:
      init_code_iterative();

      msg.data = "init";
      state_pub_->publish(msg);

      if (init_2_move()) {
        deactivateAllDeps();

        state_ = MOVE;
        state_ts_ = now();

        move_activateDeps();
        move_code_once();
      }
      break;
    case MOVE:
      move_code_iterative();

      msg.data = "move";
      state_pub_->publish(msg);

      if (move_2_pick()) {
        deactivateAllDeps();

        state_ = PICK;
        state_ts_ = now();

        pick_activateDeps();
        pick_code_once();
      }
      if (move_2_place()) {
        deactivateAllDeps();

        state_ = PLACE;
        state_ts_ = now();

        place_activateDeps();
        place_code_once();
      }
      break;
    case PLACE:
      place_code_iterative();

      msg.data = "place";
      state_pub_->publish(msg);

      if (place_2_move()) {
        deactivateAllDeps();

        state_ = MOVE;
        state_ts_ = now();

        move_activateDeps();
        move_code_once();
      }
      break;
    case PICK:
      pick_code_iterative();

      msg.data = "pick";
      state_pub_->publish(msg);

      if (pick_2_move()) {
        deactivateAllDeps();

        state_ = MOVE;
        state_ts_ = now();

        move_activateDeps();
        move_code_once();
      }
      break;
  }
}

void
hfsm_cognitive_apartment::deactivateAllDeps()
{
}

void
hfsm_cognitive_apartment::init_activateDeps()
{
}
void
hfsm_cognitive_apartment::move_activateDeps()
{
}
void
hfsm_cognitive_apartment::place_activateDeps()
{
}
void
hfsm_cognitive_apartment::pick_activateDeps()
{
}


}  // namespace cascade_hfsm
