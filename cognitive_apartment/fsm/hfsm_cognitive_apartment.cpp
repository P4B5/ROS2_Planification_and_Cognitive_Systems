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
    case BATHROOM:
      bathroom_code_iterative();

      msg.data = "bathroom";
      state_pub_->publish(msg);

      if (bathroom_2_bedroom()) {
        deactivateAllDeps();

        state_ = BEDROOM;
        state_ts_ = now();

        bedroom_activateDeps();
        bedroom_code_once();
      }
      break;
    case CORRIDOR:
      corridor_code_iterative();

      msg.data = "corridor";
      state_pub_->publish(msg);

      if (corridor_2_bathroom()) {
        deactivateAllDeps();

        state_ = BATHROOM;
        state_ts_ = now();

        bathroom_activateDeps();
        bathroom_code_once();
      }
      break;
    case LIVING_ROOM:
      living_room_code_iterative();

      msg.data = "living_room";
      state_pub_->publish(msg);

      if (living_room_2_corridor()) {
        deactivateAllDeps();

        state_ = CORRIDOR;
        state_ts_ = now();

        corridor_activateDeps();
        corridor_code_once();
      }
      break;
    case KITCHEN:
      kitchen_code_iterative();

      msg.data = "kitchen";
      state_pub_->publish(msg);

      if (kitchen_2_living_room()) {
        deactivateAllDeps();

        state_ = LIVING_ROOM;
        state_ts_ = now();

        living_room_activateDeps();
        living_room_code_once();
      }
      break;
    case INIT:
      init_code_iterative();

      msg.data = "init";
      state_pub_->publish(msg);

      if (init_2_kitchen()) {
        deactivateAllDeps();

        state_ = KITCHEN;
        state_ts_ = now();

        kitchen_activateDeps();
        kitchen_code_once();
      }
      break;
    case FINISH:
      finish_code_iterative();

      msg.data = "finish";
      state_pub_->publish(msg);

      break;
    case BEDROOM:
      bedroom_code_iterative();

      msg.data = "bedroom";
      state_pub_->publish(msg);

      if (bedroom_2_finish()) {
        deactivateAllDeps();

        state_ = FINISH;
        state_ts_ = now();

        finish_activateDeps();
        finish_code_once();
      }
      break;
  }
}

void
hfsm_cognitive_apartment::deactivateAllDeps()
{
}

void
hfsm_cognitive_apartment::bathroom_activateDeps()
{
}
void
hfsm_cognitive_apartment::corridor_activateDeps()
{
}
void
hfsm_cognitive_apartment::living_room_activateDeps()
{
}
void
hfsm_cognitive_apartment::kitchen_activateDeps()
{
}
void
hfsm_cognitive_apartment::init_activateDeps()
{
}
void
hfsm_cognitive_apartment::finish_activateDeps()
{
}
void
hfsm_cognitive_apartment::bedroom_activateDeps()
{
}


}  // namespace cascade_hfsm
