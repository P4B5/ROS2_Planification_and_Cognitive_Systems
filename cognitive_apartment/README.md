# Apartment, cognitive implementation

This implementation of the apartment exercise, it has been implemented using cognitive behavior which provides a better response for robots executing complex tasks. 

## Program Execution

1. `ros2 launch cognitive_apartment plansys2_apartment_launch.py `

2. `rviz2`

3. `ros2 run cognitive_apartment hfsm_cognitive_apartment `



## State Machine

The state machine consist of:
- 5 states of location {kitchen, living_room, corridor, bathroom, bedroom}
- 1 state of configuration {init}
- 1 state of finish {finish}: this node just give info to the user

execute hfsm gui: `ros2 run rqt_gui rqt_gui`

![](https://github.com/P4B5/ROS2_Planification_and_Cognitive_Systems/blob/main/docs/bica.png)


## BlackBoard

- example of add entry to the blackboard:

```
auto kitchen_explored = blackboard::Entry<bool>::make_shared(true);   
auto entry_base = kitchen_explored->to_base();   
blackboard->add_entry("kitchen", kitchen_explored->to_base());

```

- example of get an entry of the blackboard;

```
auto entry_1_got = blackboard::as<bool>(blackboard->get_entry("kitchen"));
if(entry_1_got->data_){std::cout << "Kitchen was explored" << std::endl;}

```




- topics to track info of the blackboard

`/blackboard/info/places_explored` :explored places (String)

`/blackboard/info/tf_explored`: explored places (tf)

## Videos

[video 1](https://www.youtube.com/watch?v=3u2zh0XNpuo)
 
