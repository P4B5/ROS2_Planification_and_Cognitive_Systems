// // Copyright 2021 Intelligent Robotics Lab
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.



#include <vector>
#include <math.h>
#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include "object_recognition.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "1\n";
  auto node = std::make_shared<Filter>(); 
  std::cout << "2\n";
  rclcpp::spin(node->get_node_base_interface());;
  std::cout << "3\n";
  rclcpp::shutdown();
  std::cout << "4\n";
  return 0;
}

