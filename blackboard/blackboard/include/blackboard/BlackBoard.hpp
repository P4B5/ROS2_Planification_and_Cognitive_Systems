// Copyright 2021 Intelligent Robotics Lab
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

#ifndef BLACKBOARD__BLACKBOARD_HPP_
#define BLACKBOARD__BLACKBOARD_HPP_

#include <map>
#include <memory>
#include <string>

#include "blackboard/BlackBoardInterface.hpp"

namespace blackboard
{


class BlackBoard : public BlackBoardInterface
{
public:
  using Ptr = std::shared_ptr<BlackBoard>;
  static Ptr make_shared()
  {
    return std::make_shared<BlackBoard>();
  }

  void add_entry(const std::string & key, EntryBase::Ptr entry);
  EntryBase::Ptr get_entry(const std::string & key);

protected:
  std::map<std::string, EntryBase::Ptr> entries_;
};

}  // namespace blackboard

#endif  // BLACKBOARD__BLACKBOARD_HPP_
