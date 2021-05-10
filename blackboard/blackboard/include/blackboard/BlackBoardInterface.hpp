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

#ifndef BLACKBOARD__BLACKBOARDINTERFACE_HPP_
#define BLACKBOARD__BLACKBOARDINTERFACE_HPP_

#include <map>
#include <memory>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace blackboard
{


class EntryBase
{
public:
  using Ptr = std::shared_ptr<EntryBase>;

  static const int UNKNOWN = -1;
  static const int BOOL = 0;
  static const int STRING = 1;
  static const int FLOAT = 2;
  static const int TRANSFORM = 3;
  static const int OCTOMAP = 4;
  static const int POSE = 5;

  virtual int get_type() {return type_;}

protected:
  int type_ = UNKNOWN;
};

template<class T>
class Entry : public EntryBase, public std::enable_shared_from_this<Entry<T>>
{
public:
  using Ptr = std::shared_ptr<Entry<T>>;
  static Ptr make_shared(const T & data)
  {
    return std::make_shared<Entry<T>>(data);
  }
  using std::enable_shared_from_this<Entry<T>>::shared_from_this;

  explicit Entry(const T & data)
  {
    data_ = data;

    if (std::is_same<bool, T>::value) {
      type_ = EntryBase::BOOL;
    } else if (std::is_same<std::string, T>::value) {
      type_ = EntryBase::STRING;
    } else if (std::is_same<float, T>::value) {
      type_ = EntryBase::FLOAT;
    } else if (std::is_same<geometry_msgs::msg::TransformStamped, T>::value) {
      type_ = EntryBase::TRANSFORM;
    } else if (std::is_same<octomap_msgs::msg::Octomap, T>::value) {
      type_ = EntryBase::OCTOMAP;
    } else if (std::is_same<geometry_msgs::msg::PoseStamped, T>::value) {
      type_ = EntryBase::POSE;
    }
  }

  blackboard::EntryBase::Ptr to_base()
  {
    return std::dynamic_pointer_cast<blackboard::EntryBase>(shared_from_this());
  }

  T data_;
};

template<class T>
typename Entry<T>::Ptr as(EntryBase::Ptr entry)
{
  auto ret = std::static_pointer_cast<blackboard::Entry<T>>(entry);
  return ret;
}

class BlackBoardInterface
{
public:
  virtual void add_entry(const std::string & key, EntryBase::Ptr entry) = 0;
  virtual EntryBase::Ptr get_entry(const std::string & key) = 0;
};

}  // namespace blackboard

#endif  // BLACKBOARD__BLACKBOARDINTERFACE_HPP_
