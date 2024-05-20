// Copyright 2024 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#ifndef STP__BASE_HPP_
#define STP__BASE_HPP_

#include <string>
#include <utility>
#include <nlohmann/json.hpp>
#include <rclcpp/logger.hpp>
#include "visualization/overlays.hpp"

namespace ateam_kenobi::stp
{

struct Options
{
  std::string name;
  visualization::Overlays overlays;
  nlohmann::json play_info;
  rclcpp::Logger logger = rclcpp::get_logger("kenobi_stp_default");
};

class Base
{
public:
  explicit Base(std::string name)
  : name_(name),
    logger_(rclcpp::get_logger(name))
  {
  }

  explicit Base(Options options)
  : name_(options.name),
    overlays_(options.overlays),
    play_info_(options.play_info),
    logger_(options.logger)
  {
  }

  Base(std::string name, Options options)
  : name_(name),
    overlays_(options.overlays),
    play_info_(options.play_info),
    logger_(options.logger)
  {
  }

  virtual ~Base() = default;

  template<typename ChildType, typename ... Args>
  ChildType createChild(std::string child_name, Args &&... args)
  {
    Options options{
      child_name,
      overlays_.getChild(child_name),
      play_info_[child_name],
      logger_.get_child(child_name)
    };
    return ChildType(options, std::forward<Args>(args)...);
  }

  const std::string & getName() const
  {
    return name_;
  }

  visualization::Overlays & getOverlays()
  {
    return overlays_;
  }

  nlohmann::json & getPlayInfo()
  {
    return play_info_;
  }

  rclcpp::Logger & getLogger()
  {
    return logger_;
  }

private:
  std::string name_;
  visualization::Overlays overlays_;
  nlohmann::json play_info_;
  rclcpp::Logger logger_;
};

}  // namespace ateam_kenobi::stp

#endif  // STP__BASE_HPP_
