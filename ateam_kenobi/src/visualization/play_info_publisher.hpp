// Copyright 2021 A Team
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


#ifndef VISUALIZATION__PLAY_INFO_PUBLISHER_HPP_
#define VISUALIZATION__PLAY_INFO_PUBLISHER_HPP_

#include <rclcpp/node.hpp>
#include <ateam_msgs/msg/play_info.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace ateam_kenobi::visualization
{

class PlayInfoPublisher
{
public:
  explicit PlayInfoPublisher(rclcpp::Node & node);
  void send_play_message(const std::string & play_name);

  json message;   // json data for a play to fill out. Is sent and then cleared when publishing

private:
  rclcpp::Publisher<ateam_msgs::msg::PlayInfo>::SharedPtr publisher_;
};
} // namespace ateam_kenobi::visualization

#endif // VISUALIZATION__PLAY_INFO_PUBLISHER_HPP_
