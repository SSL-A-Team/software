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


#ifndef PLAY_SELECTOR_HPP_
#define PLAY_SELECTOR_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <ateam_msgs/msg/playbook_state.hpp>
#include "core/stp/play.hpp"
#include "core/types/world.hpp"

namespace ateam_kenobi
{

class PlaySelector
{
public:
  explicit PlaySelector(rclcpp::Node & node);

  stp::Play * getPlay(const World & world, ateam_msgs::msg::PlaybookState & state_msg);

  void setPlayOverride(const std::string & play_name)
  {
    override_play_name_ = play_name;
  }

  std::vector<std::string> getPlayNames();

  stp::Play * getPlayByName(const std::string name);

  void saveToFile(const std::filesystem::path & path);

  void loadFromFile(const std::filesystem::path & path);

private:
  rclcpp::Logger ros_logger_;
  std::shared_ptr<stp::Play> halt_play_;
  std::vector<std::shared_ptr<stp::Play>> plays_;
  std::string override_play_name_;
  void * prev_play_address_ = nullptr;

  template<typename PlayType>
  std::shared_ptr<stp::Play> addPlay(stp::Options stp_options)
  {
    return addPlay<PlayType>(PlayType::kPlayName, stp_options);
  }

  template<typename PlayType, typename ... Args>
  std::shared_ptr<stp::Play> addPlay(
    const std::string & name, stp::Options stp_options,
    Args &&... args)
  {
    stp_options.name = name;
    stp_options.overlays = visualization::Overlays(name);
    stp_options.logger = stp_options.logger.get_child(name);
    stp_options.parameter_interface = stp_options.parameter_interface.getChild(name);
    auto play = std::make_shared<PlayType>(stp_options, std::forward<Args>(args)...);
    plays_.push_back(play);
    return play;
  }

  stp::Play * selectOverridePlay();

  stp::Play * selectRankedPlay(const World & world, std::vector<double> & scores_out);

  void resetPlayIfNeeded(stp::Play * play);

  void fillStateMessage(
    ateam_msgs::msg::PlaybookState & msg, std::vector<double> & scores,
    const stp::Play * selected_play);
};

}  // namespace ateam_kenobi

#endif  // PLAY_SELECTOR_HPP_
