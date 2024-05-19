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
#include <vector>
#include "stp/play.hpp"
#include "types/world.hpp"

namespace ateam_kenobi
{

class PlaySelector
{
public:
  PlaySelector();

  stp::Play * getPlay(const World & world);

  void setPlayOverride(const std::string & play_name)
  {
    override_play_name_ = play_name;
  }

  std::vector<std::string> getPlayNames();

  stp::Play * getPlayByName(const std::string name);

private:
  std::shared_ptr<stp::Play> halt_play_;
  std::vector<std::shared_ptr<stp::Play>> plays_;
  std::string override_play_name_;
  void * prev_play_address_ = nullptr;

  template<typename PlayType>
  std::shared_ptr<stp::Play> addPlay()
  {
    auto play = std::make_shared<PlayType>();
    plays_.push_back(play);
    return play;
  }

  stp::Play * selectOverridePlay();

  stp::Play * selectRankedPlay(const World & world);

  void resetPlayIfNeeded(stp::Play * play);
};

}  // namespace ateam_kenobi

#endif  // PLAY_SELECTOR_HPP_
