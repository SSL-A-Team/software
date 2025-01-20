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

#ifndef MAPS__TEST_MAP_HPP_
#define MAPS__TEST_MAP_HPP_

#include "spatial/spatial_map_factory.hpp"

namespace ateam_kenobi::spatial::maps
{

class TestMap : public SpatialMapFactory {
public:
  TestMap()
  : SpatialMapFactory("TestMap") {}

  void FillMap(
    cv::Mat & map, const World &,
    const std::unordered_map<std::string, cv::Mat> & layers) override
  {
    const auto & goal_sight = layers.at("LineOfSightTheirGoal");
    const auto & ball_sight = layers.at("LineOfSightBall");
    const auto & def_area_keepout = layers.at("TheirDefenseAreaKeepout");
    if(goal_sight.empty() || ball_sight.empty() || def_area_keepout.empty()) {
      return;
    }
    cv::Mat mask = goal_sight & ball_sight & def_area_keepout;
    cv::Mat scores = layers.at("DistanceDownField").mul(layers.at("DistanceFromTheirBots"));
    map = cv::Scalar{0};
    scores.copyTo(map, mask);
  }

};

} // namespace ateam_kenobi::spatial::maps


#endif  // MAPS__TEST_MAP_HPP_
