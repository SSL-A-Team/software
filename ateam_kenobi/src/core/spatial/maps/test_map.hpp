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

#ifndef CORE__SPATIAL__MAPS__TEST_MAP_HPP_
#define CORE__SPATIAL__MAPS__TEST_MAP_HPP_

#include <algorithm>
#include <string>
#include <unordered_map>
#include "core/spatial/spatial_map_factory.hpp"

namespace ateam_kenobi::spatial::maps
{

class TestMap final : public SpatialMapFactory {
public:
  TestMap()
  : SpatialMapFactory("TestMap") {}

  void FillMap(
    cv::Mat & map, const World & world,
    const std::unordered_map<std::string, cv::Mat> & layers) override
  {
    const auto & goal_sight = layers.at("LineOfSightTheirGoal");
    const auto & ball_sight = layers.at("LineOfSightBall");
    const auto & def_area_keepout = layers.at("TheirDefenseAreaKeepout");
    if(goal_sight.empty() || ball_sight.empty() || def_area_keepout.empty()) {
      return;
    }
    cv::Mat mask = goal_sight & ball_sight & def_area_keepout;
    cv::Mat distance_from_bots = cv::min(layers.at("DistanceFromTheirBots"), cv::Scalar(1.0));
    cv::Mat distance_from_edge = cv::min(layers.at("DistanceFromFieldEdge"), cv::Scalar(2.0)) / 2.0;
    cv::Mat scores;
    if(!play_helpers::getVisibleRobots(world.their_robots).empty()) {
      scores = layers.at("DistanceDownField").mul(distance_from_edge).mul(distance_from_bots);
    } else {
      scores = layers.at("DistanceDownField").mul(distance_from_edge);
    }
    map = cv::Scalar{0};
    scores.copyTo(map, mask);
  }
};

}  // namespace ateam_kenobi::spatial::maps


#endif  // CORE__SPATIAL__MAPS__TEST_MAP_HPP_
