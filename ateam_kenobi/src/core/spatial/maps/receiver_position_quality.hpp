// Copyright 2025 A Team
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

#ifndef CORE__SPATIAL__MAPS__RECEIVER_POSITION_QUALITY_HPP_
#define CORE__SPATIAL__MAPS__RECEIVER_POSITION_QUALITY_HPP_

#include <algorithm>
#include <string>
#include <unordered_map>
#include "core/spatial/spatial_map_factory.hpp"
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::spatial::maps
{

class ReceiverPositionQuality final : public SpatialMapFactory {
public:
  ReceiverPositionQuality()
  : SpatialMapFactory("ReceiverPositionQuality") {}

  void FillMap(
    cv::Mat & map, const World & world,
    const std::unordered_map<std::string, cv::Mat> & layers) override
  {
    const auto & goal_sight = layers.at("LineOfSightTheirGoal");
    const auto & ball_sight = layers.at("LineOfSightBall");
    const auto & def_area_keepout = layers.at("TheirDefenseAreaKeepout");
    const auto & in_field = layers.at("InField");
    if(goal_sight.empty() || ball_sight.empty() || def_area_keepout.empty() || in_field.empty()) {
      return;
    }
    cv::Mat mask = goal_sight & ball_sight & def_area_keepout & in_field;
    cv::Mat distance_from_bots = cv::min(layers.at("DistanceFromTheirBots"), cv::Scalar(1.0));
    const auto max_dist_from_edge = 0.75;
    cv::Mat distance_from_edge = cv::min(layers.at("DistanceFromFieldEdge"),
        cv::Scalar(max_dist_from_edge)) / max_dist_from_edge;
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

#endif  // CORE__SPATIAL__MAPS__RECEIVER_POSITION_QUALITY_HPP_
