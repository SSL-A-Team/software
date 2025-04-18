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

#ifndef CORE__SPATIAL__MAPS__LINE_OF_SIGHT_BALL_MAP_HPP_
#define CORE__SPATIAL__MAPS__LINE_OF_SIGHT_BALL_MAP_HPP_

#include <string>
#include <unordered_map>
#include "core/spatial/spatial_map_factory.hpp"

namespace ateam_kenobi::spatial::maps
{

class LineOfSightBallMap final : public SpatialMapFactory {
public:
  LineOfSightBallMap()
  : SpatialMapFactory("LineOfSightBallMap") {}

  void FillMap(
    cv::Mat & map, const World &,
    const std::unordered_map<std::string, cv::Mat> & layers) override
  {
    map = layers.at("LineOfSightBall");
  }
};

}  // namespace ateam_kenobi::spatial::maps

#endif  // CORE__SPATIAL__MAPS__LINE_OF_SIGHT_BALL_MAP_HPP_
