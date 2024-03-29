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


#ifndef PLAY_HELPERS__WINDOW_EVALUATION_HPP_
#define PLAY_HELPERS__WINDOW_EVALUATION_HPP_

#include <optional>
#include <vector>
#include <ateam_geometry/types.hpp>
#include "types/robot.hpp"
#include "visualization/overlays.hpp"

namespace ateam_kenobi::play_helpers::window_evaluation
{

std::vector<ateam_geometry::Segment> getWindows(
  const ateam_geometry::Segment & target,
  const ateam_geometry::Point & source,
  const std::vector<Robot> & robots);

std::optional<ateam_geometry::Segment> getLargestWindow(
  const std::vector<ateam_geometry::Segment> & windows);

std::optional<ateam_geometry::Segment> projectRobotShadowOntoLine(
  const Robot & robot,
  const ateam_geometry::Point & source,
  const ateam_geometry::Line & line);

void removeSegmentFromWindows(
  const ateam_geometry::Segment & seg,
  std::vector<ateam_geometry::Segment> & windows);

void drawWindows(
  const std::vector<ateam_geometry::Segment> & windows,
  const ateam_geometry::Point & source, visualization::Overlays overlays_);

}  // namespace ateam_kenobi::play_helpers::window_evaluation

#endif  // PLAY_HELPERS__WINDOW_EVALUATION_HPP_
