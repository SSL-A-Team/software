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

#include "spatial_inspection.hpp"
#include <opencv2/opencv.hpp>
#include "coordinate_conversion.hpp"

namespace ateam_kenobi::spatial
{

ateam_geometry::Point GetMaxPosition(const SpatialMap & map, const Field & field)
{
  double min_val;  // unused by us, but required by minMaxLoc
  cv::Point max_loc;
  cv::minMaxLoc(map.data, &min_val, nullptr, nullptr, &max_loc);
  cv::Point2d max_loc_world = LayerToWorld(max_loc, field);
  return ateam_geometry::Point{max_loc_world.x, max_loc_world.y};
}

ateam_geometry::Point GetMaxPosition(
  const SpatialMap & map, const Field & field,
  const ateam_geometry::Rectangle & roi)
{
  int row_start = WorldToLayerY(roi.ymin(), field);
  int row_stop = WorldToLayerY(roi.ymax(), field);
  int col_start = WorldToLayerX(roi.xmin(), field);
  int col_stop = WorldToLayerX(roi.xmax(), field);
  cv::Mat roi_data = map.data(cv::Range(row_start, row_stop), cv::Range(col_start, col_stop));
  double min_val;  // unused by us, but required by minMaxLoc
  cv::Point max_loc;
  cv::minMaxLoc(roi_data, &min_val, nullptr, nullptr, &max_loc);
  cv::Point2d max_loc_world = LayerToWorld(max_loc, field);
  return ateam_geometry::Point{max_loc_world.x, max_loc_world.y};
}

} // namespace ateam_kenobi::spatial
