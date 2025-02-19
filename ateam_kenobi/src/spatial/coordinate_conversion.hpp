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

#ifndef SPATIAL__COORDINATE_CONVERSION_HPP_
#define SPATIAL__COORDINATE_CONVERSION_HPP_

#include <opencv2/core/mat.hpp>
#include "types/field.hpp"

namespace ateam_kenobi::spatial
{

static const double kResolution = 0.05;  // m/px

inline float WorldWidth(const Field & field)
{
  return field.field_length + (2.0 * field.boundary_width);
}

inline float WorldHeight(const Field & field)
{
  return field.field_width + (2.0 * field.boundary_width);
}

inline int LayerWidth(const Field & field)
{
  return WorldWidth(field) / kResolution;
}

inline int LayerHeight(const Field & field)
{
  return WorldHeight(field) / kResolution;
}

inline float LayerToWorldX(const int x, const Field & field)
{
  return (x * kResolution) - (WorldWidth(field) / 2.0);
}

inline float LayerToWorldY(const int y, const Field & field)
{
  return ((LayerHeight(field) - y) * kResolution) - (WorldHeight(field) / 2.0);
}

inline cv::Point2d LayerToWorld(const cv::Point2i & p, const Field & field)
{
  return cv::Point2d(LayerToWorldX(p.x, field), LayerToWorldY(p.y, field));
}

inline int WorldToLayerX(const float x, const Field & field)
{
  return (x + (WorldWidth(field) / 2.0)) / kResolution;
}

inline int WorldToLayerY(const float y, const Field & field)
{
  return (-y + (WorldHeight(field) / 2.0)) / kResolution;
}

inline cv::Point2i WorldToLayer(const cv::Point2d & p, const Field & field)
{
  return cv::Point2i(WorldToLayerX(p.x, field), WorldToLayerY(p.y, field));
}

inline cv::Point2i WorldToLayer(const ateam_geometry::Point & p, const Field & field)
{
  return WorldToLayer(cv::Point2d(p.x(), p.y()), field);
}

inline int WorldToLayerDist(const double d)
{
  return static_cast<int>(d / kResolution);
}

}  // namespace ateam_kenobi::spatial

#endif  // SPATIAL__COORDINATE_CONVERSION_HPP_
