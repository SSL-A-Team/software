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

#ifndef ATEAM_SPATIAL__COORDINATE_CONVERSIONS_HPP_
#define ATEAM_SPATIAL__COORDINATE_CONVERSIONS_HPP_

#include "types.hpp"
#include "cuda_hostdev.hpp"

CUDA_HOSTDEV inline float WorldWidthReal(const FieldDimensions & field)
{
  return field.field_length + (2.0 * field.boundary_width);
}

CUDA_HOSTDEV inline float WorldHeightReal(const FieldDimensions & field)
{
  return field.field_width + (2.0 * field.boundary_width);
}

CUDA_HOSTDEV inline int WorldWidthSpatial(
  const FieldDimensions & field,
  const SpatialSettings & settings)
{
  return WorldWidthReal(field) / settings.resolution;
}

CUDA_HOSTDEV inline int WorldHeightSpatial(
  const FieldDimensions & field,
  const SpatialSettings & settings)
{
  return WorldHeightReal(field) / settings.resolution;
}

CUDA_HOSTDEV inline float SpatialToRealX(
  const int x, const FieldDimensions & field,
  const SpatialSettings & settings)
{
  return (x * settings.resolution) - (WorldWidthReal(field) / 2.0);
}

CUDA_HOSTDEV inline float SpatialToRealY(
  const int y, const FieldDimensions & field,
  const SpatialSettings & settings)
{
  return (y * settings.resolution) - (WorldHeightReal(field) / 2.0);
}

CUDA_HOSTDEV inline int RealToSpatialDist(const float d, const SpatialSettings & settings)
{
  return static_cast<int>(d / settings.resolution);
}

#endif  // ATEAM_SPATIAL__COORDINATE_CONVERSIONS_HPP_
