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

#ifndef SPATIAL__SPATIAL_LAYER_FACTORY_HPP_
#define SPATIAL__SPATIAL_LAYER_FACTORY_HPP_

#include <string>
#include <ateam_geometry/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include "types/field.hpp"
#include "types/world.hpp"
#include "coordinate_conversion.hpp"

namespace ateam_kenobi::spatial
{

class SpatialLayerFactory {
public:
  explicit SpatialLayerFactory(std::string name)
  : name_(name) {}

  const std::string & GetName() const
  {
    return name_;
  }

  void Build(cv::Mat & layer, const World & world)
  {
    field_ = world.field;
    FillLayer(layer, world);
  }

protected:
  /**
   * @brief Sets layer dimensions and ensures data is allocated. Reinitializes data if field
   * dimensions change.
   */
  void SetupLayer(cv::Mat & layer, int layer_type)
  {
    const cv::Size size(LayerWidth(), LayerHeight());
    if(layer.empty() || layer.type() != layer_type) {
      layer = cv::Mat(size, layer_type);
    }
    if(layer.size() != size) {
      cv::resize(layer, layer, size);
    }
  }

  float WorldWidth()
  {
    return spatial::WorldWidth(field_);
  }

  float WorldHeight()
  {
    return spatial::WorldHeight(field_);
  }

  int LayerWidth()
  {
    return spatial::LayerWidth(field_);
  }

  int LayerHeight()
  {
    return spatial::LayerHeight(field_);
  }

  float LayerToWorldX(const int x)
  {
    return spatial::LayerToWorldX(x, field_);
  }

  float LayerToWorldY(const int y)
  {
    return spatial::LayerToWorldY(y, field_);
  }

  cv::Point2d LayerToWorld(const cv::Point2i & p)
  {
    return spatial::LayerToWorld(p, field_);
  }

  int WorldToLayerX(const float x)
  {
    return spatial::WorldToLayerX(x, field_);
  }

  int WorldToLayerY(const float y)
  {
    return spatial::WorldToLayerY(y, field_);
  }

  cv::Point2i WorldToLayer(const cv::Point2d & p)
  {
    return spatial::WorldToLayer(p, field_);
  }

  cv::Point2i WorldToLayer(const ateam_geometry::Point & p)
  {
    return spatial::WorldToLayer(p, field_);
  }

  virtual void FillLayer(cv::Mat & layer, const World & world) = 0;

private:
  const std::string name_;
  Field field_;
};

}  // namespace ateam_kenobi::spatial

#endif  // SPATIAL__SPATIAL_LAYER_FACTORY_HPP_
