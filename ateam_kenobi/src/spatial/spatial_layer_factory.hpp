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

#include <ateam_geometry/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include "types/field.hpp"
#include "types/world.hpp"

namespace ateam_kenobi::spatial
{

class SpatialLayerFactory {
public:
  SpatialLayerFactory(std::string name) : name_(name) {}

  const std::string & GetName() const {
    return name_;
  }

  void Build(cv::Mat & layer, const World & world) {
    field_ = world.field;
    FillLayer(layer, world);
  }

protected:

  /**
   * @brief Sets layer dimensions and ensures data is allocated. Reinitializes data if field
   * dimensions change.
   */
  void SetupLayer(cv::Mat & layer, int layer_type) {
    const cv::Size size(LayerWidth(), LayerHeight());
    if(layer.empty() || layer.type() != layer_type) {
      layer = cv::Mat(size, layer_type);
    }
    if(layer.size() != size) {
      cv::resize(layer, layer, size);
    }
  }

  float Resolution() {
    return kResolution;
  }

  float WorldWidth() {
    return field_.field_length + (2.0 * field_.boundary_width);
  }

  float WorldHeight() {
    return field_.field_width + (2.0 * field_.boundary_width);
  }

  int LayerWidth() {
    return WorldWidth() / kResolution;
  }

  int LayerHeight() {
    return WorldHeight() / kResolution;
  }

  float LayerToWorldX(const int x) {
    return (x * kResolution) - (WorldWidth()/2.0);
  }

  float LayerToWorldY(const int y) {
    return ((LayerHeight() - y) * kResolution) - (WorldHeight()/2.0);
  }

  cv::Point2d LayerToWorld(const cv::Point2i & p) {
    return cv::Point2d(LayerToWorldX(p.x), LayerToWorldY(p.y));
  }

  int WorldToLayerX(const float x) {
    return (x + (WorldWidth() / 2.0)) / kResolution;
  }

  int WorldToLayerY(const float y) {
    return (-y + (WorldHeight() / 2.0)) / kResolution;
  }

  cv::Point2i WorldToLayer(const cv::Point2d & p) {
    return cv::Point2i(WorldToLayerX(p.x), WorldToLayerY(p.y));
  }

  cv::Point2i WorldToLayer(const ateam_geometry::Point & p) {
    return WorldToLayer(cv::Point2d(p.x(), p.y()));
  }

  int WorldToLayerDist(const double d) {
    return static_cast<int>(d / kResolution);
  }

  virtual void FillLayer(cv::Mat & layer, const World & world) = 0;

private:
  const float kResolution = 0.05;  // meters per pixel
  const std::string name_;
  Field field_;

};

}  // namespace ateam_kenobi::spatial

#endif  // SPATIAL__SPATIAL_LAYER_FACTORY_HPP_
