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


#include "overlays.hpp"
#include <algorithm>
#include <memory>
#include <opencv2/opencv.hpp>

namespace ateam_kenobi::visualization
{

Overlays::Overlays(std::string ns, ateam_msgs::msg::OverlayArray::SharedPtr msg)
: ns_(ns), overlay_array_(msg)
{
  if (!overlay_array_) {
    overlay_array_ = std::make_shared<ateam_msgs::msg::OverlayArray>();
  }
}

Overlays Overlays::getChild(std::string name)
{
  return Overlays(ns_ + "/" + name, overlay_array_);
}


const ateam_msgs::msg::OverlayArray & Overlays::getMsg() const
{
  if (!overlay_array_) {
    throw std::runtime_error(
            "Cannot call getMsg() on a default-intializated instance of Overlays.");
  }
  return *overlay_array_;
}

void Overlays::clear()
{
  if (overlay_array_) {
    overlay_array_->overlays.clear();
  }
}

void Overlays::drawLine(
  const std::string & name, const std::vector<ateam_geometry::Point> & points,
  const std::string & color, const uint8_t stroke_width,
  const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns_;
  msg.name = name;
  msg.visible = true;
  msg.type = ateam_msgs::msg::Overlay::LINE;
  msg.command = ateam_msgs::msg::Overlay::REPLACE;
  msg.position.x = 0.0;
  msg.position.y = 0.0;
  msg.stroke_color = color;
  msg.stroke_width = stroke_width;
  msg.lifetime = lifetime;
  msg.depth = 1;
  std::ranges::transform(
    points, std::back_inserter(msg.points), [](const auto & point) {
      geometry_msgs::msg::Point point_msg;
      point_msg.x = point.x();
      point_msg.y = point.y();
      return point_msg;
    });
  addOverlay(msg);
}

void Overlays::drawCircle(
  const std::string & name, const ateam_geometry::Circle & circle,
  const std::string & stroke_color,
  const std::string & fill_color, const uint8_t stroke_width,
  const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns_;
  msg.name = name;
  msg.visible = true;
  msg.type = ateam_msgs::msg::Overlay::ELLIPSE;
  msg.command = ateam_msgs::msg::Overlay::REPLACE;
  msg.position.x = circle.center().x();
  msg.position.y = circle.center().y();
  msg.scale.x = 2.0 * std::sqrt(circle.squared_radius());
  msg.scale.y = msg.scale.x;
  msg.stroke_color = stroke_color;
  msg.stroke_width = stroke_width;
  msg.fill_color = fill_color;
  msg.lifetime = lifetime;
  msg.depth = 1;
  addOverlay(msg);
}

void Overlays::drawPolygon(
  const std::string & name, const ateam_geometry::Polygon & polygon,
  const std::string & stroke_color,
  const std::string & fill_color, const uint8_t stroke_width,
  const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns_;
  msg.name = name;
  msg.visible = true;
  msg.type = ateam_msgs::msg::Overlay::POLYGON;
  msg.command = ateam_msgs::msg::Overlay::REPLACE;
  msg.position.x = 0.0;
  msg.position.y = 0.0;
  msg.stroke_color = stroke_color;
  msg.stroke_width = stroke_width;
  msg.fill_color = fill_color;
  msg.lifetime = lifetime;
  msg.depth = 1;
  std::transform(
    polygon.vertices_begin(), polygon.vertices_end(), std::back_inserter(msg.points),
    [](const auto & vertex) {
      geometry_msgs::msg::Point point_msg;
      point_msg.x = vertex.x();
      point_msg.y = vertex.y();
      return point_msg;
    });
  addOverlay(msg);
}

void Overlays::drawText(
  const std::string & name, const std::string & text, const ateam_geometry::Point & position,
  const std::string & color, const uint8_t font_size,
  const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns_;
  msg.name = name;
  msg.visible = true;
  msg.type = ateam_msgs::msg::Overlay::TEXT;
  msg.command = ateam_msgs::msg::Overlay::REPLACE;
  msg.position.x = position.x();
  msg.position.y = position.y();
  msg.fill_color = color;
  msg.stroke_width = font_size;
  msg.lifetime = lifetime;
  msg.depth = 1;
  msg.text = text;
  addOverlay(msg);
}

void Overlays::drawRectangle(
  const std::string & name, const ateam_geometry::Rectangle & rectangle,
  const std::string & stroke_color, const std::string & fill_color,
  const uint8_t stroke_width, const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns_;
  msg.name = name;
  msg.visible = true;
  msg.type = ateam_msgs::msg::Overlay::RECTANGLE;
  msg.command = ateam_msgs::msg::Overlay::REPLACE;
  msg.position.x = std::midpoint(rectangle.xmin(), rectangle.xmax());
  msg.position.y = std::midpoint(rectangle.ymin(), rectangle.ymax());
  msg.scale.x = rectangle.xmax() - rectangle.xmin();
  msg.scale.y = rectangle.ymax() - rectangle.ymin();
  msg.stroke_color = stroke_color;
  msg.stroke_width = stroke_width;
  msg.fill_color = fill_color;
  msg.depth = 1;
  msg.lifetime = lifetime;
  addOverlay(msg);
}

void Overlays::drawArc(
  const std::string & name, const ateam_geometry::Arc & arc,
  const std::string & stroke_color, const uint8_t stroke_width,
  const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns_;
  msg.name = name;
  msg.visible = true;
  msg.type = ateam_msgs::msg::Overlay::ARC;
  msg.command = ateam_msgs::msg::Overlay::REPLACE;
  msg.position.x = arc.center().x();
  msg.position.y = arc.center().y();
  msg.scale.x = 2.0 * arc.radius();
  msg.scale.y = msg.scale.x;
  msg.start_angle = std::atan2(arc.start().dy(), arc.start().dx());
  msg.end_angle = std::atan2(arc.end().dy(), arc.end().dx());
  msg.stroke_color = stroke_color;
  msg.stroke_width = stroke_width;
  msg.lifetime = lifetime;
  msg.depth = 1;
  addOverlay(msg);
}

void Overlays::drawHeatmap(
  const std::string & name, const ateam_geometry::Rectangle & bounds,
  const cv::Mat & data, const uint8_t alpha, const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns_;
  msg.name = name;
  msg.visible = true;
  msg.type = ateam_msgs::msg::Overlay::HEATMAP;
  msg.command = ateam_msgs::msg::Overlay::REPLACE;
  msg.lifetime = lifetime;
  msg.position.x = std::midpoint(bounds.xmin(), bounds.xmax());
  msg.position.y = std::midpoint(bounds.ymin(), bounds.ymax());
  msg.scale.x = bounds.xmax() - bounds.xmin();
  msg.scale.y = bounds.ymax() - bounds.ymin();
  msg.heatmap_resolution_width = data.cols;
  msg.heatmap_resolution_height = data.rows;
  msg.heatmap_alpha = {alpha};
  msg.heatmap_data.reserve(data.rows * data.cols);
  if(!data.empty()) {
    switch(data.type()) {
      case CV_8UC1:
        std::copy(data.begin<uint8_t>(), data.end<uint8_t>(), std::back_inserter(msg.heatmap_data));
        break;
      case CV_32FC1:
        {
          cv::Mat byte_data{data.rows, data.cols, CV_8UC1};
          cv::normalize(data, byte_data, 0, 255, cv::NORM_MINMAX, CV_8UC1);
          std::copy(byte_data.begin<uint8_t>(), byte_data.end<uint8_t>(),
            std::back_inserter(msg.heatmap_data));
          break;
        }
      default:
        throw std::runtime_error("Unsupported heatmap data type: " + std::to_string(data.type()));
    }
  }
  addOverlay(msg);
}

void Overlays::drawHeatmap(
  const std::string & name, const ateam_geometry::Rectangle & bounds,
  const cv::Mat & data, const cv::Mat & alpha, const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns_;
  msg.name = name;
  msg.visible = true;
  msg.type = ateam_msgs::msg::Overlay::HEATMAP;
  msg.command = ateam_msgs::msg::Overlay::REPLACE;
  msg.lifetime = lifetime;
  msg.position.x = std::midpoint(bounds.xmin(), bounds.xmax());
  msg.position.y = std::midpoint(bounds.ymin(), bounds.ymax());
  msg.scale.x = bounds.xmax() - bounds.xmin();
  msg.scale.y = bounds.ymax() - bounds.ymin();
  msg.heatmap_resolution_width = data.cols;
  msg.heatmap_resolution_height = data.rows;
  msg.heatmap_alpha.reserve(alpha.rows * alpha.cols);
  if(!alpha.empty()) {
    switch(alpha.type()) {
      case CV_8UC1:
        std::copy(alpha.begin<uint8_t>(), alpha.end<uint8_t>(),
          std::back_inserter(msg.heatmap_alpha));
        break;
      case CV_32FC1:
        {
          cv::Mat byte_alpha{alpha.rows, alpha.cols, CV_8UC1};
          cv::normalize(alpha, byte_alpha, 0, 255, cv::NORM_MINMAX, CV_8UC1);
          std::copy(byte_alpha.begin<uint8_t>(), byte_alpha.end<uint8_t>(),
            std::back_inserter(msg.heatmap_alpha));
          break;
        }
      default:
        throw std::runtime_error("Unsupported heatmap alpha type: " + std::to_string(data.type()));
    }
  }
  msg.heatmap_data.reserve(data.rows * data.cols);
  if(!data.empty()) {
    switch(data.type()) {
      case CV_8UC1:
        std::copy(data.begin<uint8_t>(), data.end<uint8_t>(), std::back_inserter(msg.heatmap_data));
        break;
      case CV_32FC1:
        {
          cv::Mat byte_data{data.rows, data.cols, CV_8UC1};
          cv::normalize(data, byte_data, 0, 255, cv::NORM_MINMAX, CV_8UC1);
          std::copy(byte_data.begin<uint8_t>(), byte_data.end<uint8_t>(),
            std::back_inserter(msg.heatmap_data));
          break;
        }
      default:
        throw std::runtime_error("Unsupported heatmap data type: " + std::to_string(data.type()));
    }
  }
  addOverlay(msg);
}


void Overlays::drawHeatmap(
  const std::string & name, const ateam_geometry::Rectangle & bounds,
  const std::vector<uint8_t> & data, const std::size_t resolution_width,
  const std::size_t resolution_height, const uint8_t alpha, const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns_;
  msg.name = name;
  msg.visible = true;
  msg.type = ateam_msgs::msg::Overlay::HEATMAP;
  msg.command = ateam_msgs::msg::Overlay::REPLACE;
  msg.lifetime = lifetime;
  msg.position.x = std::midpoint(bounds.xmin(), bounds.xmax());
  msg.position.y = std::midpoint(bounds.ymin(), bounds.ymax());
  msg.scale.x = bounds.xmax() - bounds.xmin();
  msg.scale.y = bounds.ymax() - bounds.ymin();
  msg.heatmap_resolution_width = resolution_width;
  msg.heatmap_resolution_height = resolution_height;
  msg.heatmap_alpha = {alpha};
  msg.heatmap_data = data;
  addOverlay(msg);
}

void Overlays::addOverlay(ateam_msgs::msg::Overlay overlay)
{
  if (overlay_array_) {
    overlay_array_->overlays.push_back(overlay);
  }
}

}  // namespace ateam_kenobi::visualization
