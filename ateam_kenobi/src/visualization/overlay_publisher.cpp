// Copyright 2021 A Team
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


#include "overlay_publisher.hpp"
#include <algorithm>
#include <numeric>

namespace ateam_kenobi::visualization
{

OverlayPublisher::OverlayPublisher(const std::string & ns, rclcpp::Node & node)
: ns(ns),
  publisher_(node.create_publisher<ateam_msgs::msg::OverlayArray>(
      "/overlays",
      rclcpp::SystemDefaultsQoS()))
{
}

void OverlayPublisher::drawLine(
  const std::string & name,
  const std::vector<ateam_geometry::Point> & points,
  const std::string & color, const uint8_t stroke_width,
  const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns;
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
  overlays_.overlays.push_back(msg);
}

void OverlayPublisher::drawCircle(
  const std::string & name, const ateam_geometry::Circle & circle,
  const std::string & stroke_color, const std::string & fill_color, const uint8_t stroke_width,
  const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns;
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
  overlays_.overlays.push_back(msg);
}

void OverlayPublisher::drawPolygon(
  const std::string & name,
  const ateam_geometry::Polygon & polygon,
  const std::string & stroke_color, const std::string & fill_color,
  const uint8_t stroke_width, const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns;
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
  overlays_.overlays.push_back(msg);
}

void OverlayPublisher::drawText(
  const std::string & name, const std::string & text, const ateam_geometry::Point & position,
  const std::string & color, const uint8_t font_size,
  const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns;
  msg.name = name;
  msg.visible = true;
  msg.type = ateam_msgs::msg::Overlay::TEXT;
  msg.command = ateam_msgs::msg::Overlay::REPLACE;
  msg.position.x = position.x();
  msg.position.y = position.y();
  msg.stroke_color = color;
  msg.stroke_width = font_size;
  msg.lifetime = lifetime;
  msg.depth = 1;
  msg.text = text;
  overlays_.overlays.push_back(msg);
}

void OverlayPublisher::drawRectangle(const std::string &name, const ateam_geometry::Rectangle &rectangle, const std::string &stroke_color, const std::string &fill_color, const uint8_t stroke_width, const uint32_t lifetime)
{
  ateam_msgs::msg::Overlay msg;
  msg.ns = ns;
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
  overlays_.overlays.push_back(msg);
}

void OverlayPublisher::publishOverlays() {
  if (!overlays_.overlays.empty()) { publisher_->publish(overlays_); }
  overlays_.overlays.clear();
}

} // namespace ateam_kenobi::visualization
