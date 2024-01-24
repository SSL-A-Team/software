#include "overlays.hpp"

namespace ateam_kenobi::visualization
{

Overlays::Overlays(std::string ns, ateam_msgs::msg::OverlayArray::SharedPtr msg)
: ns_(ns), overlay_array_(msg)
{
  if(!overlay_array_) {
    overlay_array_ = std::make_shared<ateam_msgs::msg::OverlayArray>();
  }
}

Overlays Overlays::getChild(std::string name)
{
  return Overlays(ns_ + "/" + name, overlay_array_);
}


const ateam_msgs::msg::OverlayArray & Overlays::getMsg() const
{
  return *overlay_array_;
}

void Overlays::clear()
{
  overlay_array_->overlays.clear();
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
  msg.stroke_color = color;
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

void Overlays::addOverlay(ateam_msgs::msg::Overlay overlay)
{
  if (overlay_array_) {
    overlay_array_->overlays.push_back(overlay);
  }
}

}  // namespace ateam_kenobi::visualization
