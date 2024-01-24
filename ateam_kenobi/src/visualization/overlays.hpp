#ifndef OVERLAYS_HPP_
#define OVERLAYS_HPP_

#include <string>
#include <ateam_msgs/msg/overlay_array.hpp>
#include <ateam_geometry/types.hpp>

namespace ateam_kenobi::visualization
{

class Overlays
{
public:
  Overlays() = default;

  explicit Overlays(std::string ns, ateam_msgs::msg::OverlayArray::SharedPtr msg = nullptr);

  Overlays getChild(std::string name);

  const ateam_msgs::msg::OverlayArray & getMsg() const;

  void clear();

  void drawLine(
    const std::string & name, const std::vector<ateam_geometry::Point> & points,
    const std::string & color = "white", const uint8_t stroke_width = 5,
    const uint32_t lifetime = kDefaultLifetime);

  void drawCircle(
    const std::string & name, const ateam_geometry::Circle & circle,
    const std::string & stroke_color = "white",
    const std::string & fill_color = "#FFFFFF7F", const uint8_t stroke_width = 5,
    const uint32_t lifetime = 100);

  void drawPolygon(
    const std::string & name, const ateam_geometry::Polygon & polygon,
    const std::string & stroke_color = "white",
    const std::string & fill_color = "#FFFFFF7F", const uint8_t stroke_width = 5,
    const uint32_t lifetime = kDefaultLifetime);

  void drawText(
    const std::string & name, const std::string & text, const ateam_geometry::Point & position,
    const std::string & color = "white", const uint8_t font_size = 12,
    const uint32_t lifetime = kDefaultLifetime);

  void drawRectangle(
    const std::string & name, const ateam_geometry::Rectangle & rectangle,
    const std::string & stroke_color = "white", const std::string & fill_color = "#FFFFFF7F",
    const uint8_t stroke_width = 5, const uint32_t lifetime = kDefaultLifetime);

private:
  static const uint32_t kDefaultLifetime = 200;
  std::string ns_;
  ateam_msgs::msg::OverlayArray::SharedPtr overlay_array_;

  void addOverlay(ateam_msgs::msg::Overlay overlay);
};

}  // namespace ateam_kenobi::visualization

#endif  // OVERLAYS_HPP_
