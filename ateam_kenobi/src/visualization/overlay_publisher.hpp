#ifndef VISUALIZATION__OVERLAY_PUBLISHER_HPP_
#define VISUALIZATION__OVERLAY_PUBLISHER_HPP_

#include <rclcpp/node.hpp>
#include <ateam_msgs/msg/overlay.hpp>
#include <ateam_msgs/msg/play_info.hpp>
#include <ateam_geometry/types.hpp>

namespace ateam_kenobi::visualization
{

class OverlayPublisher
{
public:
  explicit OverlayPublisher(const std::string & ns, rclcpp::Node & node);

  void drawLine(
    const std::string & name, const std::vector<ateam_geometry::Point> & points,
    const std::string & color = "white", const uint8_t stroke_width = 5,
    const uint32_t lifetime = 100);

  void drawCircle(
    const std::string & name, const ateam_geometry::Circle & circle,
    const std::string & stroke_color = "white",
    const std::string & fill_color = "#FFFFFF7F", const uint8_t stroke_width = 5,
    const uint32_t lifetime = 100);

  void drawPolygon(
    const std::string & name, const ateam_geometry::Polygon & polygon,
    const std::string & stroke_color = "white",
    const std::string & fill_color = "#FFFFFF7F", const uint8_t stroke_width = 5,
    const uint32_t lifetime = 100);

  void drawText(
    const std::string & name, const std::string & text, const ateam_geometry::Point & position,
    const std::string & color = "white", const uint8_t font_size = 12,
    const uint32_t lifetime = 100);

  /**
   * @brief Publishes and clears the current buffer of overlays.
   * @note Plays should NOT call this function. It will be called for you at the end of the frame.
   */
  void publishOverlays();

private:
  const std::string ns;
  ateam_msgs::msg::OverlayArray overlays_;
  rclcpp::Publisher<ateam_msgs::msg::OverlayArray>::SharedPtr publisher_;
};

}  // namespace ateam_kenobi::visualization

#endif  // VISUALIZATION__OVERLAY_PUBLISHER_HPP_
