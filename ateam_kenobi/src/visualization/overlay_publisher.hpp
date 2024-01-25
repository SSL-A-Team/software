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


#ifndef VISUALIZATION__OVERLAY_PUBLISHER_HPP_
#define VISUALIZATION__OVERLAY_PUBLISHER_HPP_

#include <string>
#include <vector>
#include <rclcpp/node.hpp>
#include <ateam_msgs/msg/overlay_array.hpp>
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

  /**
   * @brief Publishes and clears the current buffer of overlays.
   * @note Plays should NOT call this function. It will be called for you at the end of the frame.
   */
  void publishOverlays();

private:
  static const uint32_t kDefaultLifetime = 200;
  const std::string ns;
  ateam_msgs::msg::OverlayArray overlays_;
  rclcpp::Publisher<ateam_msgs::msg::OverlayArray>::SharedPtr publisher_;
};

}  // namespace ateam_kenobi::visualization

#endif  // VISUALIZATION__OVERLAY_PUBLISHER_HPP_
