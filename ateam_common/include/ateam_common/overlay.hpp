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

#ifndef ATEAM_COMMON__OVERLAY_HPP_
#define ATEAM_COMMON__OVERLAY_HPP_

#include <Eigen/Dense>

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <ateam_msgs/msg/overlay.hpp>

namespace ateam_common
{
class Overlay
{
public:
  static Overlay & GetOverlay()
  {
    static Overlay o;
    return o;
  }

  void SetOverlayPublishCallback(std::function<void(ateam_msgs::msg::Overlay)> cb)
  {
    this->cb = cb;
  }

  void SetNamespace(const std::string & ns)
  {
    this->ns = ns;
  }

  void DrawLine(const std::vector<Eigen::Vector2d> & line)
  {
    ateam_msgs::msg::Overlay overlay_msg;
    overlay_msg.ns = ns;
    overlay_msg.name = "trajectory";
    overlay_msg.visible = true;
    overlay_msg.type = ateam_msgs::msg::Overlay::LINE;
    overlay_msg.command = ateam_msgs::msg::Overlay::REPLACE;
    overlay_msg.position.x = 0;
    overlay_msg.position.y = 0;
    overlay_msg.stroke_color = "#FF00FF88";
    overlay_msg.lifetime = 10;

    for (const auto & pt : line) {
      geometry_msgs::msg::Point p;
      p.x = pt.x();
      p.y = pt.y();
      overlay_msg.points.push_back(p);
    }

    cb(overlay_msg);
  }

private:
  std::function<void(ateam_msgs::msg::Overlay)> cb;
  std::string ns = "";
};
}  // namespace ateam_common

#endif  // ATEAM_COMMON__OVERLAY_HPP_
