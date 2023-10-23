#include <variant>
#include <array>
#include <optional>
#include <function>

#include "world.hpp"
#include "message_conversions.hpp"

#include <ssl_league_msgs/msg/vision_wrapper.hpp>
#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>

#include <ssl_league_msgs/msg/referee.hpp>
#include <ssl_league_protobufs/ssl_gc_referee_message.pb.h>

namespace ateam_data_conversion
{

struct FilterOutput {
  std::optional<Ball> ball_estimate;
  std::array<std::optional<Robot>, 16> yellow_robots_estimate;
  std::array<std::optional<Robot>, 16> blue_robots_estimate;
}

using vision_ufunc_t = std::function<void(FilterOutput)>;
class SslVisionAdapter
{

public:
// constructor takes functional for what to do with returned world state for each cycle
SslVisionAdapter::SslVisionAdapter(std::string filename, vision_ufunc_t ufunc) : ufunc_{ufunc} {

}

// message processing callback
void process_message(LogMessage m) {

    std::visit([&](auto&& msg) {
        using T = std::decay_t<decltype(msg.msg)>;
        // if ref message
        if constexpr (std::is_same_v<T, Referee>) {
            auto ref_msg = ateam_game_controller_bridge::message_conversions::fromProto(msg.msg);
            // TODO I shouldnt dynamic allocate here
            game_controller_listener_.RefereeMessageCallback(std::make_shared<ssl_league_msgs::msg::Referee>(ref_msg);

        // if vision message
        } else if constexpr (std::is_same_v<T, SSL_WrapperPacket>) {
            // do ssl to ros conversion (because lazy)
            auto vision_wrapper_msg =  ateam_ssl_vision_bridge::message_conversions::fromProto(msg.msg);

            // do vision filter sample append
            int camera_id = vision_wrapper_msg->detection.camera_id;
            CameraMeasurement camera_measurement = message_conversions::getCameraMeasurement(vision_wrapper_msg);
            ateam_msgs::msg::FieldInfo field_msg = message_conversions::getFieldGeometry(vision_wrapper_msg);

            // Our field convention is we should always been on the negative half.
            // So if this is positive for our team we should invert coords
            if (game_controller_listener_.GetTeamSide() == ateam_common::TeamSide::PositiveHalf) {
              camera_measurement.invert();
              message_conversions::invert_field_info(field_msg);
            }

            // this is what triggers the sample update
            // https://github.com/SSL-A-Team/software/blob/047e0e52e05afce5211ad9764914f381efff14df/ateam_vision_filter/src/camera.cpp#L45
            world_.update_camera(camera_id, camera_measurement);
        }

    }, m);
}

private:
ateam_vision_filter::World world_;
ateam_common::GameControllerListener game_controller_listener_;
// func to call when the appropriate time passes from incoming messages
vision_ufunc_t ufunc_;
// Last timestamp (discard out of order messages)


// timer call back
void generate_prediction_(std::variant<>) {
    world_.predict();

    std::optional<Ball> maybe_ball = world_.get_ball_estimate();
    ball_publisher_->publish(message_conversions::toMsg(maybe_ball));

    std::array<std::optional<Robot>, 16> blue_team_robots = world_.get_blue_robots_estimate();
    for (std::size_t id = 0; id < 16; id++) {
      const auto & maybe_robot = blue_team_robots.at(id);
      blue_robots_publisher_.at(id)->publish(message_conversions::toMsg(maybe_robot));
    }

    std::array<std::optional<Robot>, 16> yellow_team_robots = world_.get_yellow_robots_estimate();
    for (std::size_t id = 0; id < 16; id++) {
      const auto & maybe_robot = yellow_team_robots.at(id);
      yellow_robots_publisher_.at(id)->publish(message_conversions::toMsg(maybe_robot));
    }
}