#include <ateam_data_conversion/ssl_vision_adapter.hpp>

#include <optional>

namespace ateam_data_conversion
{

void SslVisionAdapter::process_message(LogMessage m) {
    if(next_cb_time_ == 0) {
        // set initial callback time based off first message
        next_cb_time_ = m.header.timestamp_ns + TIMER_PERIOD_NS;
    } else if (m.header.timestamp > next_cb_time_) {
        // generate and write prediction if that timestamp has passed
        generate_prediction_();
    }

    const DataHeader& header = m.header;
    std::visit([&](auto&& msg) {
        using T = std::decay_t<decltype(msg)>;

        if constexpr (std::is_same_v<T, Referee>) {
            // if ref message
            auto ref_msg = ateam_game_controller_bridge::message_conversions::fromProto(msg);
            // TODO I shouldnt dynamic allocate here
            game_controller_listener_.RefereeMessageCallback(std::make_shared<ssl_league_msgs::msg::Referee>(ref_msg);
            ufunc_({ros_time_from_timestamp(next_cb_time_), ref_msg});

        } else if constexpr (std::is_same_v<T, SSL_WrapperPacket>) {
            // if vision message
            // do ssl to ros conversion (because lazy)
            auto vision_wrapper_msg = ateam_ssl_vision_bridge::message_conversions::fromProto(msg);

            // do vision filter sample append
            int camera_id = vision_wrapper_msg->detection.camera_id;
            CameraMeasurement camera_measurement = ateam_vision_filter::message_conversions::getCameraMeasurement(vision_wrapper_msg);
            ateam_msgs::msg::FieldInfo field_msg = ateam_vision_filter::message_conversions::getFieldGeometry(vision_wrapper_msg);

            // Our field convention is we should always been on the negative half.
            // So if this is positive for our team we should invert coords
            if (game_controller_listener_.GetTeamSide() == ateam_common::TeamSide::PositiveHalf) {
                camera_measurement.invert();
                ateam_vision_filter::message_conversions::invert_field_info(field_msg);
            }

            // this is what triggers the sample update
            // https://github.com/SSL-A-Team/software/blob/047e0e52e05afce5211ad9764914f381efff14df/ateam_vision_filter/src/camera.cpp#L45
            world_.update_camera(camera_id, camera_measurement);
        }
    }, m.msg);
}

rclcpp::Time SslVisionAdapter::ros_time_from_timestamp(uint64_t timestamp) {
    return rclcpp::Time(timestamp / CONVERSION_CONSTANT, timestamp % CONVERSION_CONSTANT);
}

void SslVisionAdapter::generate_prediction_() {
    FilterOutput out;

    world_.predict();

    std::optional<Ball> maybe_ball = world_.get_ball_estimate();
    out.ball = ateam_vision_filter::message_conversions::toMsg(maybe_ball);

    std::array<std::optional<Robot>, 16> blue_team_robots = world_.get_blue_robots_estimate();
    for (std::size_t id = 0; id < 16; id++) {
        const auto& maybe_robot = blue_team_robots.at(id);
        out.blue_robots.at(id) = ateam_vision_filter::message_conversions::toMsg(maybe_robot);
    }

    std::array<std::optional<Robot>, 16> yellow_team_robots = world_.get_yellow_robots_estimate();
    for (std::size_t id = 0; id < 16; id++) {
        const auto& maybe_robot = yellow_team_robots.at(id);
        out.yellow_robots.at(id) = ateam_vision_filter::message_conversions::toMsg(maybe_robot);
    }

    // call user function with output
    ufunc_({ros_time_from_timestamp(next_cb_time_), out});

    // update next cb time
    next_cb_time_ = m.header.timestamp_ns + TIMER_PERIOD_NS;
}
} // ateam_data_conversion