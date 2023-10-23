#ifndef ATEAM_DATA_CONVERSION__SSL_VISION_ADAPTER_HPP_
#define ATEAM_DATA_CONVERSION__SSL_VISION_ADAPTER_HPP_

#include <variant>
#include <array>
#include <functional>

#include "world.hpp"
#include <message_conversions.hpp>

#include <ateam_common/game_controller_listener.hpp>

#include <ssl_league_msgs/msg/vision_wrapper.hpp>
#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>

#include <ssl_league_msgs/msg/referee.hpp>
#include <ssl_league_protobufs/ssl_gc_referee_message.pb.h>

namespace ateam_data_conversion
{

struct FilterOutput {
    ateam_msgs::msg::BallState ball;
    std::array<ateam_msgs::msg::RobotState>, 16> yellow_robots;
    std::array<ateam_msgs::msg::RobotState>, 16> blue_robots;
};

// May actually be to big to be a variant at the size of 16 robots...
using vision_variant_t = std::variant<FilterOutput, ssl_league_msgs::msg::Referee>;
using vision_ufunc_arg_t = std::pair<rclcpp::Time, vision_variant_t>;
using vision_ufunc_t = std::function<void(vision_ufunc_arg_t)>;

class SslVisionAdapter
{
public: // CONSTEXPRS
    static constexpr uint64_t TIMER_PERIOD_NS = 10'000'000; // 10ms

public: // ctors
    SslVisionAdapter() = delete;
    SslVisionAdapter(vision_ufunc_t ufunc) : ufunc_{ufunc} {};
    // constructor takes functional for what to do with returned world state for each cycle

public: // funcs
    // message processing callback
    void process_message(LogMessage m);
    rclcpp::Time SslVisionAdapter::ros_time_from_timestamp(uint64_t timestamp);

private: // vars
    uint64_t next_cb_time {0};
    ateam_vision_filter::World world_;
    ateam_common::GameControllerListener game_controller_listener_;
    // func to call when the appropriate time passes from incoming messages
    vision_ufunc_t ufunc_;
    // Last timestamp (discard out of order messages)
private: // funcs
    // timer call back
    void generate_prediction_();

};

} // namespace
#endif // ATEAM_DATA_CONVERSION__SSL_VISION_ADAPTER_HPP_