
// TODO this was all pointless as I kinda knew it was unless vision ufunc was not called vision and was flexibly type under an interface not really worth any of this
// really I just wanted writing strategy to be flexible but this is not I should have made a generic writer class for any ros message that could come out of the logs
namespace ateam_data_conversion
{
SslToRosbag::SslToRosbag(std::string input_ssl_filename, std::string out_bag_filename) {
    // TODO Need to make output dir?
    std::unique_ptr<rosbag2_cpp::Writer> writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(out_bag_filename);

    // Should really take a variant of messages able to be written back
    ateam_data_conversion::vision_ufunc_t output_func = [&](vision_ufunc_arg_t& output) -> void {

        const rclcpp::Time& time = output.first;
        // Should have written the overload version I just cant brain about anything anymore
        std::visit([&](auto&& msg) {
            using T = std::decay_t<decltype(msg)>;
            if constexpr (std::is_same_v<T, ateam_data_conversion::FilterOutput>) {
                writer->write(msg.ball, Topic::kBall, "ateam_msgs/msg/ball_state", time);
                for (std::size_t id = 0; id < 16; id++) {
                    writer->write(msg.blue_robots.at(id), std::string(Topic::kBlueTeamRobotPrefix) + std::to_string(id), "ateam_msgs/msg/robot_state", time);
                }
                for (std::size_t id = 0; id < 16; id++) {
                    writer->write(msg.yellow_robots.at(id), std::string(Topic::kYellowTeamRobotPrefix) + std::to_string(id), "ateam_msgs/msg/robot_state", time);
                }
            } else if constexpr (std::is_same_v<T, ssl_league_msgs::msg::Referee>) {
                writer->write(msg, Topic::kRefereeMessages, "ssl_league_msgs/msg/referee", time);
            }
        }, output.second);

    };
    ateam_data_conversion::SslVisionAdapter vision_filter(output_func);
    ateam_data_conversion::SslLogReplayer player(input_ssl_filename, std::bind(&ateam_data_conversion::SslVisionAdapter, &vision_filter, std::placeholders::_1));
}
} // namespace