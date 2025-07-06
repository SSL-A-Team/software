#include <getopt.h>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <optional>
#include <ranges>
#include <angles/angles.h>
#include <rclcpp/rclcpp.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_geometry/types.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_radio_msgs/msg/basic_telemetry.hpp>
#include <ateam_radio_msgs/msg/extended_telemetry.hpp>
#include <ateam_radio_msgs/msg/connection_status.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


enum class Direction
{
  Pos,
  Neg
};

Direction Invert(Direction dir)
{
  if(dir == Direction::Pos) {
    return Direction::Neg;
  } else {
    return Direction::Pos;
  }
}


struct Options
{
  ateam_common::TeamColor team_color = ateam_common::TeamColor::Blue;
  std::optional<int> robot_id = std::nullopt;
  Direction direction = Direction::Pos;
  double radius = 0.0;
  double rotations = 1.0;
  double max_speed = 2.0;
  double accel = 1.0;

  static Options from_args(int argc, char ** argv)
  {
    Options opts;

    static struct option long_options[] = {
      {"color", required_argument, 0, 'c'},
      {"robot", required_argument, 0, 'r'},
      {"direction", required_argument, 0, 'd'},
      {"radius", required_argument, 0, 'o'},
      {"rotations", required_argument, 0, 'n'},
      {"speed", required_argument, 0, 's'},
      {"accel", required_argument, 0, 'x'},
      {0, 0, 0, 0}
    };

    const char * short_options = "c:r:d:o:n:s:x:";

    int opt_res = 0;
    while ((opt_res = getopt_long(argc, argv, short_options, long_options, nullptr)) != -1) {
      switch(opt_res) {
        case 'c':
          if(std::strcmp(optarg, "yellow") == 0) {
            opts.team_color = ateam_common::TeamColor::Yellow;
          } else if(std::strcmp(optarg, "blue") == 0) {
            opts.team_color = ateam_common::TeamColor::Blue;
          } else {
            throw std::runtime_error("Unrecognized team color value.");
          }
          break;
        case 'r':
          {
            const auto robot_id = std::atoi(optarg);
            if(robot_id < 0 || robot_id > 16) {
              throw std::runtime_error("Invalid robot ID given.");
            }
            opts.robot_id = robot_id;
            break;
          }
        case 'd':
          if(std::strcmp(optarg, "pos") == 0) {
            opts.direction = Direction::Pos;
          } else if(std::strcmp(optarg, "neg") == 0) {
            opts.direction = Direction::Neg;
          } else {
            throw std::runtime_error("Unrecognized direction value.");
          }
          break;
        case 'o':
          opts.radius = std::abs(std::atof(optarg));
          if(opts.radius <= 0.0) {
            throw std::runtime_error("Radius must be a positive value.");
          }
          break;
        case 'n':
          {
            opts.rotations = std::atoi(optarg);
            if(opts.rotations <= 0) {
              throw std::runtime_error("Number of rotations must be a positive value.");
            }
            break;
          }
        case 's':
          opts.max_speed = std::abs(std::atof(optarg));
          break;
        case 'x':
          opts.accel = std::abs(std::atof(optarg));
          break;
        default:
          throw std::runtime_error("Unrecognized option given.");
      }
    }

    return opts;
  }
};


struct Robot
{
  bool connected = false;
  std::optional<ateam_msgs::msg::RobotState> state;
  std::optional<ateam_radio_msgs::msg::BasicTelemetry> feedback;
  std::optional<ateam_radio_msgs::msg::ExtendedTelemetry> motion_feedback;
};


struct DataEntry
{
  double time;
  double command_speed;
  double vision_speed;
  double vision_perp_speed;
  double vision_perp_distance;
  double firmware_speed;
  double firmware_perp_speed;
};


class AngularVelocityBenchmarkNode : public rclcpp::Node
{
public:
  explicit AngularVelocityBenchmarkNode(Options options)
  : Node("angular_velocity_benchmark"), options_(options)
  {
    using ateam_common::indexed_topic_helpers::create_indexed_subscribers;

    field_sub_ = create_subscription<ateam_msgs::msg::FieldInfo>(std::string(Topics::kField), 1,
      std::bind(&AngularVelocityBenchmarkNode::FieldCallback, this, std::placeholders::_1));

    const auto state_topic_prefix = options_.team_color ==
      ateam_common::TeamColor::Blue ? Topics::kBlueTeamRobotPrefix : Topics::kYellowTeamRobotPrefix;
    create_indexed_subscribers<ateam_msgs::msg::RobotState>(
      robot_state_subs_, state_topic_prefix, 1, &AngularVelocityBenchmarkNode::RobotStateCallback,
      this);

    create_indexed_subscribers<ateam_radio_msgs::msg::BasicTelemetry>(robot_feedback_subs_,
      Topics::kRobotFeedbackPrefix, 1, &AngularVelocityBenchmarkNode::RobotFeedbackCallback, this);

    create_indexed_subscribers<ateam_radio_msgs::msg::ExtendedTelemetry>(
      robot_motion_feedback_subs_, Topics::kRobotMotionFeedbackPrefix, 1,
      &AngularVelocityBenchmarkNode::RobotMotionFeedbackCallback, this);

    create_indexed_subscribers<ateam_radio_msgs::msg::ConnectionStatus>(robot_connection_subs_,
      Topics::kRobotConnectionStatusPrefix, 1, &AngularVelocityBenchmarkNode::ConnectionCallback,
      this);

    RCLCPP_INFO(get_logger(), "Waiting for robot...");
    timer_ = create_wall_timer(kTimerDuration,
      std::bind(&AngularVelocityBenchmarkNode::WaitForRobot, this));
  }

private:
  const std::chrono::milliseconds kTimerDuration = std::chrono::milliseconds(10);
  const double kPerpendicularDistanceAllowance = 0.2;

  Options options_;
  std::optional<ateam_msgs::msg::FieldInfo> field_;
  std::array<Robot, 16> robots_;
  std::vector<DataEntry> data_;

  std::chrono::steady_clock::time_point start_time_;
  int robot_id_ = -1;
  ateam_geometry::Point circle_center_;
  double command_speed_ = 0.0;
  double starting_angle_ = 0.0;
  double prev_angle_ = 0.0;
  bool passed_halfway_ = false;
  int rotation_count_ = 0;

  rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<ateam_msgs::msg::FieldInfo>::SharedPtr field_sub_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr, 16> robot_state_subs_;
  std::array<rclcpp::Subscription<ateam_radio_msgs::msg::BasicTelemetry>::SharedPtr,
    16> robot_feedback_subs_;
  std::array<rclcpp::Subscription<ateam_radio_msgs::msg::ExtendedTelemetry>::SharedPtr,
    16> robot_motion_feedback_subs_;
  std::array<rclcpp::Subscription<ateam_radio_msgs::msg::ConnectionStatus>::SharedPtr,
    16> robot_connection_subs_;
  rclcpp::TimerBase::SharedPtr timer_;

  void FieldCallback(const ateam_msgs::msg::FieldInfo::SharedPtr msg)
  {
    field_ = *msg;
  }

  void RobotStateCallback(const ateam_msgs::msg::RobotState::SharedPtr msg, int robot_id)
  {
    robots_[robot_id].state = *msg;
  }

  void RobotFeedbackCallback(
    const ateam_radio_msgs::msg::BasicTelemetry::SharedPtr msg,
    int robot_id)
  {
    robots_[robot_id].feedback = *msg;
  }

  void RobotMotionFeedbackCallback(
    const ateam_radio_msgs::msg::ExtendedTelemetry::SharedPtr msg, int robot_id)
  {
    robots_[robot_id].motion_feedback = *msg;
  }

  void ConnectionCallback(
    const ateam_radio_msgs::msg::ConnectionStatus::SharedPtr msg,
    int robot_id)
  {
    robots_[robot_id].connected = msg->radio_connected;
  }

  void WaitForRobot()
  {
    bool robot_ready = false;
    if(options_.robot_id.has_value()) {
      robot_id_ = *options_.robot_id;
      const auto & robot = robots_.at(robot_id_);
      robot_ready = robot.feedback.has_value() && robot.state.has_value() &&
        robot.connected && robot.state->visible;
    } else {
      const auto found_robot = std::find_if(robots_.begin(), robots_.end(),
          [](const Robot & robot)->bool{
            return robot.feedback.has_value() && robot.state.has_value() &&
                   robot.connected && robot.state->visible;
      });
      if(found_robot != robots_.end()) {
        robot_id_ = std::distance(robots_.begin(), found_robot);
        robot_ready = true;
      }
    }
    if(robot_ready) {
      command_pub_ =
        create_publisher<ateam_msgs::msg::RobotMotionCommand>(std::string(
        Topics::kRobotMotionCommandPrefix) + std::to_string(robot_id_), 1);
      RCLCPP_INFO(get_logger(), "Robot %d is ready.", robot_id_);
      RCLCPP_INFO(get_logger(), "Waiting for field info...");
      timer_ = create_wall_timer(kTimerDuration,
        std::bind(&AngularVelocityBenchmarkNode::WaitForField, this));
    }
  }

  void WaitForField()
  {
    if(field_.has_value()) {
      RCLCPP_INFO(get_logger(), "Field is ready.");

      const auto & robot = robots_[robot_id_];
      const auto yaw = GetYaw(robot);
      const auto dx = std::cos(yaw) * options_.radius;
      const auto dy = std::sin(yaw) * options_.radius;
      circle_center_ = ateam_geometry::Point(robot.state->pose.position.x + dx,
        robot.state->pose.position.y + dy);

      const auto safety_margin = 1.1;

      if (DistanceToFieldEdge(circle_center_.x(),
        circle_center_.y()) < (options_.radius * safety_margin))
      {
        RCLCPP_ERROR(get_logger(), "Not enough space to perform the benchmark.");
        rclcpp::shutdown();
        return;
      }

      starting_angle_ = yaw;

      RCLCPP_INFO(get_logger(), "Running benchmark...");
      start_time_ = std::chrono::steady_clock::now();
      timer_ = create_wall_timer(kTimerDuration,
        std::bind(&AngularVelocityBenchmarkNode::RunBenchmark, this));
    }
  }

  void RunBenchmark()
  {
    const auto & robot = robots_[robot_id_];

    const auto perp_distance = PerpendicularDistance(robot);

    if(perp_distance > kPerpendicularDistanceAllowance) {
      RCLCPP_ERROR(get_logger(), "Robot moved too far along the perpendicular axis.");
      EndBenchmark();
      return;
    }

    if(RobotOutsideOfField(robot)) {
      RCLCPP_ERROR(get_logger(), "Robot is outside of the field.");
      EndBenchmark();
      return;
    }

    CheckRotation(robot);

    const auto robot_speed = robot.state->twist_body.angular.z;

    if(rotation_count_ >= options_.rotations && std::abs(robot_speed) < 1e-2) {
      RCLCPP_INFO(get_logger(), "Benchmark completed successfully.");
      EndBenchmark();
      return;
    }

    ateam_msgs::msg::RobotMotionCommand command;

    const auto delta_t =
      std::chrono::duration_cast<std::chrono::duration<double>>(kTimerDuration).count();

    auto speed = 0.0;

    if (rotation_count_ < options_.rotations) {
      speed = std::min(std::abs(command_speed_) + (options_.accel * delta_t), options_.max_speed);
    } else {
      speed = std::max(std::abs(command_speed_) - (options_.accel * delta_t), 0.0);
    }
    if(options_.direction == Direction::Neg) {
      speed = -speed;
    }

    command_speed_ = speed;

    command.twist.angular.z = command_speed_;
    command.twist.linear.y = -1 * command_speed_ * options_.radius;

    command.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_DISABLE;

    command_pub_->publish(command);

    const auto vision_speed = robot.state->twist_body.angular.z;
    const auto vision_perp_speed = robot.state->twist_body.linear.y;
    const auto firmware_speed = robot.motion_feedback->cgkf_body_velocity_state_estimate[2];
    const auto firmware_perp_speed = robot.motion_feedback->cgkf_body_velocity_state_estimate[0];

    DataEntry entry{
      .time = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::steady_clock::now() - start_time_).count(),
      .command_speed = speed,
      .vision_speed = vision_speed,
      .vision_perp_speed = vision_perp_speed,
      .vision_perp_distance = perp_distance,
      .firmware_speed = firmware_speed,
      .firmware_perp_speed = firmware_perp_speed
    };
    data_.push_back(entry);
  }

  void EndBenchmark()
  {
    timer_.reset();
    ateam_msgs::msg::RobotMotionCommand command;
    command.twist.linear.x = 0.0;
    command.twist.linear.y = 0.0;
    command.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_DISABLE;
    command_pub_->publish(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    RCLCPP_INFO(get_logger(), "Benchmark finished.");

    const std::string timestamp = std::to_string(std::time(nullptr));
    const std::string filename = "vel_benchmark_" + timestamp + "_data.csv";

    std::ofstream file(filename);
    if(!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s for writing.", filename.c_str());
      return;
    }
    file <<
      "Time, Command speed, Vision speed, Vision perp speed, Vision perp distance, Firmware speed, Firmware perp speed\n";
    for(const auto & entry : data_) {
      file << entry.time << ", " << entry.command_speed << ", "
           << entry.vision_speed << ", " << entry.vision_perp_speed << ", "
           << entry.vision_perp_distance << ", " << entry.firmware_speed << ", " <<
        entry.firmware_perp_speed << "\n";
    }
    file.close();
    RCLCPP_INFO(get_logger(), "Data saved to %s", filename.c_str());

    CalculateStats(timestamp);
    RenderGraph(filename, timestamp, GraphDestination::PNG);
    RenderGraph(filename, timestamp, GraphDestination::X11);

    rclcpp::shutdown();
  }

  void CalculateStats(const std::string & filename_timestamp)
  {
    if(data_.empty()) {
      RCLCPP_WARN(get_logger(), "No data collected during the benchmark.");
      return;
    }

    const auto vision_errors = std::views::transform(data_,
        [](const DataEntry & entry) {
          return entry.vision_speed - entry.command_speed;
      });
    const auto firmware_errors = std::views::transform(data_,
        [](const DataEntry & entry) {
          return entry.firmware_speed - entry.command_speed;
      });

    const auto speed_vision_sum_sq_error = std::accumulate(vision_errors.begin(),
      vision_errors.end(), 0.0,
        [](double acc, const double & error) {
          return acc + std::pow(error, 2);
      });
    const auto speed_vision_avg_sq_error = speed_vision_sum_sq_error / vision_errors.size();
    const auto speed_firmware_sum_sq_error = std::accumulate(firmware_errors.begin(),
      firmware_errors.end(), 0.0,
        [](double acc, const double & error) {
          return acc + std::pow(error, 2);
      });
    const auto speed_firmware_avg_sq_error = speed_firmware_sum_sq_error / firmware_errors.size();

    const auto max_vision_error = std::ranges::max(vision_errors |
      std::views::transform([](double error) {
          return std::abs(error);
    }));
    const auto max_firmware_error = std::ranges::max(firmware_errors |
      std::views::transform([](double error) {
          return std::abs(error);
    }));
    const auto min_vision_error = std::ranges::min(vision_errors |
      std::views::transform([](double error) {
          return std::abs(error);
    }));
    const auto min_firmware_error = std::ranges::min(firmware_errors |
      std::views::transform([](double error) {
          return std::abs(error);
    }));

    std::stringstream ss;
    ss << '\n';
    ss << "Benchmark Statistics:\n";
    ss << "  Total entries: " << data_.size() << "\n";
    ss        << "  Benchmark Duration: "
              << data_.back().time << " seconds\n";
    ss << "  Vision Speed Loss: " << speed_vision_avg_sq_error << "\n";
    ss << "  Firmware Speed Loss: " << speed_firmware_avg_sq_error << "\n";
    ss << "  Max Vision Speed Error: " << max_vision_error << "\n";
    ss << "  Max Firmware Speed Error: " << max_firmware_error << "\n";
    ss << "  Min Vision Speed Error: " << min_vision_error << "\n";
    ss << "  Min Firmware Speed Error: " << min_firmware_error << "\n";

    const auto stats_display = ss.str();
    RCLCPP_INFO(get_logger(), "%s", stats_display.c_str());

    const std::string filename = "vel_benchmark_" + filename_timestamp + "_stats.csv";

    std::ofstream file(filename);
    if(!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s for writing.", filename.c_str());
      return;
    }
    file << stats_display;
    file.close();
    RCLCPP_INFO(get_logger(), "Statistics saved to %s", filename.c_str());
  }

  enum class GraphDestination
  {
    X11,
    PNG
  };

  void RenderGraph(
    const std::string & results_filename, const std::string & filename_timestamp,
    const GraphDestination dest)
  {
    const auto width = 1200;
    const auto height = 400;
    const auto x11_terminal_commands = std::format("set terminal x11 size {} {}", width, height);
    const auto png_filename = "vel_benchmark_" + filename_timestamp + "_graphs.png";
    const auto png_terminal_commands =
      std::format("set terminal png size {} {} font 'Helvetica,10'; set output '{}'", width, height,
      png_filename);
    const auto terminal_commands = dest ==
      GraphDestination::X11 ? x11_terminal_commands : png_terminal_commands;
    std::string command = std::format("gnuplot -p -e \""
      "{0};"
      "set title 'Velocity Benchmark Results';"
      "set multiplot layout 1,3;"
      "set xlabel 'Time (s)';"
      "plot '{1}' using 1:2 with lines title 'Command Speed', "
      "'{1}' using 1:3 with lines title 'Vision Speed', "
      "'{1}' using 1:6 with lines title 'Firmware Speed';"
      "plot '{1}' using 1:4 with lines title 'Vision Perpendicular Speed', "
      "'{1}' using 1:7 with lines title 'Firmware Perpendicular Speed';"
      "plot '{1}' using 1:5 with lines title 'Vision Perpendicular Distance';"
      "\"", terminal_commands, results_filename);
    int ret = std::system(command.c_str());
    if(ret == -1) {
      RCLCPP_ERROR(get_logger(), "Failed to run gnuplot.");
    } else {
      RCLCPP_INFO(get_logger(), "Graph view started.");
    }
  }

  double PerpendicularDistance(const Robot & robot)
  {
    // In this behchmark, "perpendicular distance" is the distance from the intended circle
    return std::abs(CGAL::approximate_sqrt(CGAL::squared_distance(circle_center_,
      ateam_geometry::Point(robot.state->pose.position.x,
      robot.state->pose.position.y))) - options_.radius);
  }

  bool RobotOutsideOfField(const Robot & robot)
  {
    return !(std::abs(robot.state->pose.position.x) < field_->field_length / 2.0 &&
           std::abs(robot.state->pose.position.y) < field_->field_width / 2.0);
  }

  double DistanceToFieldEdge(const auto x, const auto y)
  {
    double distance_to_edge = std::numeric_limits<double>::max();

    distance_to_edge = std::min(distance_to_edge,
        (field_->field_length / 2.0) - std::abs(x));
    distance_to_edge = std::min(distance_to_edge,
        (field_->field_width / 2.0) - std::abs(y));

    return distance_to_edge;
  }

  double DistanceToFieldEdge(const Robot & robot)
  {
    return DistanceToFieldEdge(robot.state->pose.position.x, robot.state->pose.position.y);
  }

  double GetYaw(const Robot & robot)
  {
    tf2::Quaternion quat;
    tf2::fromMsg(robot.state->pose.orientation, quat);
    double yaw, pitch, roll;
    tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);
    return yaw;
  }

  void CheckRotation(const Robot & robot)
  {
    const auto current_angle = GetYaw(robot);

    auto normalized_current_angle = angles::shortest_angular_distance(starting_angle_,
      current_angle);

    if (options_.direction == Direction::Neg) {
      normalized_current_angle = -normalized_current_angle;
    }

    if (std::abs(normalized_current_angle) < 1e-2) {
      normalized_current_angle = 0.0;
    }

    if (passed_halfway_) {
      if (prev_angle_ < 0.0 && normalized_current_angle >= 0.0) {
        rotation_count_++;
        passed_halfway_ = false;
      }
    } else {
      if (prev_angle_ > 0.0 && normalized_current_angle <= 0.0) {
        passed_halfway_ = true;
      }
    }

    prev_angle_ = normalized_current_angle;
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto options = Options::from_args(argc, argv);
  rclcpp::spin(std::make_shared<AngularVelocityBenchmarkNode>(options));
  return 0;
}
