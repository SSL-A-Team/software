#include <getopt.h>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <optional>
#include <ranges>
#include <rclcpp/rclcpp.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_geometry/types.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_msgs/msg/robot_feedback.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/robot_motion_feedback.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


enum class Axis
{
  X,
  Y
};


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


struct DirDistance
{
  Direction direction;
  double distance;
};


struct Options
{
  ateam_common::TeamColor team_color = ateam_common::TeamColor::Blue;
  std::optional<int> robot_id = std::nullopt;
  Axis axis = Axis::X;
  double max_speed = 2.0;
  double accel = 1.0;

  static Options from_args(int argc, char ** argv)
  {
    Options opts;

    static struct option long_options[] = {
      {"color", required_argument, 0, 'c'},
      {"robot", required_argument, 0, 'r'},
      {"axis", required_argument, 0, 'a'},
      {"speed", required_argument, 0, 's'},
      {"accel", required_argument, 0, 'x'},
      {0, 0, 0, 0}
    };

    const char * short_options = "c:r:a:s:x:";

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
        case 'a':
          if(std::strcmp(optarg, "x") == 0) {
            opts.axis = Axis::X;
          } else if(std::strcmp(optarg, "y") == 0) {
            opts.axis = Axis::Y;
          } else {
            throw std::runtime_error("Unrecognized axis value.");
          }
          break;
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
  std::optional<ateam_msgs::msg::RobotState> state;
  std::optional<ateam_msgs::msg::RobotFeedback> feedback;
  std::optional<ateam_msgs::msg::RobotMotionFeedback> motion_feedback;
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


class VelocityBenchmarkNode : public rclcpp::Node
{
public:
  explicit VelocityBenchmarkNode(Options options)
  : Node("velocity_benchmark"), options_(options)
  {
    using ateam_common::indexed_topic_helpers::create_indexed_subscribers;

    field_sub_ = create_subscription<ateam_msgs::msg::FieldInfo>(std::string(Topics::kField), 1,
      std::bind(&VelocityBenchmarkNode::FieldCallback, this, std::placeholders::_1));

    const auto state_topic_prefix = options_.team_color ==
      ateam_common::TeamColor::Blue ? Topics::kBlueTeamRobotPrefix : Topics::kYellowTeamRobotPrefix;
    create_indexed_subscribers<ateam_msgs::msg::RobotState>(
      robot_state_subs_, state_topic_prefix, 1, &VelocityBenchmarkNode::RobotStateCallback, this);

    create_indexed_subscribers<ateam_msgs::msg::RobotFeedback>(robot_feedback_subs_,
      Topics::kRobotFeedbackPrefix, 1, &VelocityBenchmarkNode::RobotFeedbackCallback, this);

    create_indexed_subscribers<ateam_msgs::msg::RobotMotionFeedback>(
      robot_motion_feedback_subs_, Topics::kRobotMotionFeedbackPrefix, 1,
      &VelocityBenchmarkNode::RobotMotionFeedbackCallback, this);

    RCLCPP_INFO(get_logger(), "Waiting for robot...");
    timer_ = create_wall_timer(kTimerDuration,
      std::bind(&VelocityBenchmarkNode::WaitForRobot, this));
  }

private:
  const std::chrono::milliseconds kTimerDuration = std::chrono::milliseconds(10);
  const double kPerpendicularDistanceAllowance = 0.5;

  Options options_;
  std::optional<ateam_msgs::msg::FieldInfo> field_;
  std::array<Robot, 16> robots_;
  std::vector<DataEntry> data_;

  std::chrono::steady_clock::time_point start_time_;
  int robot_id_ = -1;
  double perpendicular_start_pos_ = 0.0;
  Direction direction_ = Direction::Pos;
  ateam_geometry::Ray motion_ray_;
  double command_speed_ = 0.0;

  rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<ateam_msgs::msg::FieldInfo>::SharedPtr field_sub_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr, 16> robot_state_subs_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotFeedback>::SharedPtr,
    16> robot_feedback_subs_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotMotionFeedback>::SharedPtr,
    16> robot_motion_feedback_subs_;
  rclcpp::TimerBase::SharedPtr timer_;

  void FieldCallback(const ateam_msgs::msg::FieldInfo::SharedPtr msg)
  {
    field_ = *msg;
  }

  void RobotStateCallback(const ateam_msgs::msg::RobotState::SharedPtr msg, int robot_id)
  {
    robots_[robot_id].state = *msg;
  }

  void RobotFeedbackCallback(const ateam_msgs::msg::RobotFeedback::SharedPtr msg, int robot_id)
  {
    robots_[robot_id].feedback = *msg;
  }

  void RobotMotionFeedbackCallback(
    const ateam_msgs::msg::RobotMotionFeedback::SharedPtr msg, int robot_id)
  {
    robots_[robot_id].motion_feedback = *msg;
  }

  void WaitForRobot()
  {
    bool robot_ready = false;
    if(options_.robot_id.has_value()) {
      robot_id_ = *options_.robot_id;
      const auto & robot = robots_.at(robot_id_);
      robot_ready = robot.feedback.has_value() && robot.state.has_value() &&
        robot.feedback->radio_connected && robot.state->visible;
    } else {
      const auto found_robot = std::find_if(robots_.begin(), robots_.end(),
          [](const Robot & robot)->bool{
            return robot.feedback.has_value() && robot.state.has_value() &&
                   robot.feedback->radio_connected && robot.state->visible;
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
        std::bind(&VelocityBenchmarkNode::WaitForField, this));
    }
  }

  void WaitForField()
  {
    if(field_.has_value()) {
      RCLCPP_INFO(get_logger(), "Field is ready.");

      const auto & robot = robots_[robot_id_];

      if(options_.axis == Axis::X) {
        perpendicular_start_pos_ = robot.state->pose.position.y;
      } else if (options_.axis == Axis::Y) {
        perpendicular_start_pos_ = robot.state->pose.position.x;
      }

      direction_ = PickDirection(robot);
      RCLCPP_INFO(get_logger(), "Direction: %s",
        direction_ == Direction::Pos ? "Positive" : "Negative");
      motion_ray_ = GetLocalRay(robot, options_.axis, direction_);

      RCLCPP_INFO(get_logger(), "Running benchmark...");
      start_time_ = std::chrono::steady_clock::now();
      timer_ = create_wall_timer(kTimerDuration,
        std::bind(&VelocityBenchmarkNode::RunBenchmark, this));
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

    const auto safety_factor = 1.1;
    const auto stopping_distance = safety_factor *
      ((options_.max_speed * options_.max_speed) / (2 * options_.accel));

    const auto distance_to_field_edge = DistanceToFieldEdge(robot);

    const auto robot_speed = [&]{
        if(options_.axis == Axis::X) {
          return std::abs(robot.state->twist.linear.x);
        } else {
          return std::abs(robot.state->twist.linear.y);
        }
      }();

    if(distance_to_field_edge.direction == direction_ &&
      distance_to_field_edge.distance < stopping_distance && std::abs(robot_speed) < 1e-2)
    {
      RCLCPP_INFO(get_logger(), "Benchmark completed successfully.");
      EndBenchmark();
      return;
    }

    ateam_msgs::msg::RobotMotionCommand command;

    const auto delta_t =
      std::chrono::duration_cast<std::chrono::duration<double>>(kTimerDuration).count();

    auto speed = 0.0;

    if(distance_to_field_edge.direction == Invert(direction_) ||
      distance_to_field_edge.distance > stopping_distance)
    {
      speed = std::min(std::abs(command_speed_) + (options_.accel * delta_t), options_.max_speed);
      if(direction_ == Direction::Neg) {
        speed = -speed;
      }
    } else {
      speed = std::max(std::abs(command_speed_) - (options_.accel * delta_t), 0.0);
      if(direction_ == Direction::Neg) {
        speed = -speed;
      }
    }

    command_speed_ = speed;

    if(options_.axis == Axis::X) {
      command.twist.linear.x = speed;
    } else {
      command.twist.linear.y = speed;
    }

    command.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_DISABLE;

    command_pub_->publish(command);

    const auto vision_speed = options_.axis == Axis::X ?
      robot.state->twist_body.linear.x : robot.state->twist_body.linear.y;
    const auto vision_perp_speed = options_.axis == Axis::X ?
      robot.state->twist_body.linear.y : robot.state->twist_body.linear.x;
    const auto firmware_speed = options_.axis == Axis::X ?
      robot.motion_feedback->body_velocity_state_estimate.linear.x :
      robot.motion_feedback->body_velocity_state_estimate.linear.y;
    const auto firmware_perp_speed = options_.axis == Axis::X ?
      robot.motion_feedback->body_velocity_state_estimate.linear.y :
      robot.motion_feedback->body_velocity_state_estimate.linear.x;

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

    const auto max_vision_error = std::ranges::max(vision_errors | std::views::transform([](double error) {
      return std::abs(error);
    }));
    const auto max_firmware_error = std::ranges::max(firmware_errors | std::views::transform([](double error) {
      return std::abs(error);
    }));
    const auto min_vision_error = std::ranges::min(vision_errors | std::views::transform([](double error) {
      return std::abs(error);
    }));
    const auto min_firmware_error = std::ranges::min(firmware_errors | std::views::transform([](double error) {
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

  void RenderGraph(const std::string & results_filename, const std::string & filename_timestamp, const GraphDestination dest)
  {
    const auto width = 1200;
    const auto height = 400;
    const auto x11_terminal_commands = std::format("set terminal x11 size {} {}", width, height);
    const auto png_filename = "vel_benchmark_" + filename_timestamp + "_graphs.png";
    const auto png_terminal_commands = std::format("set terminal png size {} {} font 'Helvetica,10'; set output '{}'", width, height, png_filename);
    const auto terminal_commands = dest == GraphDestination::X11 ? x11_terminal_commands : png_terminal_commands;
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

  Direction PickDirection(const Robot & robot)
  {
    return Invert(DistanceToFieldEdge(robot).direction);
  }

  double PerpendicularDistance(const Robot & robot)
  {
    const auto motion_line = ateam_geometry::Line(motion_ray_.source(), motion_ray_.direction());
    const auto robot_pos = ateam_geometry::Point(robot.state->pose.position.x,
      robot.state->pose.position.y);
    return CGAL::approximate_sqrt(CGAL::squared_distance(motion_line, robot_pos));
  }

  bool RobotOutsideOfField(const Robot & robot)
  {
    return !(std::abs(robot.state->pose.position.x) < field_->field_length / 2.0 &&
           std::abs(robot.state->pose.position.y) < field_->field_width / 2.0);
  }

  DirDistance DistanceToFieldEdge(const Robot & robot)
  {
    const auto pos_ray = GetLocalRay(robot, options_.axis, Direction::Pos);
    const auto neg_ray = GetLocalRay(robot, options_.axis, Direction::Neg);

    const ateam_geometry::Rectangle field_rectangle(
      ateam_geometry::Point(field_->field_length / 2.0, field_->field_width / 2.0),
      ateam_geometry::Point(-field_->field_length / 2.0, -field_->field_width / 2.0));

    const auto pos_intersection_res = CGAL::intersection(field_rectangle, pos_ray);
    const auto neg_intersection_res = CGAL::intersection(field_rectangle, neg_ray);

    if(!pos_intersection_res || !neg_intersection_res) {
      throw std::runtime_error(
        "Failed to find intersection between robot ray and field rectangle. Is robot outside of the field?");
    }

    const auto * pos_intersection_pt = boost::get<ateam_geometry::Point>(&*pos_intersection_res);
    const auto * neg_intersection_pt = boost::get<ateam_geometry::Point>(&*neg_intersection_res);

    if(pos_intersection_pt && neg_intersection_pt) {
      throw std::runtime_error(
        "Both intersection results are points. The robot is not well positioned for this benchmark.");
    } else if(pos_intersection_pt) {
      return {Direction::Pos, 0.0};
    } else if(neg_intersection_pt) {
      return {Direction::Neg, 0.0};
    }

    const auto * pos_segment = boost::get<ateam_geometry::Segment>(&*pos_intersection_res);
    const auto * neg_segment = boost::get<ateam_geometry::Segment>(&*neg_intersection_res);

    if(pos_segment && neg_segment) {
      const auto pos_distance = CGAL::approximate_sqrt(pos_segment->squared_length());
      const auto neg_distance = CGAL::approximate_sqrt(neg_segment->squared_length());
      if(pos_distance < neg_distance) {
        return {Direction::Pos, pos_distance};
      } else {
        return {Direction::Neg, neg_distance};
      }
    } else if(pos_segment) {
      return {Direction::Pos, CGAL::approximate_sqrt(pos_segment->squared_length())};
    } else if(neg_segment) {
      return {Direction::Neg, CGAL::approximate_sqrt(neg_segment->squared_length())};
    } else {
      throw std::runtime_error("Unexpected intersection result type.");
    }
  }

  ateam_geometry::Ray GetLocalRay(
    const Robot & robot, const Axis & axis,
    const Direction & direction)
  {
    tf2::Quaternion quat;
    tf2::fromMsg(robot.state->pose.orientation, quat);
    double yaw, pitch, roll;
    tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);

    ateam_geometry::Vector vec;
    if(axis == Axis::X) {
      vec = ateam_geometry::Vector(cos(yaw), sin(yaw));
    } else if (axis == Axis::Y) {
      vec = ateam_geometry::Vector(-sin(yaw), cos(yaw));
    }

    const auto robot_pos = ateam_geometry::Point(robot.state->pose.position.x,
      robot.state->pose.position.y);

    if(direction == Direction::Pos) {
      return ateam_geometry::Ray(robot_pos, vec);
    } else {
      return ateam_geometry::Ray(robot_pos, -vec);
    }
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto options = Options::from_args(argc, argv);
  rclcpp::spin(std::make_shared<VelocityBenchmarkNode>(options));
  return 0;
}
