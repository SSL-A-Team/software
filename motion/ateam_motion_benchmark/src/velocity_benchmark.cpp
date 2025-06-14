#include <getopt.h>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_geometry/types.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_msgs/msg/robot_feedback.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>


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
};


struct DataEntry
{
  double time;
  double command_speed;
  double robot_speed;
  double robot_perp_speed;
  double robot_perp_distance;
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

  int robot_id_ = -1;
  double perpendicular_start_pos_ = 0.0;
  Direction direction_ = Direction::Pos;
  ateam_geometry::Ray motion_ray_;

  rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<ateam_msgs::msg::FieldInfo>::SharedPtr field_sub_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr, 16> robot_state_subs_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotFeedback>::SharedPtr,
    16> robot_feedback_subs_;
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
      motion_ray_ = GetLocalRay(robot, options_.axis, direction_);

      RCLCPP_INFO(get_logger(), "Running benchmark...");
      timer_ = create_wall_timer(kTimerDuration,
        std::bind(&VelocityBenchmarkNode::RunBenchmark, this));
    }
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

    std::string filename = "benchmark_results_" + std::to_string(std::time(nullptr)) + ".csv";

    std::ofstream file(filename);
    if(!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open file for writing.");
      return;
    }
    file << "Time, Command speed, Robot speed, Robot perp speed, Robot perp distance\n";
    for(const auto & entry : data_) {
      file << entry.time << ", " << entry.command_speed << ", "
           << entry.robot_speed << ", " << entry.robot_perp_speed << ", "
           << entry.robot_perp_distance << "\n";
    }
    file.close();
    RCLCPP_INFO(get_logger(), "Results saved to %s", filename.c_str());

    rclcpp::shutdown();
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

    ateam_msgs::msg::RobotMotionCommand command;

    const auto delta_t =
      std::chrono::duration_cast<std::chrono::duration<double>>(kTimerDuration).count();

    auto speed = 0.0;

    if(distance_to_field_edge.direction == direction_ ||
      distance_to_field_edge.distance > stopping_distance)
    {
      speed = std::min(robot_speed + (options_.accel * delta_t), options_.max_speed);
      if(direction_ == Direction::Neg) {
        speed = -speed;
      }
    } else {
      speed = std::max(robot_speed - (options_.accel * delta_t), 0.0);
      if(direction_ == Direction::Pos) {
        speed = -speed;
      }
    }

    if(options_.axis == Axis::X) {
      command.twist.linear.x = speed;
    } else {
      command.twist.linear.y = speed;
    }

    command.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_DISABLE;

    command_pub_->publish(command);
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
    return std::abs(robot.state->pose.position.x) < field_->field_length / 2.0 &&
           std::abs(robot.state->pose.position.y) < field_->field_width / 2.0;
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
    ateam_geometry::Vector vec;
    if(axis == Axis::X) {
      vec = ateam_geometry::Vector(cos(robot.state->pose.orientation.z),
        sin(robot.state->pose.orientation.z));
    } else if (axis == Axis::Y) {
      vec = ateam_geometry::Vector(sin(robot.state->pose.orientation.z),
        cos(robot.state->pose.orientation.z));
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
