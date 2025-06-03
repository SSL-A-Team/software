#include <getopt.h>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
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

    timer_ = create_wall_timer(kTimerDuration,
      std::bind(&VelocityBenchmarkNode::WaitForRobot, this));
  }

private:
  const std::chrono::milliseconds kTimerDuration = std::chrono::milliseconds(10);
  const double kPerpendicularDistanceAllowance = 0.5;

  Options options_;
  std::optional<ateam_msgs::msg::FieldInfo> field_;
  std::array<Robot, 16> robots_;

  int robot_id_ = -1;
  double perpendicular_start_pos_ = 0.0;
  Direction direction_ = Direction::Pos;

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
      if(found_robot == robots_.end()) {
        robot_id_ = std::distance(robots_.begin(), found_robot);
        robot_ready = true;
      }
    }
    if(robot_ready) {
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

      RCLCPP_INFO(get_logger(), "Running benchmark...");
      timer_ = create_wall_timer(kTimerDuration,
        std::bind(&VelocityBenchmarkNode::RunBenchmark, this));
    }
  }

  void RunBenchmark()
  {
    const auto & robot = robots_[robot_id_];

    if(PerpendicularDistanceExceeded(robot)) {
      throw std::runtime_error(
        "Benchmark failed: Robot moved too far along the perpendicular axis.");
    }

    if(RobotOutsideOfField(robot)) {
      throw std::runtime_error("Benchmark failed: Robot got too close to the field edge.");
    }

    // TODO(barulicm): implement benchmark

    rclcpp::shutdown();
  }

  Direction PickDirection(const Robot & robot) {
    // TODO(barulicm): Implement this
    return Direction::Pos;
  }


  bool PerpendicularDistanceExceeded(const Robot & robot)
  {
    double perp_distance = 0.0;
    if(options_.axis == Axis::X) {
      perp_distance = std::abs(robot.state->pose.position.y - perpendicular_start_pos_);
    } else if (options_.axis == Axis::Y) {
      perp_distance = std::abs(robot.state->pose.position.x - perpendicular_start_pos_);
    }
    return perp_distance > kPerpendicularDistanceAllowance;
  }

  bool RobotOutsideOfField(const Robot & robot)
  {
    // TODO(barulicm): Implement this
    return false;
  }

  double SignedDistanceToFieldEdge(const Robot & robot)
  {

  }

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto options = Options::from_args(argc, argv);
  rclcpp::spin(std::make_shared<VelocityBenchmarkNode>(options));
  return 0;
}
