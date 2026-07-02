#include <ssl_league_protobufs/ssl_vision_wrapper_tracked.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_common/multicast_receiver.hpp>
#include <ateam_common/protobuf_logging.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_msgs/msg/vision_state_ball.hpp>
#include <ateam_msgs/msg/vision_state_robot.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ateam_vision_tracker_bridge
{

class VisionTrackerBridgeNode : public rclcpp::Node {
public:
  explicit VisionTrackerBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("vision_tracker_bridge", options),
    receiver_(
      declare_parameter<std::string>("ssl_tracking_ip", "224.5.23.2"),
      declare_parameter<int>("ssl_tracking_port", 10010),
      std::bind(&VisionTrackerBridgeNode::MulticastCallback, this, std::placeholders::_1, std::placeholders::_3,
      std::placeholders::_4),
      declare_parameter<std::string>("net_interface_address", "")
    ),
    gc_listener_(*this)
  {
    SET_ROS_PROTOBUF_LOG_HANDLER("vision_tracker_bridge.protobuf");

    declare_parameter<std::string>("source", "TIGERs");

    ball_publisher_ = create_publisher<ateam_msgs::msg::VisionStateBall>(
      std::string(Topics::kBall),
      rclcpp::SystemDefaultsQoS());

    ateam_common::indexed_topic_helpers::create_indexed_publishers
    <ateam_msgs::msg::VisionStateRobot>(
      blue_robots_publishers_,
      Topics::kBlueTeamRobotPrefix,
      rclcpp::SystemDefaultsQoS(),
      this
    );
    ateam_common::indexed_topic_helpers::create_indexed_publishers
    <ateam_msgs::msg::VisionStateRobot>(
      yellow_robots_publishers_,
      Topics::kYellowTeamRobotPrefix,
      rclcpp::SystemDefaultsQoS(),
      this
    );

    field_subscription_ =
      create_subscription<ateam_msgs::msg::FieldInfo>(
      std::string(Topics::kField),
      10,
      std::bind(&VisionTrackerBridgeNode::FieldCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Vision tracker bridge ready.");
    ready_ = true;
  }

private:
  std::atomic<bool> ready_ = false;
  std::string last_seen_uuid = "";
  std::string last_seen_sender_address = "";
  ateam_common::MulticastReceiver receiver_;
  ateam_common::GameControllerListener gc_listener_;
  rclcpp::Publisher<ateam_msgs::msg::VisionStateBall>::SharedPtr ball_publisher_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::VisionStateRobot>::SharedPtr,
    16> blue_robots_publishers_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::VisionStateRobot>::SharedPtr,
    16> yellow_robots_publishers_;
  rclcpp::Subscription<ateam_msgs::msg::FieldInfo>::SharedPtr field_subscription_;

  int ignore_side_raw_;

  void MulticastCallback(const std::string & sender_address, uint8_t * buffer, size_t bytes_received)
  {
    if(!ready_) {
      std::cerr << "Skipping not ready.\n";
      return;
    }
    TrackerWrapperPacket tracker_proto;

    if(!tracker_proto.ParseFromArray(buffer, bytes_received)) {
      RCLCPP_WARN(get_logger(), "Failed to parse tracker protobuf packet.");
      return;
    }

    const auto trusted_source = get_parameter("source").as_string();
    if(!tracker_proto.has_source_name()) {
      return;
    }
    if(!trusted_source.empty() && tracker_proto.source_name() != trusted_source) {
      return;
    }

    if(last_seen_uuid.empty()) {
      last_seen_uuid = tracker_proto.uuid();
    }
    if(tracker_proto.uuid() != last_seen_uuid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Receiving packets from multiple UUIDs");
    }

    if(last_seen_sender_address.empty()) {
      last_seen_sender_address = sender_address;
    }
    if(sender_address != last_seen_sender_address) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Receiving packets from multiple IPs");
    }

    if(!tracker_proto.has_tracked_frame()) {
      return;
    }

    auto side_multiplier = 1.0;
    auto side_angle_offset = 0.0;
    if(gc_listener_.GetTeamSide() == ateam_common::TeamSide::PositiveHalf) {
      side_multiplier = -1.0;
      side_angle_offset = M_PI;
    }

    const auto & proto_balls = tracker_proto.tracked_frame().balls();
    for(const auto & proto_ball : proto_balls) {
      if(ignore_side_raw_ < 0 && proto_ball.pos().x() < 0.0) {
        continue;
      }
      if(ignore_side_raw_ > 0 && proto_ball.pos().x() > 0.0) {
        continue;
      }
      ateam_msgs::msg::VisionStateBall ball_msg;
      ball_msg.pose.position.x = side_multiplier * proto_ball.pos().x();
      ball_msg.pose.position.y = side_multiplier * proto_ball.pos().y();
      ball_msg.pose.position.z = proto_ball.pos().z();
      ball_msg.twist.linear.x = side_multiplier * proto_ball.vel().x();
      ball_msg.twist.linear.y = side_multiplier * proto_ball.vel().y();
      ball_msg.twist.linear.z = proto_ball.vel().z();
      ball_msg.visible = proto_ball.visibility();
      ball_publisher_->publish(ball_msg);
      break;
    }

    std::array<bool, 16> blue_seen_ids;
    std::array<bool, 16> yellow_seen_ids;
    std::fill(blue_seen_ids.begin(), blue_seen_ids.end(), false);
    std::fill(yellow_seen_ids.begin(), yellow_seen_ids.end(), false);
    const auto & proto_bots = tracker_proto.tracked_frame().robots();
    for(const auto & proto_bot : proto_bots) {
      if(ignore_side_raw_ < 0 && proto_bot.pos().x() < 0.0) {
        continue;
      }
      if(ignore_side_raw_ > 0 && proto_bot.pos().x() > 0.0) {
        continue;
      }
      const auto & color = proto_bot.robot_id().team();
      const auto & id = proto_bot.robot_id().id();
      if(id > 15) {
        continue;
      }
      if(color == 0) {
        continue;
      }
      if(color == 1 && yellow_seen_ids[id]) {
        continue;
      }
      if(color == 2 && blue_seen_ids[id]) {
        continue;
      }

      ateam_msgs::msg::VisionStateRobot bot_msg;
      bot_msg.pose.position.x = side_multiplier * proto_bot.pos().x();
      bot_msg.pose.position.y = side_multiplier * proto_bot.pos().y();
      bot_msg.pose.orientation =
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), proto_bot.orientation() + side_angle_offset));
      bot_msg.twist.linear.x = side_multiplier * proto_bot.vel().x();
      bot_msg.twist.linear.y = side_multiplier * proto_bot.vel().y();
      bot_msg.twist.angular.z = proto_bot.vel_angular();
      bot_msg.visible = proto_bot.visibility();

      if(color == 1) {
        yellow_robots_publishers_[id]->publish(bot_msg);
        yellow_seen_ids[id] = true;
      } else if(color == 2) {
        blue_robots_publishers_[id]->publish(bot_msg);
        blue_seen_ids[id] = true;
      }
    }
    for(auto id = 0; id < 16; ++id) {
      if(!blue_seen_ids[id]) {
        ateam_msgs::msg::VisionStateRobot bot_msg;
        bot_msg.visible = false;
        blue_robots_publishers_[id]->publish(bot_msg);
      }
      if(!yellow_seen_ids[id]) {
        ateam_msgs::msg::VisionStateRobot bot_msg;
        bot_msg.visible = false;
        yellow_robots_publishers_[id]->publish(bot_msg);
      }
    }
  }

  void FieldCallback(
    const ateam_msgs::msg::FieldInfo::SharedPtr field_info_msg)
  {
    ignore_side_raw_ = field_info_msg->ignore_side_raw;
  }

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_vision_tracker_bridge::VisionTrackerBridgeNode)
