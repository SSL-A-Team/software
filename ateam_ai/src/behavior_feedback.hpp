struct Trajectory {};
struct DribblerAndKickerBehavior {};

struct BehaviorFeedback {
  std::optional<int> assigned_robot_id;
  // behavior start time
  // behavior end time
  // behavior wasted time (how long does this robot need to wait for other robots to get into position)
  Trajectory trajectory;
  DribblerAndKickerBehavior dribbler_and_kicker_behavior; // Maybe?
};