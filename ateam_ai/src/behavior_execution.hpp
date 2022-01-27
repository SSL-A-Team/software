struct Trajectory {};
struct DribblerAndKickerBehavior {};

struct BehaviorExecution {
  int assigned_robot_id;
  // behavior start time
  Trajectory trajectory;
  DribblerAndKickerBehavior dribbler_and_kicker_behavior; // Maybe?
};