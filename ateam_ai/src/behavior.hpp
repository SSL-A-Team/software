#include <Eigen/Dense>

struct GetBallParam {};

struct KickParam {
  Eigen::Vector2d target_location;

  KickParam(Eigen::Vector2d target_location) : target_location(target_location) {}
};

struct ReceiveParam {
  Eigen::Vector2d receive_location;  // Repetitive info from previous pass info, do we need?
  Eigen::Vector2d target_location;

  ReceiveParam(Eigen::Vector2d receive_location, Eigen::Vector2d target_location)
    : receive_location(receive_location), target_location(target_location) {}
};

struct ShotParam {
};

struct ReceiveShotParam {
  Eigen::Vector2d receive_location;

  ShotParam(Eigen::Vector2d receive_location) : receive_location(receive_location) {}
}

struct MoveParam {
  Eigen::Vector2d target_location;

  MoveParam(Eigen::Vector2d target_location) : target_location(target_location) {}
};

struct CostParam {
  // std::function<Eigen::Vector2d()>
};

struct Behavior {
  enum Type {
    GetBall,

    // Straight kick
    MovingKick,
    PivotKick,

    // Receive + kick
    OneTouchReceiveKick,
    TwoTouchReceiveKick,

    // Kick + instantaneous target selection near kick time
    Shot,
    OneTouchShot,

    // Normal moves
    MoveToPoint,
    CostFunctionPoint
  } type;

  enum Priority {
    Required,
    Medium,
    Low
  } priority;

  using Params = std::varaint<GetBallParam, KickParam, ReceiveParam, ShotParam, MoveParam, CostParam>;
  Params params;
};