struct Behavior {
  enum Type {
    GetBall,

    // Straight kick
    MovingKick,
    PivotKick,

    // Receive + kick
    OneTouchReceiveKick,

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

  // ALWAYS
  // time action should happen
  // time to complete the action

  // DEPENDING ON TYPE
  // location of action
  // target location of action
  // kick speed
  // cost function
};