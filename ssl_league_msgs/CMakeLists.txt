cmake_minimum_required(VERSION 3.5)
project(ssl_league_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/game_controller/ControllerReply.msg
  msg/game_controller/GameEvent.msg
  msg/game_controller/GameEventProposalGroup.msg
  msg/game_controller/Referee.msg
  msg/game_controller/TeamInfo.msg

  msg/game_controller/common/Team.msg
  msg/game_controller/common/RobotId.msg
  msg/game_controller/common/Division.msg

  msg/game_controller/game_events/DefenderInDefenseArea.msg
  msg/game_controller/game_events/TooManyRobots.msg
  msg/game_controller/game_events/BotSubstitution.msg
  msg/game_controller/game_events/EmergencyStop.msg
  msg/game_controller/game_events/UnsportingBehaviorMajor.msg
  msg/game_controller/game_events/AttackerTooCloseToDefenseArea.msg
  msg/game_controller/game_events/BotInterferedPlacement.msg
  msg/game_controller/game_events/KeeperHeldBall.msg
  msg/game_controller/game_events/BotCrashUnique.msg
  msg/game_controller/game_events/Prepared.msg
  msg/game_controller/game_events/AttackerTouchedOpponentInDefenseArea.msg
  msg/game_controller/game_events/DefenderTooCloseToKickPoint.msg
  msg/game_controller/game_events/PlacementFailed.msg
  msg/game_controller/game_events/DefenderInDefenseAreaPartially.msg
  msg/game_controller/game_events/BotDribbledBallTooFar.msg
  msg/game_controller/game_events/UnsportingBehaviorMinor.msg
  msg/game_controller/game_events/PenaltyKickFailed.msg
  msg/game_controller/game_events/KickTimeout.msg
  msg/game_controller/game_events/BotCrashDrawn.msg
  msg/game_controller/game_events/ChippedGoal.msg
  msg/game_controller/game_events/NoProgressInGame.msg
  msg/game_controller/game_events/PlacementSucceeded.msg
  msg/game_controller/game_events/BotKickedBallTooFast.msg
  msg/game_controller/game_events/MultiplePlacementFailures.msg
  msg/game_controller/game_events/ChallengeFlag.msg
  msg/game_controller/game_events/BallLeftField.msg
  msg/game_controller/game_events/Goal.msg
  msg/game_controller/game_events/BotHeldBallDeliberately.msg
  msg/game_controller/game_events/AttackerTouchedBallInDefenseArea.msg
  msg/game_controller/game_events/BotTooFastInStop.msg
  msg/game_controller/game_events/AimlessKick.msg
  msg/game_controller/game_events/IndirectGoal.msg
  msg/game_controller/game_events/BotPushedBot.msg
  msg/game_controller/game_events/MultipleFouls.msg
  msg/game_controller/game_events/BotTippedOver.msg
  msg/game_controller/game_events/AttackerDoubleTouchedBall.msg
  msg/game_controller/game_events/BoundaryCrossing.msg
  msg/game_controller/game_events/MultipleCards.msg
  msg/game_controller/game_events/BotDroppedParts.msg
  msg/game_controller/game_events/ChallengeFlagHandled.msg
  msg/game_controller/game_events/ExcessiveBotSubstitution.msg

  msg/vision/VisionDetectionBall.msg
  msg/vision/VisionDetectionRobot.msg
  msg/vision/VisionDetectionFrame.msg

  msg/vision/VisionFieldCircularArc.msg
  msg/vision/VisionFieldLineSegment.msg
  msg/vision/VisionGeometryFieldSize.msg
  msg/vision/VisionGeometryCameraCalibration.msg
  msg/vision/VisionGeometryData.msg

  msg/vision/VisionWrapper.msg

  msg/simulator/SimulatorControl.msg
  msg/simulator/TeleportBallCommand.msg
  msg/simulator/TeleportRobotCommand.msg

  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
