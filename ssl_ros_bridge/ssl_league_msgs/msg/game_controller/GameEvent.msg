string id
uint8 type
string[] origin
uint64 created_timestamp
ssl_league_msgs/BallLeftField[] ball_left_field_touch_line
ssl_league_msgs/BallLeftField[] ball_left_field_goal_line
ssl_league_msgs/AimlessKick[] aimless_kick
ssl_league_msgs/AttackerTooCloseToDefenseArea[] attacker_too_close_to_defense_area
ssl_league_msgs/DefenderInDefenseArea[] defender_in_defense_area
ssl_league_msgs/BoundaryCrossing[] boundary_crossing
ssl_league_msgs/KeeperHeldBall[] keeper_held_ball
ssl_league_msgs/BotDribbledBallTooFar[] bot_dribbled_ball_too_far
ssl_league_msgs/BotPushedBot[] bot_pushed_bot
ssl_league_msgs/BotHeldBallDeliberately[] bot_held_ball_deliberately
ssl_league_msgs/BotTippedOver[] bot_tipped_over
ssl_league_msgs/BotDroppedParts[] bot_dropped_parts
ssl_league_msgs/AttackerTouchedBallInDefenseArea[] attacker_touched_ball_in_defense_area
ssl_league_msgs/BotKickedBallTooFast[] bot_kicked_ball_too_fast
ssl_league_msgs/BotCrashUnique[] bot_crash_unique
ssl_league_msgs/BotCrashDrawn[] bot_crash_drawn
ssl_league_msgs/DefenderTooCloseToKickPoint[] defender_too_close_to_kick_point
ssl_league_msgs/BotTooFastInStop[] bot_too_fast_in_stop
ssl_league_msgs/BotInterferedPlacement[] bot_interfered_placement
ssl_league_msgs/Goal[] possible_goal
ssl_league_msgs/Goal[] goal
ssl_league_msgs/Goal[] invalid_goal
ssl_league_msgs/AttackerDoubleTouchedBall[] attacker_double_touched_ball
ssl_league_msgs/PlacementSucceeded[] placement_succeeded
ssl_league_msgs/PenaltyKickFailed[] penalty_kick_failed
ssl_league_msgs/NoProgressInGame[] no_progress_in_game
ssl_league_msgs/PlacementFailed[] placement_failed
ssl_league_msgs/MultipleCards[] multiple_cards
ssl_league_msgs/MultipleFouls[] multiple_fouls
ssl_league_msgs/BotSubstitution[] bot_substitution
ssl_league_msgs/ExcessiveBotSubstitution[] excessive_bot_substitution
ssl_league_msgs/TooManyRobots[] too_many_robots
ssl_league_msgs/ChallengeFlag[] challenge_flag
ssl_league_msgs/ChallengeFlagHandled[] challenge_flag_handled
ssl_league_msgs/EmergencyStop[] emergency_stop
ssl_league_msgs/UnsportingBehaviorMinor[] unsporting_behavior_minor
ssl_league_msgs/UnsportingBehaviorMajor[] unsporting_behavior_major
ssl_league_msgs/Prepared[] prepared
ssl_league_msgs/IndirectGoal[] indirect_goal
ssl_league_msgs/ChippedGoal[] chipped_goal
ssl_league_msgs/KickTimeout[] kick_timeout
ssl_league_msgs/AttackerTouchedOpponentInDefenseArea[] attacker_touched_opponent_in_defense_area
ssl_league_msgs/AttackerTouchedOpponentInDefenseArea[] attacker_touched_opponent_in_defense_area_skipped
ssl_league_msgs/BotCrashUnique[] bot_crash_unique_skipped
ssl_league_msgs/BotPushedBot[] bot_pushed_bot_skipped
ssl_league_msgs/DefenderInDefenseAreaPartially[] defender_in_defense_area_partially
ssl_league_msgs/MultiplePlacementFailures[] multiple_placement_failures


# Types
uint8 TYPE_UNKNOWN_GAME_EVENT_TYPE = 0
uint8 TYPE_BALL_LEFT_FIELD_TOUCH_LINE = 6
uint8 TYPE_BALL_LEFT_FIELD_GOAL_LINE = 7
uint8 TYPE_AIMLESS_KICK = 11
uint8 TYPE_ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA = 19
uint8 TYPE_DEFENDER_IN_DEFENSE_AREA = 31
uint8 TYPE_BOUNDARY_CROSSING = 41
uint8 TYPE_KEEPER_HELD_BALL = 13
uint8 TYPE_BOT_DRIBBLED_BALL_TOO_FAR = 17
uint8 TYPE_BOT_PUSHED_BOT = 24
uint8 TYPE_BOT_HELD_BALL_DELIBERATELY = 26
uint8 TYPE_BOT_TIPPED_OVER = 27
uint8 TYPE_BOT_DROPPED_PARTS = 47
uint8 TYPE_ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA = 15
uint8 TYPE_BOT_KICKED_BALL_TOO_FAST = 18
uint8 TYPE_BOT_CRASH_UNIQUE = 22
uint8 TYPE_BOT_CRASH_DRAWN = 21
uint8 TYPE_DEFENDER_TOO_CLOSE_TO_KICK_POINT = 29
uint8 TYPE_BOT_TOO_FAST_IN_STOP = 28
uint8 TYPE_BOT_INTERFERED_PLACEMENT = 20
uint8 TYPE_EXCESSIVE_BOT_SUBSTITUTION = 48
uint8 TYPE_POSSIBLE_GOAL = 39
uint8 TYPE_GOAL = 8
uint8 TYPE_INVALID_GOAL = 42
uint8 TYPE_ATTACKER_DOUBLE_TOUCHED_BALL = 14
uint8 TYPE_PLACEMENT_SUCCEEDED = 5
uint8 TYPE_PENALTY_KICK_FAILED = 43
uint8 TYPE_NO_PROGRESS_IN_GAME = 2
uint8 TYPE_PLACEMENT_FAILED = 3
uint8 TYPE_MULTIPLE_CARDS = 32
uint8 TYPE_MULTIPLE_FOULS = 34
uint8 TYPE_BOT_SUBSTITUTION = 37
uint8 TYPE_TOO_MANY_ROBOTS = 38
uint8 TYPE_CHALLENGE_FLAG = 44
uint8 TYPE_CHALLENGE_FLAG_HANDLED = 46
uint8 TYPE_EMERGENCY_STOP = 45
uint8 TYPE_UNSPORTING_BEHAVIOR_MINOR = 35
uint8 TYPE_UNSPORTING_BEHAVIOR_MAJOR = 36
# deprecated types
uint8 TYPE_PREPARED = 1
uint8 TYPE_INDIRECT_GOAL = 9
uint8 TYPE_CHIPPED_GOAL = 10
uint8 TYPE_KICK_TIMEOUT = 12
uint8 TYPE_ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA = 16
uint8 TYPE_ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA_SKIPPED = 40
uint8 TYPE_BOT_CRASH_UNIQUE_SKIPPED = 23
uint8 TYPE_BOT_PUSHED_BOT_SKIPPED = 25
uint8 TYPE_DEFENDER_IN_DEFENSE_AREA_PARTIALLY = 30
uint8 TYPE_MULTIPLE_PLACEMENT_FAILURES = 33
