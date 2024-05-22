string source_identifier
uint8 match_type
builtin_interfaces/Time timestamp
uint8 stage
int64 stage_time_left
uint8 command
uint32 command_counter
builtin_interfaces/Time command_timestamp
ssl_league_msgs/TeamInfo yellow
ssl_league_msgs/TeamInfo blue
geometry_msgs/Point32 designated_position
bool blue_team_on_positive_half
uint8 next_command
ssl_league_msgs/GameEvent[] game_events
ssl_league_msgs/GameEventProposalGroup[] game_event_proposals
int64 current_action_time_remaining
string status_message


# Match Types
uint8 MATCH_TYPE_UNKNOWN_MATCH = 0
uint8 MATCH_TYPE_GROUP_PHASE = 1
uint8 MATCH_TYPE_ELIMINATION_PHASE = 2
uint8 MATCH_TYPE_FRIENDLY = 3

# Stages
uint8 STAGE_NORMAL_FIRST_HALF_PRE = 0
uint8 STAGE_NORMAL_FIRST_HALF = 1
uint8 STAGE_NORMAL_HALF_TIME = 2
uint8 STAGE_NORMAL_SECOND_HALF_PRE = 3
uint8 STAGE_NORMAL_SECOND_HALF = 4
uint8 STAGE_EXTRA_TIME_BREAK = 5
uint8 STAGE_EXTRA_FIRST_HALF_PRE = 6
uint8 STAGE_EXTRA_FIRST_HALF = 7
uint8 STAGE_EXTRA_HALF_TIME = 8
uint8 STAGE_EXTRA_SECOND_HALF_PRE = 9
uint8 STAGE_EXTRA_SECOND_HALF = 10
uint8 STAGE_PENALTY_SHOOTOUT_BREAK = 11
uint8 STAGE_PENALTY_SHOOTOUT = 12
uint8 STAGE_POST_GAME = 13

# Commands
uint8 COMMAND_HALT = 0
uint8 COMMAND_STOP = 1
uint8 COMMAND_NORMAL_START = 2
uint8 COMMAND_FORCE_START = 3
uint8 COMMAND_PREPARE_KICKOFF_YELLOW = 4
uint8 COMMAND_PREPARE_KICKOFF_BLUE = 5
uint8 COMMAND_PREPARE_PENALTY_YELLOW = 6
uint8 COMMAND_PREPARE_PENALTY_BLUE = 7
uint8 COMMAND_DIRECT_FREE_YELLOW = 8
uint8 COMMAND_DIRECT_FREE_BLUE = 9
uint8 COMMAND_TIMEOUT_YELLOW = 12
uint8 COMMAND_TIMEOUT_BLUE = 13
uint8 COMMAND_BALL_PLACEMENT_YELLOW = 16
uint8 COMMAND_BALL_PLACEMENT_BLUE = 17
# Deprecated commands
uint8 COMMAND_INDIRECT_FREE_YELLOW = 10
uint8 COMMAND_INDIRECT_FREE_BLUE = 11
uint8 COMMAND_GOAL_YELLOW = 14
uint8 COMMAND_GOAL_BLUE = 15
