import { Team, TeamInfo, TeamColor } from "@/team"
import ROSLIB from "roslib"

enum gameStage {
    STAGE_NORMAL_FIRST_HALF_PRE,
    STAGE_NORMAL_FIRST_HALF,
    STAGE_NORMAL_HALF_TIME,
    STAGE_NORMAL_SECOND_HALF_PRE,
    STAGE_NORMAL_SECOND_HALF,
    STAGE_EXTRA_TIME_BREAK,
    STAGE_EXTRA_FIRST_HALF_PRE,
    STAGE_EXTRA_FIRST_HALF,
    STAGE_EXTRA_HALF_TIME,
    STAGE_EXTRA_SECOND_HALF_PRE,
    STAGE_EXTRA_SECOND_HALF,
    STAGE_PENALTY_SHOOTOUT_BREAK,
    STAGE_PENALTY_SHOOTOUT,
    STAGE_POST_GAME
}

enum gameCommand {
    COMMAND_HALT,
    COMMAND_STOP,
    COMMAND_NORMAL_START,
    COMMAND_FORCE_START,
    COMMAND_PREPARE_KICKOFF_YELLOW,
    COMMAND_PREPARE_KICKOFF_BLUE,
    COMMAND_PREPARE_PENALTY_YELLOW,
    COMMAND_PREPARE_PENALTY_BLUE,
    COMMAND_DIRECT_FREE_YELLOW,
    COMMAND_DIRECT_FREE_BLUE,
    COMMAND_INDIRECT_FREE_YELLOW,
    COMMAND_INDIRECT_FREE_BLUE,
    COMMAND_TIMEOUT_YELLOW,
    COMMAND_TIMEOUT_BLUE,
    COMMAND_BALL_PLACEMENT_YELLOW,
    COMMAND_BALL_PLACEMENT_BLUE,
    // Deprecated commands
    COMMAND_GOAL_YELLOW,
    COMMAND_GOAL_BLUE
}

export class Referee {
    stage: gameStage = gameStage.STAGE_NORMAL_FIRST_HALF_PRE;
    stage_time_left: number;
    command: gameCommand = gameCommand.COMMAND_HALT;
    command_counter: number = 0;
    command_timestamp: number = 0; // May need to write a handler for this
    
    yellow: TeamInfo;
    blue: TeamInfo;

    designatedPosition: Point;
    blue_team_on_positive_half: boolean;
    next_command: gameCommand;

    current_action_time_remaining: number;
    
    constructor() {
        this.blue = new TeamInfo();
    }

    // TODO: Add game_events[]
}
