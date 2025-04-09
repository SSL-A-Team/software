import { Team, TeamInfo, TeamColor } from "@/team";

export class GameProperty{
    name: String
    color: String
}

export enum GameStage {
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

export const GameStageProperties: GameProperty[] = [
    {name:"Normal First Half Pre", color:"yellow"},
    {name:"Normal First Half", color:"green"},
    {name:"Normal Half Time", color:"red"},
    {name:"Normal Second Half Pre", color:"yellow"},
    {name:"Normal Second Half", color:"green"},
    {name:"Extra Time Break", color:"red"},
    {name:"Extra First Half Pre", color:"yellow"},
    {name:"Extra First Half", color:"green"},
    {name:"Extra Half Time", color:"red"},
    {name:"Extra Second Half Pre", color:"yellow"},
    {name:"Extra Second Half", color:"green"},
    {name:"Penalty Shootout Break", color:"red"},
    {name:"Penalty Shootout", color:"green"},
    {name:"Post Game", color:"red"}
];

export enum GameCommand {
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
    COMMAND_GOAL_YELLOW, // Deprecated
    COMMAND_GOAL_BLUE, // Deprecated
    COMMAND_BALL_PLACEMENT_YELLOW,
    COMMAND_BALL_PLACEMENT_BLUE
}

export const GameCommandProperties: GameProperty[] = [
    {name: "Halt", color:"red"},
    {name: "Stop", color:"red"},
    {name: "Normal Start", color:"green"},
    {name: "Force Start", color:"green"},
    {name: "Prepare Kickoff Yellow", color:"yellow"},
    {name: "Prepare Kickoff Blue", color:"blue"},
    {name: "Prepare Penalty Yellow", color:"yellow"},
    {name: "Prepare Penalty Blue", color:"blue"},
    {name: "Direct Free Yellow", color:"yellow"},
    {name: "Direct Free Blue", color:"blue"},
    {name: "Indirect Free Yellow", color:"yellow"},
    {name: "Indirect Free Blue", color:"blue"},
    {name: "Timeout Yellow", color:"yellow"},
    {name: "Timeout Blue", color:"blue"},
    {name: "Ball Placement Yellow", color:"yellow"},
    {name: "Ball Placement Blue", color:"blue"},
    {name: "Goal Yellow", color:"yellow"},
    {name: "Goal Blue", color:"blue"}
];

// TODO: Add game_events[]
export class Referee {
    stage: GameStage = GameStage.STAGE_NORMAL_FIRST_HALF_PRE;
    stage_time_left: number;
    command: GameCommand = GameCommand.COMMAND_HALT;
    command_counter: number = 0;
    command_timestamp: number = 0;
    
    yellow: TeamInfo;
    blue: TeamInfo;

    designatedPosition: Point;
    blue_team_on_positive_half: boolean;
    next_command: GameCommand;

    current_action_time_remaining: number;

    constructor() {
        this.blue = new TeamInfo();
        this.yellow = new TeamInfo();
    }
}

export function getStageProperty(referee: Referee): GameProperty {
    return GameStageProperties[referee.stage];
}

export function getCommandProperty(referee: Referee): GameProperty {
    return GameCommandProperties[referee.command];
}