import { Robot } from "@/robot"
import ROSLIB from "roslib"

// Types used for team data, our team and the opponent team intentionally both use the same class
// with a place for data about robot status, active plays, etc so that we can use a single UI to control
// two instances of our software playing against each other for testing

export const maxRobots = 16;
export enum TeamColor {
    Yellow = "yellow",
    Blue = "blue"
}

export class TeamInfo {
    name: string
    score: number
    red_cards: number
    yellow_card_times: number[]
    yellow_cards: number
    timeouts: number
    timeout_time: number
    goalkeeper: number
    foul_counter: number
    ball_placement_failures: number
    can_place_ball: boolean
    max_allowed_bots: number
    bot_substitution_intent: boolean
    ball_placement_failures_reached: boolean
}

export class Team {
    name: string;
    color: TeamColor;
    defending: number; // +1 or -1 for defending positive or negative field coordinates
    robots: Robot[];

    //TODO: figure out how to store the current DAG(t) or whatever play structure we use

    constructor(name:string, color:TeamColor, defending:number) {
        this.color = color;
        this.defending = defending;

        this.robots = [];

        // initialize robots
        const numVisible = 11;
        for (let i = 0; i < maxRobots; i++) {
        
            let visible = (i < numVisible);
            let pose = new ROSLIB.Pose({
                position: {
                    x: 0.4 * this.defending,
                    y: 0.23 * (i - (numVisible-1)/2),
                    z: 0
                },
                orientation: {
                    x: 0,
                    y: 0,
                    z: 0,
                    w: 1
                }
            })
        
            this.robots.push(new Robot(i, visible, this.color, pose));
        }
    }
}
