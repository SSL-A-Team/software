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
                    z: -this.defending * Math.sqrt(2)/2,
                    w: Math.sqrt(2)/2
                }
            })
        
            this.robots.push(new Robot(i, visible, this.color, pose));
        }
    }
}
