import ROSLIB from "roslib"
import { Team, TeamColor } from "@/team"
import { Field } from "@/field"

export class Ball {
    visible: boolean;
    pose: Pose;
    twist: Twist;
    accel: Accel;

    constructor() {
        // TODO: convert this to use the new coordinate system that centers the origin on our goal
        this.pose = new ROSLIB.Pose({
            position: {
                x: 0,
                y: 0,
                z: 0
            }
        })
    }
}
