import { TeamColor } from "@/team"
import ROSLIB from "roslib"

// Types used for team data, our team and the opponent team intentionally both use the same class
// with a place for data about robot status, active plays, etc so that we can use a single UI to control
// two instances of our software playing against each other for testing

export class RobotStatus {
    connected: boolean
    kickerAvail: boolean
    chipperAvail: boolean
    batteryLevel: number
    name: string
    message: string
}

export class Robot {
    id: number;
    visible: boolean;
    team: TeamColor;
    pose: Pose;
    twist: Twist;
    accel: Accel;
    status: RobotStatus | null;

    constructor(id:number, visible:boolean, team:TeamColor, pose:Pose) {
        this.id = id;
        this.visible = visible;
        this.team = team;
        this.pose = pose;
    }

    rotation(): number {
        return 2*Math.acos(this.pose.orientation.z) * 180/Math.PI;
    }
    
    setRotation(degrees: number) {
        this.pose.orientation.z = Math.sin(degrees/2 * Math.PI/180);
        this.pose.orientation.w = Math.cos(degrees/2 * Math.PI/180);
    }

    draw(ctx: any) {
        // TODO: figure out how to pass scale around
        const scale = 300;
        const radius = .09;
        const sr = scale*radius;

        const start = ((this.rotation()-50)/180)*Math.PI;
        const end =  ((this.rotation()+230)/180)*Math.PI;

        ctx.beginPath();
        ctx.arc(-sr, -sr, sr, start, end);
        ctx.closePath();
        ctx.fillStrokeShape(this);

        ctx.fillStyle = this.team == TeamColor.Yellow ? "black" : "white";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.font = "29px sans-serif";
        ctx.fillText(this.id, -sr, -sr + 3);
    }   
}

