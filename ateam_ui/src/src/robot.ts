import { TeamColor } from "@/team"
import * as PIXI from "pixi.js"
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
        return (2.0*Math.acos(this.pose.orientation.z) * 180.0)/Math.PI;
    }
    
    setRotation(degrees: number) {
        this.pose.orientation.z = Math.sin(degrees/2 * Math.PI/180);
        this.pose.orientation.w = Math.cos(degrees/2 * Math.PI/180);
    }

    update(container: PIXI.Container) {
        const scale = 140;
        container.position.x = this.pose.position.x * scale;
        container.position.y = this.pose.position.y * scale;
        container.getChildAt(0).angle = this.rotation();
    }

    draw(container: PIXI.Container) {
        // TODO: figure out how to pass scale around
        const scale = 140;
        const radius = .09;
        const sr = scale*radius;

        const start = (-50/180)*Math.PI;
        const end =  (230/180)*Math.PI;

        const robot = new PIXI.Container();

        // Could possibly improve caching by using RenderTexture instead
        const graphic = new PIXI.Graphics();
        graphic.name = "robot";

        graphic.lineStyle(2, "black");
        graphic.beginFill(this.team);
        graphic.arc(0, 0, sr, start, end);
        graphic.closePath();
        graphic.endFill();

        // Maybe find a better font
        const text = new PIXI.Text(this.id, {
            fontSize: 16,
            fill: (this.team == TeamColor.Blue ? "white" : "black"),
        });
        text.name = "id";
        text.anchor.set(0.5, 0.5);

        robot.position.x = this.pose.position.x * scale;
        robot.position.y = this.pose.position.y * scale;
        graphic.angle = this.rotation();

        robot.addChild(graphic);
        robot.addChild(text);
        robot.visible = this.visible;

        container.addChild(robot);
    }   
}

