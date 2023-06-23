import { TeamColor } from "@/team"
import { RenderConfig } from "@/state"
import * as PIXI from "pixi.js"
import ROSLIB from "roslib"

// Types used for team data, our team and the opponent team intentionally both use the same class
// with a place for data about robot status, active plays, etc so that we can use a single UI to control
// two instances of our software playing against each other for testing

export class RobotStatus {
    connected: boolean
    message: string

    sequence_number: number
    robot_revision_major: number
    robot_revision_minor: number
    battery_level: number        // volts
    battery_temperature: number  // deg C
    power_error: boolean
    tipped_error: boolean
    breakbeam_error: boolean
    breakbeam_ball_detected: boolean
    accelerometer_0_error: boolean
    accelerometer_1_error: boolean
    gyroscope_0_error: boolean
    gyroscope_1_error: boolean
    motor_0_general_error: boolean
    motor_0_hall_error: boolean
    motor_1_general_error: boolean
    motor_1_hall_error: boolean
    motor_2_general_error: boolean
    motor_2_hall_error: boolean
    motor_3_general_error: boolean
    motor_3_hall_error: boolean
    motor_4_general_error: boolean
    motor_4_hall_error: boolean
    chipper_available: boolean
    kicker_available: boolean
    motor_0_temperature: number  // deg C
    motor_1_temperature: number  // deg C
    motor_2_temperature: number  // deg C
    motor_3_temperature: number  // deg C
    motor_4_temperature: number  // deg C
    kicker_charge_level: number  // volts
}

export class Robot {
    id: number;
    visible: boolean;
    team: TeamColor;
    pose: Pose;
    twist: Twist;
    accel: Accel;
    status: RobotStatus;

    constructor(id:number, visible:boolean, team:TeamColor, pose:Pose) {
        this.id = id;
        this.visible = visible;
        this.team = team;
        this.pose = pose;

        this.status = new RobotStatus();
    }

    rotation(): number {
        return (2.0*Math.acos(this.pose.orientation.z) * 180.0)/Math.PI;
    }
    
    setRotation(degrees: number) {
        this.pose.orientation.z = Math.sin(degrees/2 * Math.PI/180);
        this.pose.orientation.w = Math.cos(degrees/2 * Math.PI/180);
    }

    update(container: PIXI.Container, renderConfig: RenderConfig) {
        const scale = renderConfig.scale;
        container.position.x = this.pose.position.x * scale;
        container.position.y = this.pose.position.y * scale;
        container.getChildAt(0).angle = this.rotation();
    }

    draw(container: PIXI.Container, renderConfig: RenderConfig) {
        const scale = renderConfig.scale;
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

