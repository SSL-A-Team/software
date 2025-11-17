import { TeamColor } from "@/team";
import { RenderConfig } from "@/state";
import ROSLIB from "roslib";
import * as PIXI from "pixi.js";
import { compressedArrayToPlayMap } from "./play";

// Types used for robot data including the feedback status

export enum ErrorLevel {
    None,
    Warning,
    Error,
    Critical
}

export class RobotStatus {

    sequence_number: number
    robot_revision_major: number
    robot_revision_minor: number

    power_error: boolean
    power_board_error: boolean
    battery_error: boolean
    battery_low: boolean
    battery_crit: boolean
    shutdown_pending: boolean
    tipped_error: boolean
    breakbeam_error: boolean
    breakbeam_ball_detected: boolean
    accelerometer_0_error: boolean
    accelerometer_1_error: boolean
    gyroscope_0_error: boolean
    gyroscope_1_error: boolean
    motor_fl_general_error: boolean
    motor_fl_hall_error: boolean
    motor_bl_general_error: boolean
    motor_bl_hall_error: boolean
    motor_br_general_error: boolean
    motor_br_hall_error: boolean
    motor_fr_general_error: boolean
    motor_fr_hall_error: boolean
    motor_drib_general_error: boolean
    motor_drib_hall_error: boolean
    kicker_board_error: boolean
    chipper_available: boolean
    kicker_available: boolean

    battery_percent: number
    kicker_charge_percent: number
}

export class Robot {
    id: number;
    team: TeamColor;
    status: RobotStatus;
    radio_connected: boolean = false;

    pose: Pose;
    twist: Twist;
    accel: Accel;

    // twist_body: Twist;

    visible: boolean;

    constructor(id: number, visible: boolean, team: TeamColor, pose: Pose) {
        this.id = id;
        this.visible = visible;
        this.team = team;
        this.pose = pose;

        this.status = new RobotStatus();
    }
}

export function isValid(robot: Robot): boolean {
    return robot.visible || robot.radio_connected;
}

export function rotation(robot: Robot): number {
    return (2.0*Math.acos(robot.pose.orientation.z) * 180.0)/Math.PI;
}

export function setRotation(robot: Robot, degrees: number) {
    robot.pose.orientation.z = Math.sin(degrees/2 * Math.PI/180);
    robot.pose.orientation.w = Math.cos(degrees/2 * Math.PI/180);
}

export function getErrorLevel(robot: Robot, sim: boolean): ErrorLevel {
    // Lost radio, no status to use for other warnings
    if (!robot.radio_connected) {
        return ErrorLevel.Warning;
    }

    // Critical

    if (!sim && (robot.status.power_error || robot.status.power_board_error || robot.status.battery_error)) {
        return ErrorLevel.Critical;
    }

    if (!sim && (robot.status.battery_crit)) {
        return ErrorLevel.Critical;
    }


    // Error

    // Motor General/Hall
    for (var i of ["fl", "bl", "br", "fr", "drib"]) {
        if (robot.status["motor_" + i + "_general_error"]) {
            return ErrorLevel.Error;
        }

        if (robot.status["motor_" + i + "_hall_error"]) {
            return ErrorLevel.Error;
        }
    }

    // Warning

    if (robot.status.battery_low) {
        return ErrorLevel.Warning;
    }

    // Robot tipped over, someone should probably go pick it up
    if (robot.status.tipped_error) {
        return ErrorLevel.Warning;
    }

    if (!sim && robot.status.kicker_board_error) {
        return ErrorLevel.Warning;
    }

    // None
    return ErrorLevel.None;
}

export function updateRobot(robot: Robot, container: PIXI.Container, renderConfig: RenderConfig) {
    const scale = renderConfig.scale;
    container.position.x = robot.pose.position.x * scale;
    container.position.y = -robot.pose.position.y * scale;
    container.getChildAt(0).angle = rotation(robot) - 90;
    container.visible = robot.visible;
}

export function drawRobot(robot: Robot, container: PIXI.Container, renderConfig: RenderConfig) {
    const robot_name = robot.team + "_robot" + robot.id;

    const scale = renderConfig.scale;
    const radius = .09;
    const sr = scale*radius;

    const start = (-50/180)*Math.PI;
    const end =  (230/180)*Math.PI;


    let robotContainer = container.getChildByName(robot_name) as PIXI.Container;
    if (!robotContainer) {
        robotContainer = new PIXI.Container();
        robotContainer.name = robot_name;
        robotContainer.eventMode = "dynamic";
    } else {
        robotContainer.removeChildren();
    }

    // Could possibly improve caching by using RenderTexture instead
    const graphic = new PIXI.Graphics();
    graphic.name = "robot";

    graphic.lineStyle(2, "black");
    graphic.beginFill(robot.team);
    graphic.arc(0, 0, sr, start, end);
    graphic.closePath();
    graphic.endFill();

    // Maybe find a better font
    const text = new PIXI.Text(robot.id, {
        fontSize: 16,
        fill: (robot.team == TeamColor.Blue ? "white" : "black"),
    });
    text.name = "id";
    text.anchor.set(0.5, 0.5);
    text.angle = -renderConfig.angle; // offset the rotation of the canvas so the text always appears right side up

    robotContainer.position.x = robot.pose.position.x * scale;
    robotContainer.y = -robot.pose.position.y * scale;
    graphic.angle = rotation(robot) - 90;

    robotContainer.addChild(graphic);
    robotContainer.addChild(text);
    robotContainer.visible = robot.visible;

    const oldContainer = container.getChildByName(robotContainer.name);
    if (oldContainer) {
        container.removeChild(oldContainer);
    }
    container.addChild(robotContainer);
}
