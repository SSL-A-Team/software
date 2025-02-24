import { TeamColor } from "@/team";
import { RenderConfig } from "@/state";
import * as PIXI from "pixi.js";

// Types used for robot data including the feedback status

export enum ErrorLevel {
    None,
    Warning,
    Error,
    Critical
}

export class RobotStatus {
    radio_connected: boolean = false;
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

    constructor(id: number, visible: boolean, team: TeamColor, pose: Pose) {
        this.id = id;
        this.visible = visible;
        this.team = team;
        this.pose = pose;

        this.status = new RobotStatus();
    }
}

export function isValid(robot: Robot): boolean {
    return robot.visible || robot.status.radio_connected;
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
    if (!robot.status.radio_connected) {
        return ErrorLevel.Warning;
    }

    // Critical

    // Battery Level TODO: find what level to use
    if (!sim && robot.status.battery_level <= 23.0) {
        return ErrorLevel.Critical;
    }

    // Kicker Voltage TODO: find max voltage
    if (!sim && robot.status.kicker_charge_level >= 220) {
        return ErrorLevel.Critical;
    }

    // Error

    // Motor General/Hall
    for (var i = 0; i < 5; i++) {
        if (robot.status["motor_" + i + "_general_error"]) {
            return ErrorLevel.Error;
        }

        if (robot.status["motor_" + i + "_hall_error"]) {
            return ErrorLevel.Error;
        }
    }

    // Warning

    // Robot tipped over, someone should probably go pick it up
    if (robot.status.tipped_error) {
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
    const scale = renderConfig.scale;
    const radius = .09;
    const sr = scale*radius;

    const start = (-50/180)*Math.PI;
    const end =  (230/180)*Math.PI;

    const robotContainer = new PIXI.Container();
    robotContainer.eventMode = "dynamic";

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

    container.addChild(robotContainer);
}   

