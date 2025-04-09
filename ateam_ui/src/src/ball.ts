import { RenderConfig } from "@/state";
import ROSLIB from "roslib";
import * as PIXI from "pixi.js";

export class Ball {
    visible: boolean = true;
    pose: Pose;
    twist: Twist;
    accel: Accel;

    constructor() {
        this.pose = new ROSLIB.Pose({
            position: {
                x: 0,
                y: 0,
                z: 0
            }
        })
    }
}

export function updateBall(ball: Ball, container: PIXI.Container, renderConfig: RenderConfig) {
    const scale = renderConfig.scale;
    container.position.x = ball.pose.position.x * scale;
    container.position.y = -ball.pose.position.y * scale;
    container.visible = ball.visible;
}

export function drawBall(ball: Ball, container: PIXI.Container, renderConfig: RenderConfig) {
    // TODO: figure out how to pass scale around
    const scale = renderConfig.scale;

    const graphic = new PIXI.Graphics();

    graphic.beginFill("orange");
    graphic.drawCircle(0, 0, .022 * scale);

    container.addChild(graphic);
    container.position.x = ball.pose.position.x * scale;
    container.position.y = -ball.pose.position.y * scale;
    container.visible = ball.visible
}