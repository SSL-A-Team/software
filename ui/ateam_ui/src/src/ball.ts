import { RenderConfig } from "@/state";
import ROSLIB from "roslib";
import * as PIXI from "pixi.js";

export class Ball {
    pose: Pose;
    twist: Twist;
    accel: Accel;

    visible: boolean = true;

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
    // container.visible = ball.visible;

    let invisibleBall = container.getChildByName("invisibleBall") as PIXI.Graphics;
    let visibleBall = container.getChildByName("visibleBall") as PIXI.Graphics;
    if (invisibleBall) {
        invisibleBall.visible = !ball.visible;
    }

    if (visibleBall) {
        visibleBall.visible = ball.visible;
    }

}

export function drawBall(ball: Ball, container: PIXI.Container, renderConfig: RenderConfig) {
    // TODO: figure out how to pass scale around
    const scale = renderConfig.scale;

    const invisibleBall = new PIXI.Graphics();
    invisibleBall.name = "invisibleBall";

    const visibleBall = new PIXI.Graphics();
    visibleBall.name = "visibleBall";

    invisibleBall.beginFill("orange", 0.0);
    invisibleBall.lineStyle(2, "orange", 1.0);
    invisibleBall.drawCircle(0, 0, .022 * scale);

    visibleBall.beginFill("orange", 1.0);
    visibleBall.lineStyle(2, "orange", 0.0);
    visibleBall.drawCircle(0, 0, .022 * scale);

    container.addChild(invisibleBall);
    container.addChild(visibleBall);
    container.position.x = ball.pose.position.x * scale;
    container.position.y = -ball.pose.position.y * scale;
    // container.visible = ball.visible
}