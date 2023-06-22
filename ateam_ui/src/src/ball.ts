import { Team, TeamColor } from "@/team"
import { RenderConfig } from "@state"
import { Field } from "@/field"
import ROSLIB from "roslib"
import * as PIXI from "pixi.js";

export class Ball {
    visible: boolean = true;
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

    update(container: PIXI.Container, renderConfig: RenderConfig) {
        const scale = renderConfig.scale;
        container.position.x = this.pose.position.x * scale;
        container.position.y = this.pose.position.y * scale;
        container.visible = this.visible;
    }

    draw(container: PIXI.Container, renderConfig: RenderConfig) {
        // TODO: figure out how to pass scale around
        const scale = renderConfig.scale;

        const graphic = new PIXI.Graphics();

        graphic.beginFill("orange");
        graphic.drawCircle(0, 0, .022 * scale);

        container.addChild(graphic);
        container.position.x = this.pose.position.x * scale;
        container.position.y = this.pose.position.y * scale;
        container.visible = this.visible
    }
}
