import { Robot } from "@/robot"
import { Overlay } from "@/overlay"
import { TeamColor } from "@/team"
import { RenderConfig, AppState } from "@/state"
import ROSLIB from "roslib"
import * as PIXI from "pixi.js"
import { Viewport } from 'pixi-viewport'

// Types used for field data and overlays

export class FieldDimensions {
    length: number = 12;
    width: number = 9;
    border: number = 0.7;
    lineWidth: number = 0.01;
    goalWidth: number = 1.8;
    goalDepth: number = 0.18;
    goalHeight: number = 0.16;
    penaltyShort: number = 1.8;
    penaltyLong: number = 3.6;
    centerRadius: number = 0.5;
    centerDiameter: number = 1;
    goalFlat: number = 0.5;
    floorLength: number = 13.4;
    floorWidth: number = 10.4
}

export class FieldSidedInfo {
    defense_area_corners: Point[];
    goal_corners: Point[];
}


export class Field {
    fieldDimensions: FieldDimensions;
    overlays: Overlay[];

    constructor() {
        this.fieldDimensions = new FieldDimensions();
        this.overlays = [];
    }

    drawSideIgnoreOverlay(fieldUI: PIXI.Container, state: AppState) {
        const scale = state.renderConfig.scale;

        let ignoreOverlay = fieldUI.getChildByName("ignoreOverlay") as PIXI.Graphics;
        if (!ignoreOverlay) {
            ignoreOverlay = new PIXI.Graphics;
            ignoreOverlay.name = "ignoreOverlay";
            ignoreOverlay.visible = false;
            fieldUI.addChild(ignoreOverlay);
        }

        ignoreOverlay.clear();
        ignoreOverlay.beginFill("#3b3b3bFF");
        ignoreOverlay.drawRect(0,
                      -scale * (this.fieldDimensions.width/2 + this.fieldDimensions.border),
                      scale * (this.fieldDimensions.length/2 + this.fieldDimensions.border),
                      scale * (this.fieldDimensions.width + 2*this.fieldDimensions.border),
                     );


        let hoverOverlay = fieldUI.getChildByName("hoverOverlay") as PIXI.Graphics;
        if (!hoverOverlay) {
            hoverOverlay = new PIXI.Graphics;
            hoverOverlay.name = "hoverOverlay";
            hoverOverlay.visible = false;
            fieldUI.addChild(hoverOverlay);
        }

        hoverOverlay.clear();
        hoverOverlay.beginFill("#3b3b3bBF");
        hoverOverlay.drawRect(0,
                      -scale * (this.fieldDimensions.width/2 + this.fieldDimensions.border),
                      scale * (this.fieldDimensions.length/2 + this.fieldDimensions.border),
                      scale * (this.fieldDimensions.width + 2*this.fieldDimensions.border),
                     );

    }

    drawFieldLines(fieldLines: PIXI.Graphics, state: AppState) {
        const scale = state.renderConfig.scale;

        fieldLines.clear();

        // Draw the Field Lines
        fieldLines.lineStyle(4, 0xFFFFFF);

        // Field Outline
        fieldLines.drawRect(-scale * this.fieldDimensions.length / 2,
            -scale * this.fieldDimensions.width / 2,
            this.fieldDimensions.length * scale,
            this.fieldDimensions.width * scale
        );

        // Center Circle
        fieldLines.drawCircle(0, 0, scale * this.fieldDimensions.centerRadius);

        // Width Center Line
        fieldLines.moveTo(0, -scale * this.fieldDimensions.width / 2);
        fieldLines.lineTo(0, scale * this.fieldDimensions.width / 2);

        // Length Center Line
        fieldLines.moveTo(-scale * this.fieldDimensions.length / 2, 0);
        fieldLines.lineTo(scale * this.fieldDimensions.length / 2, 0);

        // Team Goal Boxes
        for (const color in state.world.teams) {
            const team = state.world.teams[color];
            const direction = (color == state.world.team ? -1 : 1); // always draw our goal on the left
            const goalX = direction * scale * this.fieldDimensions.length / 2

            // Goal Box
            fieldLines.lineStyle(4, 0xFFFFFF);
            fieldLines.moveTo(goalX, -scale * this.fieldDimensions.penaltyLong / 2);
            fieldLines.lineTo(goalX - direction * scale * this.fieldDimensions.penaltyShort, -scale * this.fieldDimensions.penaltyLong / 2);
            fieldLines.lineTo(goalX - direction * scale * this.fieldDimensions.penaltyShort, scale * this.fieldDimensions.penaltyLong / 2);
            fieldLines.lineTo(goalX, scale * this.fieldDimensions.penaltyLong / 2);

            // Goal
            fieldLines.lineStyle(4, color);
            fieldLines.moveTo(goalX, -scale * this.fieldDimensions.goalWidth / 2);
            fieldLines.lineTo(goalX + direction * scale * this.fieldDimensions.goalDepth, -scale * this.fieldDimensions.goalWidth / 2);
            fieldLines.lineTo(goalX + direction * scale * this.fieldDimensions.goalDepth, scale * this.fieldDimensions.goalWidth / 2);
            fieldLines.lineTo(goalX, scale * this.fieldDimensions.goalWidth / 2);
        }
    }

    initializePixi(app: PIXI.Application, state: AppState) {

        // create viewport
        const viewport = new Viewport({
            screenWidth: 1876,
            screenHeight: 1456,
            worldWidth: 1876,
            worldHeight: 1456,
            passiveWheel: false,
            stopPropagation: true,
            disableOnContextMenu: true,

            events: app.renderer.events // the interaction module is important for wheel to work properly when renderer.view is placed or scaled
        })

        // add the viewport to the stage
        viewport.name = "viewport";
        const offsetX = app.screen.width / 2;
        const offsetY = app.screen.height / 2;
        viewport.position.set(offsetX, offsetY);
        app.stage.addChild(viewport);

        // activate plugins
        viewport
            .drag({
                mouseButtons: "left"
            })
            .pinch()
            .wheel()

        const ballVelLine = new PIXI.Graphics;
        ballVelLine.name = "ballVelLine";
        app.stage.addChild(ballVelLine);


        const fieldLines = new PIXI.Graphics();
        fieldLines.name = "fieldLines";

        const robots = new PIXI.Container();
        robots.name = "robots";

        const ball = new PIXI.Container();
        ball.name = "ball";

        const underlay = new PIXI.Container();
        underlay.name = "underlay";
        const overlay = new PIXI.Container();
        overlay.name = "overlay";

        const fieldUI = new PIXI.Container();
        fieldUI.name = "fieldUI";

        // Field Lines
        this.drawFieldLines(fieldLines, state);

        // Rectangle that covers the ignored side of the field
        this.drawSideIgnoreOverlay(fieldUI, state);

        // Robots
        // surely there is a more elegant way to do this
        for (const robot of Object.entries(state.world.teams).map(i => { return i[1].robots }).flat()) {
            robot.draw(robots, state.renderConfig);
        }

        // Ball
        state.world.ball.draw(ball, state.renderConfig);

        viewport.addChild(fieldLines);
        viewport.addChild(underlay);
        viewport.addChild(robots);
        viewport.addChild(ball);
        viewport.addChild(overlay);
        viewport.addChild(fieldUI);
    }

    update(app: PIXI.Application, state: AppState) {

        const viewport = app.stage.getChildByName("viewport") as Viewport;

        // this also seems like an inefficient way to do this
        const robotArray = Object.entries(state.world.teams).map(i => { return i[1].robots }).flat()
        const robots = viewport.getChildByName("robots").children;
        for (var i = 0; i < robotArray.length; i++) {
            if (i != state.draggedRobot) {
                const robot = robots[i] as PIXI.Container;
                robotArray[i].update(robot, state.renderConfig);
            }
        }

        state.world.ball.update(viewport.getChildByName("ball").children[0] as PIXI.Container, state.renderConfig);

        for (const id in this.overlays) {
            const should_delete = this.overlays[id].update(viewport.getChildByName("overlay"), viewport.getChildByName("underlay"), state.renderConfig);

            if (should_delete) {
                delete this.overlays[id];
            }
        }

    }
}

