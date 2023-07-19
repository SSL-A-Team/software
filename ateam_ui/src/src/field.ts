import { Robot } from "@/robot"
import { Overlay } from "@/overlay"
import { TeamColor} from "@/team"
import { RenderConfig, AppState } from "@/state"
import ROSLIB from "roslib"
import * as PIXI from "pixi.js";

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

export class Field {
    fieldDimensions: FieldDimensions;
    overlays: Overlay[];

    constructor() {
        this.fieldDimensions = new FieldDimensions();
        // this.overlays = [];
        this.overlays = [];
        this.overlays.push(new Overlay("shader_test",
          {
            id: "shader_test",
            ns: "nope",
            name: "shader_test",
            type: 6,
            position: {x: 0, y: 0},
            scale: {x: 12, y: 9},
            mesh: [{mesh1d: [1,4,5,5,6,7,8,1,7,4,14,1,34,1]},
                   {mesh1d: [1,4,5,5,6,7,8,1,7,3,14,1,34,1]},
                   {mesh1d: [1,4,5,5,6,7,8,1,7,4,14,1,34,1]},
                   {mesh1d: [1,4,5,40,6,7,8,1,7,4,14,1,34,1]},
                   {mesh1d: [1,4,14,5,6,7,8,1,3,4,14,1,34,1]},
                   {mesh1d: [1,4,14,5,6,7,8,1,7,4,14,1,34,1]},
                   {mesh1d: [1,4,14,5,6,7,8,1,7,4,14,1,34,1]},
                   {mesh1d: [1,4,5,5,6,7,8,1,7,4,14,1,34,1]}
                  ]
          }
        )
      )
    }

    drawFieldLines(fieldLines: PIXI.Graphics, state: AppState) {
        const scale = state.renderConfig.scale;

        fieldLines.clear();

        // Draw the Field Lines
        fieldLines.lineStyle(4, 0xFFFFFF);

        // Field Outline
        fieldLines.drawRect(-scale * this.fieldDimensions.length/2,
                            -scale * this.fieldDimensions.width/2,
                            this.fieldDimensions.length * scale,
                            this.fieldDimensions.width * scale
                           );

        // Center Circle
        fieldLines.drawCircle(0, 0, scale * this.fieldDimensions.centerRadius);

        // Width Center Line
        fieldLines.moveTo(0, -scale * this.fieldDimensions.width/2);
        fieldLines.lineTo(0, scale * this.fieldDimensions.width/2);

        // Length Center Line
        fieldLines.moveTo(-scale * this.fieldDimensions.length/2, 0);
        fieldLines.lineTo(scale * this.fieldDimensions.length/2, 0);

        // Team Goal Boxes
        for (const color in state.world.teams) {
            const team = state.world.teams[color];
            const direction = (color == state.world.team ? -1 : 1); // always draw our goal on the left
            const goalX = direction * scale * this.fieldDimensions.length/2

            // Goal Box
            fieldLines.lineStyle(4, 0xFFFFFF);
            fieldLines.moveTo(goalX, -scale * this.fieldDimensions.goalWidth);
            fieldLines.lineTo(goalX - direction * scale * this.fieldDimensions.goalWidth, -scale * this.fieldDimensions.goalWidth);
            fieldLines.lineTo(goalX - direction * scale * this.fieldDimensions.goalWidth, scale * this.fieldDimensions.goalWidth);
            fieldLines.lineTo(goalX, scale * this.fieldDimensions.goalWidth);

            // Goal
            fieldLines.lineStyle(4, color);
            fieldLines.moveTo(goalX, -scale * this.fieldDimensions.goalWidth/2);
            fieldLines.lineTo(goalX + direction * scale * this.fieldDimensions.goalDepth, -scale * this.fieldDimensions.goalWidth/2);
            fieldLines.lineTo(goalX + direction * scale * this.fieldDimensions.goalDepth, scale * this.fieldDimensions.goalWidth/2);
            fieldLines.lineTo(goalX, scale * this.fieldDimensions.goalWidth/2);
        }
    }

    initializePixi(app: PIXI.Application, state: AppState) {

        // Set origin to center of field
        const offsetX = app.screen.width/2;
        const offsetY = app.screen.height/2;
        app.stage.position.set(offsetX, offsetY);

        const fieldLines = new PIXI.Graphics();
        fieldLines.name = "fieldLines";

        const robots = new PIXI.Container();
        robots.name = "robots";

        const ball = new PIXI.Container();
        ball.name = "ball";
        //ball.eventMode = 'static';

        const underlay = new PIXI.Container();
        underlay.name = "underlay";
        const overlay = new PIXI.Container();
        overlay.name = "overlay";

        // Field Lines
        this.drawFieldLines(fieldLines, state);

        // Robots
        // surely there is a more elegant way to do this
        for (const robot of Object.entries(state.world.teams).map(i => {return i[1].robots}).flat()) {
            robot.draw(robots, state.renderConfig);
        }

        // Ball
        state.world.ball.draw(ball, state.renderConfig);

        app.stage.addChild(fieldLines);
        app.stage.addChild(underlay);
        app.stage.addChild(robots);
        app.stage.addChild(ball);
        app.stage.addChild(overlay);
    }

    update(app: PIXI.Application, state: AppState) {
        // TODO: figure out how to trigger field dimension update
        // drawFieldLines(app.stage.getChildByName("fieldLines");

        // this also seems like an inefficient way to do this
        const robotArray = Object.entries(state.world.teams).map(i => {return i[1].robots}).flat()
        const robots = app.stage.getChildByName("robots").children;
        for (var i = 0; i < robotArray.length; i++) {
                const robot = robots[i] as PIXI.Container;
                robotArray[i].update(robot, state.renderConfig);
        }

        state.world.ball.update(app.stage.getChildByName("ball").children[0] as PIXI.Container, state.renderConfig);

        for (const id in this.overlays) {
            this.overlays[id].update(app.stage.getChildByName("overlay"), app.stage.getChildByName("underlay"), state.renderConfig);
        }
    }
}

