import { TeamColor} from "@/team"
import { WorldState } from "@/state"
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

// Overlay Types
export class Overlay {
    namespace: string
    name: string
    visible: boolean
    type: number
    command: number
    position: Point
    scale: Point
    stroke_color: string
    fill_color: string
    stroke_width: number
    lifetime: number
    points: Point[] | null
    mesh: Mesh1d[] | null
    mesh_alpha: Mesh1d[] | null
    text: string | null
    depth: number
}

export class Field {
    pixelsPerMeter: number = 140;
    canvasScale: number = 1; // Scales the size of the canvas in the window
    fieldDimensions: FieldDimensions;
    overlays: Overlay[];

    constructor() {
        this.fieldDimensions = new FieldDimensions();
        this.overlays = [];
    }

    initializePixi(app: PIXI.Application, state: WorldState) {

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

        // Draw the Field Lines
        fieldLines.lineStyle(4, 0xFFFFFF);

        // Field Outline
        fieldLines.drawRect(this.pixelsPerMeter * this.fieldDimensions.border - offsetX,
                            this.pixelsPerMeter * this.fieldDimensions.border - offsetY,
                            this.fieldDimensions.length * this.pixelsPerMeter,
                            this.fieldDimensions.width * this.pixelsPerMeter
                           );

        // Center Circle
        fieldLines.drawCircle(0, 0, this.pixelsPerMeter * this.fieldDimensions.centerRadius);

        // Width Center Line
        fieldLines.moveTo(0, -this.pixelsPerMeter * this.fieldDimensions.width/2);
        fieldLines.lineTo(0, this.pixelsPerMeter * this.fieldDimensions.width/2);

        // Length Center Line
        fieldLines.moveTo(-this.pixelsPerMeter * this.fieldDimensions.length/2, 0);
        fieldLines.lineTo(this.pixelsPerMeter * this.fieldDimensions.length/2, 0);

        // Team Goal Boxes
        for (const color in state.world.teams) {
            const team = state.world.teams[color];
            const goalX = team.defending * this.pixelsPerMeter * this.fieldDimensions.length/2

            // Goal Box
            fieldLines.lineStyle(4, 0xFFFFFF);
            fieldLines.moveTo(goalX, -this.pixelsPerMeter * this.fieldDimensions.goalWidth);
            fieldLines.lineTo(goalX - team.defending * this.pixelsPerMeter * this.fieldDimensions.goalWidth, -this.pixelsPerMeter * this.fieldDimensions.goalWidth);
            fieldLines.lineTo(goalX - team.defending * this.pixelsPerMeter * this.fieldDimensions.goalWidth, this.pixelsPerMeter * this.fieldDimensions.goalWidth);
            fieldLines.lineTo(goalX, this.pixelsPerMeter * this.fieldDimensions.goalWidth);

            // Goal
            fieldLines.lineStyle(4, color);
            fieldLines.moveTo(goalX, -this.pixelsPerMeter * this.fieldDimensions.goalWidth/2);
            fieldLines.lineTo(goalX + team.defending * this.pixelsPerMeter * this.fieldDimensions.goalDepth, -this.pixelsPerMeter * this.fieldDimensions.goalWidth/2);
            fieldLines.lineTo(goalX + team.defending * this.pixelsPerMeter * this.fieldDimensions.goalDepth, this.pixelsPerMeter * this.fieldDimensions.goalWidth/2);
            fieldLines.lineTo(goalX, this.pixelsPerMeter * this.fieldDimensions.goalWidth/2);
        }

        // Robots
        // surely there is a more elegant way to do this
        for (const robot of Object.entries(state.world.teams).map(i => {return i[1].robots}).flat()) {
            robot.draw(robots);
        }

        state.world.ball.draw(ball);

        app.stage.addChild(fieldLines);
        app.stage.addChild(underlay);
        app.stage.addChild(robots);
        app.stage.addChild(ball);
        app.stage.addChild(overlay);
    }

    update(app: PIXI.Application, state: WorldState) {
        // update field dimensions (probably want to break this into its own function)

        // this also seems like an inefficient way to do this
        const robotArray = Object.entries(state.world.teams).map(i => {return i[1].robots}).flat()
        const robots = app.stage.getChildByName("robots").children;
        for (var i = 0; i < robotArray.length; i++) {
            robotArray[i].update(robots[i]);
        }

        state.world.ball.update(app.stage.getChildByName("ball").children[0]);

        //TODO: handle overlays
    }
}

