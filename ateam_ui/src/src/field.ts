import ROSLIB from "roslib"
import * as PIXI from 'pixi.js';

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
    pixelsPerMeter: number = 50;
    canvasScale: number = 0.25; // Scales the size of the canvas in the window
    fieldDimensions: FieldDimensions;
    overlays: Overlay[];

    constructor() {
        this.fieldDimensions = new FieldDimensions();
        this.overlays = [];
    }

    initializePixi(app: PIXI.Application) {
        console.log("initialize pixi");
        const fieldLines = new PIXI.Graphics();

        const robots = new PIXI.Container();
        const ball = new PIXI.Container();

        const underlay = new PIXI.Container();
        const overlay = new PIXI.Container();

        // Draw the Field Lines
        fieldLines.lineStyle(2, 0xFFFFFF, 2);
        fieldLines.drawRect(0, 0,
                            this.fieldDimensions.width * this.pixelsPerMeter,
                            this.fieldDimensions.length * this.pixelsPerMeter
                           );

        app.stage.addChild(fieldLines);
        app.stage.addChild(robots);
        app.stage.addChild(ball);
        app.stage.addChild(underlay);
        app.stage.addChild(overlay);
    }

    update(app: PIXI.Application) {
        // update field dimensions (probably want to break this into its own function)
        // update robot locations
        // update ball locations

        for (var overlay of this.overlays) {
            // handle overlays
        }
    }
}

