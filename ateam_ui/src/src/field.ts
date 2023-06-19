import ROSLIB from "roslib"

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
    pixelsPerMeter: number = 300;
    canvasScale: number = 0.25; // Scales the size of the canvas in the window
    fieldDimensions: FieldDimensions;
    overlays: Overlay[];

    constructor() {
        this.fieldDimensions = new FieldDimensions();
    }
    
    drawField(ctx: HTMLCanvasElement) {
        // Background
        // rectangle field outline
        // center circle
        // center line
        // goal boxes
        // goal
    }

    drawOverlays( underCtx: HTMLCanvasElement, overCtx: HTMLCanvasElement) {
        for (var overlay of this.overlays) {
            let ctx = overCtx;
            if (overlay.depth == 0) {
                ctx = underCtx;
            }

            // handle overlays
        }
    }
}

