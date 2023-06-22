import { Field } from "@/field"
import * as PIXI from "pixi.js";

// Overlay Types
export class Overlay {
    id: string
    ns: string
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

    pixelsPerMeter: number = 140;
    canvasScale: number = 1; // Scales the size of the canvas in the window
    
    constructor(id: string, msg: any) {
        this.id = id;
    	for (const member of Object.getOwnPropertyNames(msg)) {
            this[member] = msg[member];
        }
    }

    update(overlay: PIXI.Container, underlay: PIXI.Container) {
        let container = underlay;
        if (this.depth == 0) {
            container = overlay;
        }

        // this could get slow if we have hundreds of overlays, hopefully its not a problem
        let graphic = container.getChildByName(this.id);
    
        if (graphic) {
            // There might be a way to improve performance if we can confirm that
            // we are just translating the overlay without changing its internal points
            graphic.clear();
            this.draw(graphic);
        } else {
            console.log("creating graphic");
            graphic = new PIXI.Graphics();
            graphic.name = this.id;
            this.draw(graphic);
            container.addChild(graphic);
        }
    }

    draw(graphic: PIXI.Container) {
        graphic.position.x = this.position.x;
        graphic.position.y = this.position.y;
        
        switch(this.type) {
            // POINT
            case 0:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(0, this.stroke_color);
                graphic.drawEllipse(0, 0, this.pixelsPerMeter/30, this.pixelsPerMeter/30);
                graphic.endFill();
                break;
            // LINE
            case 1:
                if (this.points.length >= 2) {
                    graphic.lineStyle(this.stroke_width, this.stroke_color);
                    graphic.moveTo(this.pixelsPerMeter*this.points[0].x, this.pixelsPerMeter*this.points[0].y);
                    for (var i = 1; i < this.points.length; i++) {
                        graphic.lineTo(this.pixelsPerMeter*this.points[i].x, this.pixelsPerMeter*this.points[i].y);
                    }
                }
                break;
            // RECTANGLE
            case 2:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.drawRect(-this.pixelsPerMeter*this.scale.x/2, -this.pixelsPerMeter*this.scale.y/2, this.pixelsPerMeter*this.scale.x, this.pixelsPerMeter*this.scale.y);
                graphic.endFill();
                break;
            // ELLIPSE
            case 3:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.drawEllipse(0, 0, this.pixelsPerMeter*this.scale.x, this.pixelsPerMeter*this.scale.y);
                graphic.endFill();
                break;
            // POLYGON
            case 4:
                if (this.points.length >= 2){
                    graphic.moveTo(this.pixelsPerMeter*this.points.at(-1).x, this.pixelsPerMeter*this.points.at(-1).y);
                    graphic.beginFill(this.fill_color);
                    graphic.lineStyle(this.stroke_width, this.stroke_color);
                    for (const point of this.points) {
                        graphic.lineTo(this.pixelsPerMeter*point.x, this.pixelsPerMeter*point.y);
                    }
                    graphic.endFill();
                }
                break;
            // TEXT
            case 5:
                ctx.textAlign = "center";
                ctx.textBaseline = "middle";
                ctx.font = overlay.stroke_width + "sans-serif";
                ctx.fillText(overlay.text, 0, 0);
                break;
            // MESH
            case 6:
                // I think I can use a PIXI filter to do this more efficiently
                break;
            // CUSTOM
            case 7:
                // TODO: This is probably a very low priority to implement
                break;
        }
    }       
}
