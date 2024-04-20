import { RenderConfig } from "@/state"
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
    start_angle: number
    end_angle: number

    lifetime_end: number;
    
    constructor(id: string, msg: any) {
        this.id = id;
    	for (const member of Object.getOwnPropertyNames(msg)) {
            this[member] = msg[member];
        }

        // lifetime is falsey if it will live forever
        if (this.lifetime) {
            this.lifetime_end = Date.now() + this.lifetime;
        }
    }

    update(overlay: PIXI.Container, underlay: PIXI.Container, renderConfig: RenderConfig) {
        let container = underlay;
        if (this.depth == 0) {
            container = overlay;
        }

        // this could get slow if we have hundreds of overlays, hopefully its not a problem
        let graphic = container.getChildByName(this.id) as PIXI.Graphics;
    
        if (graphic) {
            // There might be a way to improve performance if we can confirm that
            // we are just translating the overlay without changing its internal points
            graphic.clear();
            if (!this.lifetime_end || Date.now() < this.lifetime_end) {
                this.draw(graphic, renderConfig);
            }
        } else {
            graphic = new PIXI.Graphics();
            graphic.name = this.id;
            this.draw(graphic, renderConfig);
            container.addChild(graphic);
        }
    }

    draw(graphic: PIXI.Graphics, renderConfig: RenderConfig) {
        const scale = renderConfig.scale;

        graphic.position.x = scale * this.position.x;
        graphic.position.y = -scale * this.position.y;
        
        switch(this.type) {
            // POINT
            case 0:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(0, this.stroke_color);
                graphic.drawEllipse(0, 0, scale/30, scale/30);
                graphic.endFill();
                break;
            // LINE
            case 1:
                if (this.points.length >= 2) {
                    graphic.lineStyle(this.stroke_width, this.stroke_color);
                    graphic.moveTo(scale*this.points[0].x, -scale*this.points[0].y);
                    for (var i = 1; i < this.points.length; i++) {
                        graphic.lineTo(scale*this.points[i].x, -scale*this.points[i].y);
                    }
                }
                break;
            // RECTANGLE
            case 2:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.drawRect(-scale*this.scale.x/2, -scale*this.scale.y/2, scale*this.scale.x, scale*this.scale.y);
                graphic.endFill();
                break;
            // ELLIPSE
            case 3:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.drawEllipse(0, 0, scale*this.scale.x/2, scale*this.scale.y/2);
                graphic.endFill();
                break;
            // POLYGON
            case 4:
                if (this.points.length >= 2){
                    graphic.moveTo(scale*this.points.at(-1).x, -scale*this.points.at(-1).y);
                    graphic.beginFill(this.fill_color);
                    graphic.lineStyle(this.stroke_width, this.stroke_color);
                    for (const point of this.points) {
                        graphic.lineTo(scale*point.x, -scale*point.y);
                    }
                    graphic.endFill();
                }
                break;
            // TEXT
            case 5:
                const text = new PIXI.Text(this.text, {
                    fontSize: this.stroke_width,
                    fill: this.fill_color
                });
                text.anchor.set(0.5, 0.5);
                text.rotation = -renderConfig.angle; // offset the rotation of the canvas so the text always appears right side up

                graphic.addChild(text);
                break;
            // MESH
            case 6:
                // I think I can use a PIXI filter to do this more efficiently
                break;
            // CUSTOM
            case 7:
                // TODO: This is probably a very low priority to implement
                break;
            // ARC
            case 8:
                graphic.beginFill(0, 0);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.arc(this.position.x, this.position.y, scale*this.scale.x/2, this.start_angle, this.end_angle, true);
                graphic.endFill();
                break;
        }
    }       
}
