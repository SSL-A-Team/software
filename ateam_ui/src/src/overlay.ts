import { RenderConfig } from "@/state"
import { Field } from "@/field"
import * as PIXI from "pixi.js";
import { render } from "vue";

enum OverlayType {
    Point=0,
    Line,
    Rectangle,
    Ellipse,
    Polygon,
    Text,
    Mesh,
    Custom,
    Arc,
}

// Overlay Types
export class Overlay {
    id: string
    ns: string
    name: string
    visible: boolean = true
    type: OverlayType
    command: number
    position: Point
    scale: Point
    stroke_color: string = "black"
    fill_color: string = "black"
    stroke_width: number = 20
    lifetime: number
    points: Point[] | null
    mesh: Mesh1d[] | null
    mesh_alpha: Mesh1d[] | null
    text: string | null
    depth: number = 0
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

    /**
     * @param overlay background container
     * @param underlay foreground container
     * @param renderConfig field rendering properties
     * @returns true if this overlay should be deleted, false otherwise
     */
    update(overlay: PIXI.Container, underlay: PIXI.Container, renderConfig: RenderConfig): boolean {
        let container = underlay;
        if (this.depth == 0) {
            container = overlay;
        }
        if(this.isExpired()) {
            this.deleteGraphic(container);
            return true;
        }
        this.draw(container, renderConfig);
        return false;
    }

    isExpired(): boolean {
        return this.lifetime_end && Date.now() >= this.lifetime_end;
    }

    deleteGraphic(container: PIXI.Container) {
        let graphic = container.getChildByName(this.id) as PIXI.Graphics;
        if(!graphic) {
            return;
        }
        graphic.clear();
        container.removeChild(graphic);
    }

    draw(container: PIXI.Container, renderConfig: RenderConfig) {
        // this could get slow if we have hundreds of overlays, hopefully its not a problem
        let graphic = container.getChildByName(this.id) as PIXI.Graphics;

        if (!graphic) {
            graphic = new PIXI.Graphics();
            graphic.name = this.id;
        }

        const scale = renderConfig.scale;

        graphic.position.x = scale * this.position.x;
        graphic.position.y = -scale * this.position.y;
        
        switch(this.type) {
            case OverlayType.Point:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(0, this.stroke_color);
                graphic.drawEllipse(0, 0, scale/30, scale/30);
                graphic.endFill();
                break;
            case OverlayType.Line:
                if (this.points.length >= 2) {
                    graphic.lineStyle(this.stroke_width, this.stroke_color);
                    graphic.moveTo(scale*this.points[0].x, -scale*this.points[0].y);
                    for (var i = 1; i < this.points.length; i++) {
                        graphic.lineTo(scale*this.points[i].x, -scale*this.points[i].y);
                    }
                }
                break;
            case OverlayType.Rectangle:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.drawRect(-scale*this.scale.x/2, -scale*this.scale.y/2, scale*this.scale.x, scale*this.scale.y);
                graphic.endFill();
                break;
            case OverlayType.Ellipse:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.drawEllipse(0, 0, scale*this.scale.x/2, scale*this.scale.y/2);
                graphic.endFill();
                break;
            case OverlayType.Polygon:
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
            case OverlayType.Text:
                // TEXT IS WEIRD IN PIXI
                // The text is rendered as a new object and added
                // as a child to the graphics object

                const text = new PIXI.Text(this.text, {
                    fontSize: this.stroke_width,
                    fill: this.fill_color
                });
                text.name = "text"

                text.anchor.set(0.5, 0.5);
                text.rotation = -renderConfig.angle; // offset the rotation of the canvas so the text always appears right side up

                let graphicChild = graphic.getChildByName("text");

                if (graphicChild) {
                    graphicChild = text;
                } else {
                    graphic.addChild(text);
                }

                break;
            case OverlayType.Mesh:
                // I think I can use a PIXI filter to do this more efficiently
                break;
            case OverlayType.Custom:
                // TODO: This is probably a very low priority to implement
                break;
            case OverlayType.Arc:
                graphic.beginFill(0, 0);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.arc(0, 0, scale*this.scale.x/2, -this.start_angle + renderConfig.angle, -this.end_angle + renderConfig.angle, true);
                graphic.endFill();
                break;
        }

        container.addChild(graphic);
    }
}
