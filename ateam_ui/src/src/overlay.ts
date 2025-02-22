import { RenderConfig } from "@/state"
import * as PIXI from "pixi.js"
import { Buffer } from "buffer";

export enum OverlayType {
    Point = 0,
    Line,
    Rectangle,
    Ellipse,
    Polygon,
    Text,
    Heatmap,
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
    heatmap_data: Uint8Array
    heatmap_alpha: Uint8Array
    heatmap_resolution_width: number
    heatmap_resolution_height: number
    text: string | null
    depth: number = 0
    start_angle: number
    end_angle: number

    lifetime_end: number
    check_other_depth: boolean
    heatmap_texture: PIXI.Texture

    constructor(id: string, msg: any, check_other_depth: boolean = false) {
        this.id = id;
        this.check_other_depth = check_other_depth;

        for (const member of Object.getOwnPropertyNames(msg)) {
            if (member === "heatmap_data" || member == "heatmap_alpha") {
                this[member] = Buffer.from(msg[member], 'base64');
            } else {
                this[member] = msg[member];
            }
        }

        // lifetime is falsey if it will live forever
        if (this.lifetime) {
            this.lifetime_end = Date.now() + this.lifetime;
        }

        if (this.type == OverlayType.Heatmap) {
            this.initializeHeatmapTexture();
        }
    }

    /**
     * @param timestamp time in millis
     * @param overlay foreground container
     * @param underlay background container
     * @param renderConfig field rendering properties
     * @returns true if this overlay should be deleted, false otherwise
     */
    update(timestamp: number, overlay: PIXI.Container, underlay: PIXI.Container, renderConfig: RenderConfig): boolean {

        // Handle if the overlay was moved between graphics containers
        if (this.check_other_depth) {
            const opposite_container = (this.depth) ? underlay : overlay;
            this.deleteGraphic(opposite_container);
            this.check_other_depth = false;
        }

        const container = (this.depth) ? overlay : underlay;
        if (this.isExpired(timestamp)) {
            this.deleteGraphic(container);
            return true;
        }
        this.draw(container, renderConfig);
        return false;
    }

    isExpired(timestamp: number): boolean {
        return this.lifetime_end && timestamp >= this.lifetime_end;
    }

    deleteGraphic(container: PIXI.Container) {
        let graphic = container.getChildByName(this.id) as PIXI.Graphics;
        if (!graphic) {
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

        graphic.clear()
        const scale = renderConfig.scale;

        graphic.position.x = scale * this.position.x;
        graphic.position.y = -scale * this.position.y;

        switch (this.type) {
            case OverlayType.Point:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(0, this.stroke_color);
                graphic.drawEllipse(0, 0, scale / 30, scale / 30);
                graphic.endFill();
                break;
            case OverlayType.Line:
                if (this.points.length >= 2) {
                    graphic.lineStyle(this.stroke_width, this.stroke_color);
                    graphic.moveTo(scale * this.points[0].x, -scale * this.points[0].y);
                    for (var i = 1; i < this.points.length; i++) {
                        graphic.lineTo(scale * this.points[i].x, -scale * this.points[i].y);
                    }
                }
                break;
            case OverlayType.Rectangle:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.drawRect(-scale * this.scale.x / 2, -scale * this.scale.y / 2, scale * this.scale.x, scale * this.scale.y);
                graphic.endFill();
                break;
            case OverlayType.Ellipse:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.drawEllipse(0, 0, scale * this.scale.x / 2, scale * this.scale.y / 2);
                graphic.endFill();
                break;
            case OverlayType.Polygon:
                if (this.points.length >= 2) {
                    graphic.moveTo(scale * this.points.at(-1).x, -scale * this.points.at(-1).y);
                    graphic.beginFill(this.fill_color);
                    graphic.lineStyle(this.stroke_width, this.stroke_color);
                    for (const point of this.points) {
                        graphic.lineTo(scale * point.x, -scale * point.y);
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

                {
                    let graphicChild = graphic.getChildByName("text");

                    if (graphicChild) {
                        graphicChild = text;
                    } else {
                        graphic.addChild(text);
                    }
                }

                break;
            case OverlayType.Heatmap:
                {
                    let graphicChild = graphic.getChildByName(this.id) as PIXI.Sprite;
                    if (!graphicChild) {
                        graphicChild = new PIXI.Sprite();
                        graphicChild.name = this.id;
                        graphic.addChild(graphicChild);
                    }

                    graphicChild.texture = this.heatmap_texture;
                    graphicChild.width = scale * this.scale.x;
                    graphicChild.height = scale * this.scale.y;
                    graphicChild.x = -scale * this.scale.x / 2;
                    graphicChild.y = -scale * this.scale.y / 2;
                }

                break;
            case OverlayType.Custom:
                // TODO: This is probably a very low priority to implement
                break;
            case OverlayType.Arc:
                graphic.beginFill(0, 0);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.arc(0, 0, scale * this.scale.x / 2, -this.start_angle + renderConfig.angle, -this.end_angle + renderConfig.angle, true);
                graphic.endFill();
                break;
        }

        container.addChild(graphic);
    }

    initializeHeatmapTexture() {
        const bytes_per_pixel = 4;
        const num_pixels = this.heatmap_resolution_width * this.heatmap_resolution_height;
        let buffer = new Uint8Array(num_pixels * bytes_per_pixel);

        if (!this.heatmap_data) {
            throw new Error("No heatmap data.");
        }

        if (this.heatmap_data.length != num_pixels) {
            throw new Error("Heatmap data array does not have expected length.");
        }

        const has_alpha_layer = this.heatmap_alpha.length != 0;
        const has_full_alpha = this.heatmap_alpha.length == num_pixels;
        // Alpha array must be empty, one element, or matching size
        if (has_alpha_layer && this.heatmap_alpha.length != 1 && !has_full_alpha) {
            throw new Error("Heatmap alpha array does not have expected length.");
        }

        for (let dst_index = 0; dst_index < buffer.length; dst_index += 4) {
            const src_index = dst_index / 4;
            let alpha = 255;
            if (has_full_alpha) {
                alpha = this.heatmap_alpha[src_index];
            } else if (has_alpha_layer) {
                alpha = this.heatmap_alpha[0];
            }
            buffer[dst_index + 0] = this.heatmap_data[src_index];
            buffer[dst_index + 1] = 0;
            buffer[dst_index + 2] = 0;
            buffer[dst_index + 3] = alpha;
        }

        this.heatmap_texture = PIXI.Texture.fromBuffer(buffer, this.heatmap_resolution_width, this.heatmap_resolution_height, { scaleMode: PIXI.SCALE_MODES.NEAREST })
    }
}
