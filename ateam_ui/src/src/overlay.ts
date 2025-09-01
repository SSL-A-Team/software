import { GraphicState, RenderConfig } from "@/state";
import * as PIXI from "pixi.js";
import { Buffer } from "buffer";
import { Vector3 } from "roslib";

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
    Arrows,
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
    scales: ROSLIB.Vector3[]
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

    lifetimeEnd: number
    checkOtherDepth: boolean

    constructor(id: string, msg: any, checkOtherDepth: boolean = false, timestamp: number) {
        this.id = id;
        this.checkOtherDepth = checkOtherDepth;

        for (const member of Object.getOwnPropertyNames(msg)) {
            if (member === "heatmap_data" || member == "heatmap_alpha") {
                this[member] = Buffer.from(msg[member], 'base64');
            } else {
                this[member] = msg[member];
            }
        }

        // lifetime is falsey if it will live forever
        if (this.lifetime) {
            this.lifetimeEnd = timestamp + this.lifetime;
        }
    }
}

/**
 * @param timestamp time in millis
 * @param overlay foreground container
 * @param underlay background container
 * @param renderConfig field rendering properties
 * @returns true if this overlay should be deleted, false otherwise
 */
export function updateOverlay(overlay: Overlay, timestamp: number, overlayContainer: PIXI.Container, underlayContainer: PIXI.Container, renderConfig: RenderConfig): boolean {

    // Handle if the overlay was moved between graphics containers
    if (overlay.checkOtherDepth) {
        const oppositeContainer = (overlay.depth) ? underlayContainer : overlayContainer;
        deleteOverlayGraphic(overlay, oppositeContainer);
        overlay.checkOtherDepth = false;
    }

    const container = (overlay.depth) ? overlayContainer : underlayContainer;
    if (isOverlayExpired(overlay, timestamp)) {
        deleteOverlayGraphic(overlay, container);
        return true;
    }
    drawOverlay(overlay, container, renderConfig);
    return false;
}

export function isOverlayExpired(overlay: Overlay, timestamp: number): boolean {
    return overlay.lifetimeEnd && timestamp >= overlay.lifetimeEnd;
}

export function deleteOverlayGraphic(overlay: Overlay, container: PIXI.Container) {
    let graphic = container.getChildByName(overlay.id) as PIXI.Graphics;
    if (!graphic) {
        return;
    }
    graphic.clear();
    container.removeChild(graphic);
}

export function drawOverlay(overlay: Overlay, container: PIXI.Container, renderConfig: RenderConfig) {
    // this could get slow if we have hundreds of overlays, hopefully its not a problem
    let graphic = container.getChildByName(overlay.id) as PIXI.Graphics;

    if (!graphic) {
        graphic = new PIXI.Graphics();
        graphic.name = overlay.id;
    }

    graphic.clear()
    const scale = renderConfig.scale;

    graphic.position.x = scale * overlay.position.x;
    graphic.position.y = -scale * overlay.position.y;

    switch (overlay.type) {
        case OverlayType.Point:
            graphic.beginFill(overlay.fill_color);
            graphic.lineStyle(0, overlay.stroke_color);
            graphic.drawEllipse(0, 0, scale / 30, scale / 30);
            graphic.endFill();
            break;
        case OverlayType.Line:
            if (overlay.points.length >= 2) {
                graphic.lineStyle(overlay.stroke_width, overlay.stroke_color);
                graphic.moveTo(scale * overlay.points[0].x, -scale * overlay.points[0].y);
                for (var i = 1; i < overlay.points.length; i++) {
                    graphic.lineTo(scale * overlay.points[i].x, -scale * overlay.points[i].y);
                }
            }
            break;
        case OverlayType.Rectangle:
            graphic.beginFill(overlay.fill_color);
            graphic.lineStyle(overlay.stroke_width, overlay.stroke_color);
            graphic.drawRect(-scale * overlay.scale.x / 2, -scale * overlay.scale.y / 2, scale * overlay.scale.x, scale * overlay.scale.y);
            graphic.endFill();
            break;
        case OverlayType.Ellipse:
            graphic.beginFill(overlay.fill_color);
            graphic.lineStyle(overlay.stroke_width, overlay.stroke_color);
            graphic.drawEllipse(0, 0, scale * overlay.scale.x / 2, scale * overlay.scale.y / 2);
            graphic.endFill();
            break;
        case OverlayType.Polygon:
            if (overlay.points.length >= 2) {
                graphic.moveTo(scale * overlay.points.at(-1).x, -scale * overlay.points.at(-1).y);
                graphic.beginFill(overlay.fill_color);
                graphic.lineStyle(overlay.stroke_width, overlay.stroke_color);
                for (const point of overlay.points) {
                    graphic.lineTo(scale * point.x, -scale * point.y);
                }
                graphic.endFill();
            }
            break;
        case OverlayType.Text:
            // TEXT IS WEIRD IN PIXI
            // The text is rendered as a new object and added
            // as a child to the graphics object

            const text = new PIXI.Text(overlay.text, {
                fontSize: overlay.stroke_width,
                fill: overlay.fill_color
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
                let graphicChild = graphic.getChildByName("heatmap") as PIXI.Sprite;
                if (!graphicChild) {
                    // This should never happen
                    return
                }

                graphicChild.width = scale * overlay.scale.x;
                graphicChild.height = scale * overlay.scale.y;
                graphicChild.x = -scale * overlay.scale.x / 2;
                graphicChild.y = -scale * overlay.scale.y / 2;
            }

            break;
        case OverlayType.Custom:
            // TODO: This is probably a very low priority to implement
            break;
        case OverlayType.Arc:
            graphic.beginFill(0, 0);
            graphic.lineStyle(overlay.stroke_width, overlay.stroke_color);
            graphic.arc(0, 0, scale * overlay.scale.x / 2, -overlay.start_angle + renderConfig.angle, -overlay.end_angle + renderConfig.angle, true);
            graphic.endFill();
            break;
        case OverlayType.Arrows:
            graphic.beginFill(0, 0);
            graphic.lineStyle(overlay.stroke_width, overlay.stroke_color);
            for(let i = 0; i < overlay.points.length; i++) {
                const startPoint = overlay.points[i];
                let endPoint = new Vector3;
                const vector = overlay.scales[i];
                endPoint.x = startPoint.x + vector.x;
                endPoint.y = startPoint.y + vector.y;

                graphic.moveTo(scale * startPoint.x, -scale * startPoint.y);
                graphic.lineTo(scale * endPoint.x, -scale * endPoint.y);

                const length = 0.1 * Math.sqrt(vector.x**2 + vector.y**2);
                const vector_angle = Math.atan2(vector.y, vector.x);
                const angle1 = vector_angle + (Math.PI/8) - (Math.PI/4);
                const angle2 = vector_angle - (Math.PI/8) - (Math.PI/4);
                graphic.moveTo(scale * endPoint.x, -scale * endPoint.y);
                graphic.lineTo(
                    scale * (endPoint.x - (length * Math.cos(angle1) - length * Math.sin(angle1))),
                    -scale * (endPoint.y - (length * Math.cos(angle1) + length * Math.sin(angle1)))
                );
                graphic.moveTo(scale * endPoint.x, -scale * endPoint.y);
                graphic.lineTo(
                    scale * (endPoint.x - (length * Math.cos(angle2) - length * Math.sin(angle2))),
                    -scale * (endPoint.y - (length * Math.cos(angle2) + length * Math.sin(angle2)))
                );
            }
            graphic.endFill();
            break;
    }

    container.addChild(graphic);
}

export function initializeHeatmapGraphic(overlay: Overlay, graphicState: GraphicState) {
    const bytes_per_pixel = 4;
    const num_pixels = overlay.heatmap_resolution_width * overlay.heatmap_resolution_height;
    let buffer = new Uint8Array(num_pixels * bytes_per_pixel);

    if (!overlay.heatmap_data) {
        throw new Error("No heatmap data.");
    }

    if (overlay.heatmap_data.length != num_pixels) {
        throw new Error("Heatmap data array does not have expected length.");
    }

    const has_alpha_layer = overlay.heatmap_alpha.length != 0;
    const has_full_alpha = overlay.heatmap_alpha.length == num_pixels;
    // Alpha array must be empty, one element, or matching size
    if (has_alpha_layer && overlay.heatmap_alpha.length != 1 && !has_full_alpha) {
        throw new Error("Heatmap alpha array does not have expected length.");
    }

    for (let dst_index = 0; dst_index < buffer.length; dst_index += 4) {
        const src_index = dst_index / 4;
        let alpha = 255;
        if (has_full_alpha) {
            alpha = overlay.heatmap_alpha[src_index];
        } else if (has_alpha_layer) {
            alpha = overlay.heatmap_alpha[0];
        }
        buffer[dst_index + 0] = overlay.heatmap_data[src_index];
        buffer[dst_index + 1] = 0;
        buffer[dst_index + 2] = 0;
        buffer[dst_index + 3] = alpha;
    }

    const heatmapTexture = PIXI.Texture.fromBuffer(buffer, overlay.heatmap_resolution_width, overlay.heatmap_resolution_height, { scaleMode: PIXI.SCALE_MODES.NEAREST })

    const container = (overlay.depth) ? graphicState.overlayContainer : graphicState.underlayContainer;

    let graphic = container.getChildByName(overlay.id) as PIXI.Graphics;
    if (!graphic) {
        graphic = new PIXI.Graphics();
        graphic.name = overlay.id;
        container.addChild(graphic);
    }

    let graphicChild = graphic.getChildByName("heatmap") as PIXI.Sprite;
    if (!graphicChild) {
        graphicChild = new PIXI.Sprite();
        graphicChild.name = "heatmap";
        graphic.addChild(graphicChild);
    }

    graphicChild.texture = heatmapTexture;
}