import { RenderConfig } from "@/state"
import * as PIXI from "pixi.js"
import { Buffer } from "buffer";

enum OverlayType {
    Point=0,
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
    heatmap_filter: PIXI.Filter
    
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

        let cmap_src = `
            // jet cmap
            vec4 cmap (float x) {
                const float e0 = 0.0;
                const vec4 v0 = vec4(0,0,0.5137254901960784,1);
                const float e1 = 0.125;
                const vec4 v1 = vec4(0,0.23529411764705882,0.6666666666666666,1);
                const float e2 = 0.375;
                const vec4 v2 = vec4(0.0196078431372549,1,1,1);
                const float e3 = 0.625;
                const vec4 v3 = vec4(1,1,0,1);
                const float e4 = 0.875;
                const vec4 v4 = vec4(0.9803921568627451,0,0,1);
                const float e5 = 1.0;
                const vec4 v5 = vec4(0.5019607843137255,0,0,1);
                float a0 = smoothstep(e0,e1,x);
                float a1 = smoothstep(e1,e2,x);
                float a2 = smoothstep(e2,e3,x);
                float a3 = smoothstep(e3,e4,x);
                float a4 = smoothstep(e4,e5,x);
                return max(mix(v0,v1,a0)*step(e0,x)*step(x,e1),
                    max(mix(v1,v2,a1)*step(e1,x)*step(x,e2),
                    max(mix(v2,v3,a2)*step(e2,x)*step(x,e3),
                    max(mix(v3,v4,a3)*step(e3,x)*step(x,e4),mix(v4,v5,a4)*step(e4,x)*step(x,e5)
                ))));
            }
        `

        let frag_src = cmap_src + `
            varying vec2 vTextureCoord;
            uniform sampler2D uSample;
            uniform sampler2D uTexture;

            void main () {
                vec4 dst = cmap(texture2D(uTexture, vTextureCoord).r);
                dst.a = 0.1; // alpha control on heatmap as a whole

                vec4 src = texture2D(uSample, vTextureCoord);
                float src_a = src.a / 255.0;
                float final_alpha = src_a + dst.a * (1.0 - src_a);
                gl_FragColor = vec4(
                    (src.rgb * src_a + dst.rgb * dst.a * (1.0 - src_a)) / final_alpha, final_alpha);
            }
        `

        this.heatmap_filter = new PIXI.Filter("", frag_src);
        this.heatmap_filter.uniforms.uTexture = this.createHeatmapTexture();
    }

    /**
     * @param overlay foreground container
     * @param underlay background container
     * @param renderConfig field rendering properties
     * @returns true if this overlay should be deleted, false otherwise
     */
    update(overlay: PIXI.Container, underlay: PIXI.Container, renderConfig: RenderConfig): boolean {

        // Handle if the overlay was moved between graphics containers
        if (this.check_other_depth) {
            const opposite_container = (this.depth) ? underlay : overlay;
            this.deleteGraphic(opposite_container);
            this.check_other_depth = false;
        }

        const container = (this.depth) ? overlay : underlay;
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
            case OverlayType.Heatmap:
                graphic.beginFill('Black');
                graphic.lineStyle(this.stroke_width, 'Black');
                graphic.drawRect(-scale * this.scale.x / 2, -scale * this.scale.y / 2, scale * this.scale.x, scale * this.scale.y);
                graphic.endFill();
                graphic.filters = [this.heatmap_filter];
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

    createHeatmapTexture() {
        const bytes_per_pixel = 4;
        const num_pixels = this.heatmap_resolution_width * this.heatmap_resolution_height;
        let buffer = new Uint8Array(num_pixels * bytes_per_pixel);

        if(!this.heatmap_data) {
            throw new Error("No heatmap data.");
        }

        if(this.heatmap_data.length != num_pixels) {
            throw new Error("Heatmap data array does not have expected length.");
        }

        const has_alpha_layer = this.heatmap_alpha.length != 0;
        const has_full_alpha = this.heatmap_alpha.length == num_pixels;
        // Alpha array must be empty, one element, or matching size
        if(has_alpha_layer && this.heatmap_alpha.length != 1 && !has_full_alpha) {
            throw new Error("Heatmap alpha array does not have expected length.");
        }

        for(let dst_index = 0; dst_index < buffer.length; dst_index+=4) {
            const src_index = dst_index / 4;
            let alpha = 255;
            if(has_full_alpha) {
                alpha = this.heatmap_alpha[src_index];
            } else if(has_alpha_layer) {
                alpha = this.heatmap_alpha[0];
            }
            buffer[dst_index + 0] = this.heatmap_data[src_index];
            buffer[dst_index + 1] = 0;
            buffer[dst_index + 2] = 0;
            buffer[dst_index + 3] = alpha;
        }

        return PIXI.Texture.fromBuffer(buffer, this.heatmap_resolution_width, this.heatmap_resolution_height, {scaleMode: PIXI.SCALE_MODES.LINEAR});
    }
}
