import { RenderConfig } from "@/state"
import { Field } from "@/field"
import * as PIXI from "pixi.js"

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

    lifetime_end: number;
    heat_filter: PIXI.Filter

    constructor(id: string, msg: any) {
        this.id = id;
        for (const member of Object.getOwnPropertyNames(msg)) {
            this[member] = msg[member];
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
                float final_alpha = src.a + dst.a * (1.0 - src.a);
                gl_FragColor = vec4(
                    (src.rgb * src.a + dst.rgb * dst.a * (1.0 - src.a)) / final_alpha, final_alpha);
            }
        `

        this.heat_filter = new PIXI.Filter("", frag_src);

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

        switch (this.type) {
            // POINT
            case 0:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(0, this.stroke_color);
                graphic.drawEllipse(0, 0, scale / 30, scale / 30);
                graphic.endFill();
                break;
            // LINE
            case 1:
                if (this.points.length >= 2) {
                    graphic.lineStyle(this.stroke_width, this.stroke_color);
                    graphic.moveTo(scale * this.points[0].x, -scale * this.points[0].y);
                    for (var i = 1; i < this.points.length; i++) {
                        graphic.lineTo(scale * this.points[i].x, -scale * this.points[i].y);
                    }
                }
                break;
            // RECTANGLE
            case 2:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.drawRect(-scale * this.scale.x / 2, -scale * this.scale.y / 2, scale * this.scale.x, scale * this.scale.y);
                graphic.endFill();
                break;
            // ELLIPSE
            case 3:
                graphic.beginFill(this.fill_color);
                graphic.lineStyle(this.stroke_width, this.stroke_color);
                graphic.drawEllipse(0, 0, scale * this.scale.x, scale * this.scale.y);
                graphic.endFill();
                break;
            // POLYGON
            case 4:
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
                let heat_texture = this.array_to_pixi_texture(this.mesh, this.mesh_alpha)
                if (heat_texture.valid) {
                    this.heat_filter.uniforms.uTexture = heat_texture;

                    // Filled black rectangle this is mapped to
                    graphic.beginFill('Black');
                    graphic.lineStyle(this.stroke_width, 'Black');
                    graphic.drawRect(-scale * this.scale.x / 2, -scale * this.scale.y / 2, scale * this.scale.x, scale * this.scale.y);
                    graphic.endFill();
                    graphic.filters = [this.heat_filter];
                }
                break;

            // CUSTOM
            case 7:
                // TODO: This is probably a very low priority to implement
                break;
        }
    }

    array_to_pixi_texture(mesh: Mesh1d[], mesh_alpha: Mesh1d[]) {
        const bpp = 4;

        let height = mesh.length;
        if (height > 0) {
            let width = mesh[0].mesh1d.length;
            if (width > 0) {
                let buff = new Uint8Array(width * height * bpp);
                for (let i = 0; i < height; i++) {
                    // should do check to make sure each ros is length width
                    if (mesh[i].mesh1d.length != width) { break; }
                    for (let j = 0; j < width; j++) {
                        let linear_index = ((width * i) + j) * bpp;
                        buff[linear_index] = mesh[i].mesh1d[j] * 20;
                        buff[linear_index + 1] = 0;
                        buff[linear_index + 2] = 0;
                        buff[linear_index + 3] = 1.0;
                    }
                }

                return PIXI.Texture.fromBuffer(buff, width, height, { scaleMode: PIXI.SCALE_MODES.LINEAR });
            }
        }

        // Regardless of what we return the user can just check if this.valid is true
        return PIXI.Texture.fromBuffer(null, 0, 0);
    }

}
