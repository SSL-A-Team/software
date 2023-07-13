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

        // LOADING A SHADER
        // https://github.com/pixijs/pixijs/issues/3654
        // Yes, PIXI includes glslify. You could use the loader.

        // const loader = new PIXI.Loader(); // you can also create your own if you want
        // loader.add('frag', 'assets/shaders/heatmap.frag');
        // loader.load((loader, resources) => {
        //   // this.heatmap_shader = PIXI.Shader("", resources['frag'].data);
        //   this.frag_src = resources['frag'].data
        // });

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
                this.draw(graphic, renderConfig,  overlay);
            }
        } else {
            graphic = new PIXI.Graphics();
            graphic.name = this.id;
            this.draw(graphic, renderConfig, overlay);
            container.addChild(graphic);
        }
    }

    draw(graphic: PIXI.Graphics, renderConfig: RenderConfig, container: PIXI.Container) {
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
                graphic.drawEllipse(0, 0, scale*this.scale.x, scale*this.scale.y);
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
                // This is for 2d arrays of meshes
                // ASSUMPTION MESH IS IN Y then X or row then column (row-major)
                // Could just use flat if this was directly an array of arrays

                let height = 400;
                let width = 400;

                const bpp = 4;
                let buff = new Uint8Array(width * height * bpp);
                for (let i = 0; i < height; i++) {
                  for (let j = 0; j < width; j++) {
                    let linear_index = ((width * i) + j) * bpp;
                    buff[linear_index] =     100;
                    buff[linear_index + 1] = 0;
                    buff[linear_index + 2] = 200;
                    buff[linear_index + 3] = 100;
                    // buff[linear_index] =     0.5;
                    // buff[linear_index + 1] = 0.0;
                    // buff[linear_index + 2] = 0.5;
                    // buff[linear_index + 3] = 0.5;
                  }
                }


                // console.log("HELLO");
                // let height = this.mesh.length;
                // if (height > 0){
                //   let width = this.mesh[0].mesh1d.length;
                //   if (width > 0) {
                    // const bpp = 4;
                    // // let buff = new Uint8Array(width * height * bpp);
                    // let buff = [];
                    // for (let i = 0; i < height; i++) {
                    //   // should do check to make sure each ros is length width
                    //   if (this.mesh[i].mesh1d.length != width){}
                    //   for (let j = 0; j < width; j++) {
                    //     let linear_index = ((width * i) + j) * bpp;
                    //     buff[linear_index] =     1.0;
                    //     // buff[linear_index + 1] = 1.0;
                    //     // buff[linear_index + 2] = 1.0;
                    //     // buff[linear_index + 3] = 1.0;


                    //     // buff[linear_index] = this.mesh[i].mesh1d[j] * 20;
                    //     buff[linear_index + 1] = 0;
                    //     buff[linear_index + 2] = 0;
                    //     buff[linear_index + 3] = 1.0;
                    //   }
                    // }

                    let texture = PIXI.Texture.fromBuffer(buff, width, height);
                    let frag_src = `
                      varying vec2 vTextureCoord;
                      uniform sampler2D temp;

                      void main(void) {
                        gl_FragColor = texture2D(temp, vTextureCoord);
                      }
                    `

                    // Filled black rectangle this is mapped to
                    graphic.beginFill('Black');
                    graphic.lineStyle(this.stroke_width, 'Black');
                    graphic.drawRect(-scale*this.scale.x/2, -scale*this.scale.y/2, scale*this.scale.x, scale*this.scale.y);
                    graphic.endFill();

                    graphic.filters = [new PIXI.Filter("", frag_src, {temp: texture})];

                //   }
                // }
                // array_to_pixi_texture(this.mesh, this.mesh_alpha)


                break;
            // CUSTOM
            case 7:
                // TODO: This is probably a very low priority to implement
                break;
        }
    }

    // array_to_pixi_texture(mesh: Mesh1d[], mesh_alpha: Mesh1d[]) {
      // flat: number[]
      // for (let i = 0; i < .length; i++) {
      // for (let j = 0; i < .length; i++) {
      //   // throw error if at any point this doesnt equal i's length
      // }
      // PIXI.Texture.fromBuffer()

      // return
    // }

}
