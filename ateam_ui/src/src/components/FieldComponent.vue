<template>
    <v-stage ref="stage" :config="{
                         width: 100+(state.fieldDimensions.floorLength*renderConfig.scale),
                         height: 100+(state.fieldDimensions.floorWidth*renderConfig.scale),
                         scaleX: renderConfig.factor,
                         scaleY: renderConfig.factor
                         }">

        <!-- layer for field background and outlines -->>
        <v-layer ref="field" :config="{
                             offsetX: -(100+(state.fieldDimensions.floorLength*renderConfig.scale))/2,
                             offsetY: -(100+(state.fieldDimensions.floorWidth*renderConfig.scale))/2,
                             listening: false
                             }">

            <v-rect ref="background" :config="{
                                     x: -(100+(state.fieldDimensions.floorLength*renderConfig.scale))/2,
                                     y: -(100+(state.fieldDimensions.floorWidth*renderConfig.scale))/2,
                                     width: 100+(state.fieldDimensions.floorLength*renderConfig.scale),
                                     height: 100+(state.fieldDimensions.floorWidth*renderConfig.scale),
                                     fill:'green'
                                     }"/>

            <v-group ref="fieldMarkings" :config="{
                                         x: 0,
                                         y: 0,
                                         listening: false
                                         }">

                <v-rect ref="fieldLines" :config="{
                                         x: -state.fieldDimensions.length*renderConfig.scale/2,
                                         y: -state.fieldDimensions.width*renderConfig.scale/2,
                                         width: state.fieldDimensions.length*renderConfig.scale,
                                         height: state.fieldDimensions.width*renderConfig.scale,
                                         stroke:'white',
                                         strokeWidth: 4
                                         }"></v-rect>

                <v-circle ref="centerCircle" :config="{
                                             x: 0,
                                             y: 0,
                                             radius: state.fieldDimensions.centerRadius*renderConfig.scale,
                                             stroke: 'white',
                                             strokeWidth: 4
                                             }"></v-circle>

                <v-line ref="centerLine" :config="{
                                         x: 0,
                                         y: 0,
                                         points: [0, -state.fieldDimensions.width*renderConfig.scale/2,
                                             0, state.fieldDimensions.width*renderConfig.scale/2],
                                         stroke: 'white',
                                         strokeWidth: 4,
                                         }"></v-line>

                <!-- The goal/box for each team is programtically generated horizontally and then rotated into position -->>
                <v-group v-for="team in state.teams" :key="team.color" :config="{
                                                     x: 0,
                                                     y: 0,
                                                     offsetX: state.fieldDimensions.length*renderConfig.scale/2,
                                                     offsetY: 0,
                                                     rotation: team.defending*90 - 90
                                                     }">

                    <!-- Goal box -->>
                    <v-line :config="{
                            x:0,
                            y:0,
                            points:[
                                0, -state.fieldDimensions.goalWidth*renderConfig.scale,
                                state.fieldDimensions.goalWidth*renderConfig.scale, -state.fieldDimensions.goalWidth*renderConfig.scale,
                                state.fieldDimensions.goalWidth*renderConfig.scale, state.fieldDimensions.goalWidth*renderConfig.scale,
                                0, state.fieldDimensions.goalWidth*renderConfig.scale,
                            ],
                            stroke: 'white',
                            strokeWidth: 4
                            }"></v-line>

                    <!-- Goal -->>
                    <v-line :config="{
                            x:0,
                            y:0,
                            points:[
                                0, -state.fieldDimensions.goalWidth*renderConfig.scale/2,
                                -state.fieldDimensions.goalDepth*renderConfig.scale, -state.fieldDimensions.goalWidth*renderConfig.scale/2,
                                -state.fieldDimensions.goalDepth*renderConfig.scale, state.fieldDimensions.goalWidth*renderConfig.scale/2,
                                0, state.fieldDimensions.goalWidth*renderConfig.scale/2,
                            ],
                            stroke: team.color,
                            strokeWidth: 4
                            }"></v-line>
                </v-group>
            </v-group>
        </v-layer>

        <!-- layer for only underlays -->>
        <v-layer ref="underlay"
            :config="{
            offsetX: -(100+(state.fieldDimensions.floorLength*renderConfig.scale))/2,
            offsetY: -(100+(state.fieldDimensions.floorWidth*renderConfig.scale))/2,
            listening: false
            }">
            <v-shape v-for="underlay in Object.entries(state.underlays).map(i => {return i[1]} )"
                     :key="underlay.ns + '/' + underlay.name"
                     :config="{
                         renderConfig: renderConfig,
                         sceneFunc: overlayShape,
                         overlay: underlay,
                         fieldDimensions: state.fieldDimensions,
                         x: underlay.position.x*renderConfig.scale,
                         y: -underlay.position.y*renderConfig.scale,
                         rotation: underlay.position.z,
                         visible: underlay.visible,
                         fill: underlay.fill_color,
                         stroke: underlay.stroke_color,
                         strokeWidth: underlay.stroke_width,
                         draggable: false,
                         cache: true
                     }">
            </v-shape>
        </v-layer>

        <!-- layer for game elements (robots and the ball) -->>
        <v-layer ref="game"
            @dragstart="handleDragStart" @dragmove="handleDrag" @dragEnd="handleDragEnd"
            @mousedown="handleDown" @mousemove="handleMouse" @mouseup="handleUp"
            :config="{
            offsetX: -(100+(state.fieldDimensions.floorLength*renderConfig.scale))/2,
            offsetY: -(100+(state.fieldDimensions.floorWidth*renderConfig.scale))/2,
            }">

            <v-rect ref="collision" :config="{
                                     x: -(100+(state.fieldDimensions.floorLength*renderConfig.scale))/2,
                                     y: -(100+(state.fieldDimensions.floorWidth*renderConfig.scale))/2,
                                     width: 100+(state.fieldDimensions.floorLength*renderConfig.scale),
                                     height: 100+(state.fieldDimensions.floorWidth*renderConfig.scale),
                                     }"/>

            <!-- programatically generate each robot -->>
            <v-shape v-for="robot in Object.entries(state.teams).map(i => {return i[1].robots}).flat()"
                :key="robot.team + '/' + robot.id"
                :config="{
                renderConfig: renderConfig,
                sceneFunc: robotShape,
                dragBoundFunc: dragBound,
                r_id: robot.id,
                team: robot.team,
                x: robot.pose.position.x*renderConfig.scale,
                y: -robot.pose.position.y*renderConfig.scale,
                offsetX: -.09*renderConfig.scale,
                offsetY: -.09*renderConfig.scale,
                rot: robot.rotation,
                visible: robot.visible,
                fill: robot.team,
                stroke: 'black',
                strokeWidth: 2,
                draggable: this.state.sim,
                cache: true
                }">
            </v-shape>

            <v-circle ref="ball" :config="{
                                 renderConfig: renderConfig,
                                 x: state.ball.pose.position.x*renderConfig.scale,
                                 y: -state.ball.pose.position.y*renderConfig.scale,
                                 visible: state.ball.visible,
                                 radius: .022 * renderConfig.scale,
                                 fill: 'orange',
                                 dragBoundFunc: dragBound
                                 }">
            </v-circle>
        </v-layer>

        <!-- layer for only overlays -->>
        <v-layer ref="overlay"
            :config="{
            offsetX: -(100+(state.fieldDimensions.floorLength*renderConfig.scale))/2,
            offsetY: -(100+(state.fieldDimensions.floorWidth*renderConfig.scale))/2,
            listening: false
            }">
            <v-shape v-for="overlay in Object.entries(state.overlays).map(i => {return i[1]} )"
                     :key="overlay.ns + '/' + overlay.name"
                     :config="{
                         renderConfig: renderConfig,
                         sceneFunc: overlayShape,
                         overlay: overlay,
                         fieldDimensions: state.fieldDimensions,
                         x: overlay.position.x*renderConfig.scale,
                         y: -overlay.position.y*renderConfig.scale,
                         rotation: overlay.position.z,
                         visible: overlay.visible,
                         fill: overlay.fill_color,
                         stroke: overlay.stroke_color,
                         strokeWidth: overlay.stroke_width,
                         draggable: false,
                         cache: true
                     }">
            </v-shape>
        </v-layer>
    </v-stage>
</template>


<script lang="js">
import { ref, inject } from 'vue';

export default {
    inject: ['state', 'renderConfig'],
    data() {
        return {
            stage: null,
            overlay: null,
            game: null,
            field: null,
            movingBall: false,
            dragBound: function(pos) {
                var clamp = function (value, min, max) {
                    return Math.min(Math.max(value, min), max);
                }

                var xlim = this.parent.canvas.width * this.attrs.renderConfig.factor;
                var ylim = this.parent.canvas.height * this.attrs.renderConfig.factor;

                return {
                    x: clamp(pos.x, 0, xlim),
                    y: clamp(pos.y, 0, ylim)
                }
            },
            robotShape: function(ctx) {
                const scale = this.attrs.renderConfig.scale; //pixels per meter
                const radius = .09;
                const sr = scale*radius;

                const start = ((this.attrs.rot-50)/180)*Math.PI;
                const end =  ((this.attrs.rot+230)/180)*Math.PI;

                ctx.beginPath();
                ctx.arc(-sr, -sr, sr, start, end);
                ctx.closePath();
                ctx.fillStrokeShape(this);

                ctx.fillStyle = this.attrs.team=="yellow" ? "black" : "white";
                ctx.textAlign = "center";
                ctx.textBaseline = "middle";
                ctx.font = "29px sans-serif";
                ctx.fillText(this.attrs.r_id, -sr, -sr + 3);
            },
            overlayShape: function(ctx) {
                const overlay = this.attrs.overlay;
                const scale = this.attrs.renderConfig.scale; //pixels per meter

                switch(overlay.type) {
                    // POINT
                    case 0:
                        ctx.beginPath();
                        ctx.lineWidth = 0;
                        ctx.ellipse(0, 0, scale/100, scale/100, 0, 0, 2*Math.PI);
                        ctx.closePath();
                        ctx.fillStrokeShape(this);
                        break;
                    // LINE
                    case 1:
                        if (overlay.points.length >= 2) {
                            ctx.beginPath();
                            ctx.lineWidth = overlay.stroke_width;
                            ctx.strokeStyle = overlay.stroke_color;
                            ctx.moveTo(scale*overlay.points[0].x, -scale*overlay.points[0].y);
                            for (var i = 1; i < overlay.points.length; i++) {
                                ctx.lineTo(scale*overlay.points[i].x, -scale*overlay.points[i].y);
                            }
                            ctx.stroke();
                        }
                        break;
                    // RECTANGLE
                    case 2:
                        //TODO: Should this be switched to specifying the top left corner instead?
                        ctx.beginPath();
                        ctx.rect(-overlay.scale.x/2, -overlay.scale.y/2, overlay.scale.x, overlay.scale.y);
                        ctx.closePath();
                        ctx.fillStrokeShape(this);
                        break;
                    // ELLIPSE
                    case 3:
                        ctx.beginPath();
                        ctx.ellipse(0, 0, overlay.scale.x, overlay.scale.y, 0, 0, 2*Math.PI);
                        ctx.closePath();
                        ctx.fillStrokeShape(this);
                        break;
                    // POLYGON
                    case 4:
                        if (overlay.points.length >= 2){
                            ctx.moveTo(overlay.points.at(-1).x, overlay.points.at(-1).y);
                            ctx.beginPath();
                            overlay.points.forEach(function(point) {
                                ctx.lineTo(point.x, point.y);
                            });
                            ctx.closePath();
                            ctx.fillStrokeShape(this);
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
                        //TODO: Should probably break things out into a set of util files
                        // Interpolation util object
                        var Interpolate = class{
                            // top/bottom left/right values
                            constructor(mesh, r, c, xstart, ystart, xstep, ystep) {
                                this.tl = mesh[r].mesh1d[c];
                                this.tr = mesh[r].mesh1d[c+1];
                                this.bl = mesh[r+1].mesh1d[c];
                                this.br = mesh[r+1].mesh1d[c+1];

                                this.xstart = xstart;
                                this.ystart = ystart;

                                this.xstep = xstep;
                                this.ystep = ystep;
                            }

                            getValue = function(x, y) {
                                // top/bottom x interpolation
                                var tx = (this.tl*(this.xstart + this.xstep - x) + this.tr*(x - this.xstart))/this.xstep;
                                var bx = (this.bl*(this.xstart + this.xstep - x) + this.br*(x - this.xstart))/this.xstep;

                                // interpolate along the y axis
                                return (tx*(this.ystart + this.ystep - y) + bx*(y - this.ystart))/this.ystep;

                            };
                        };

                        // The mesh generation code works directly with canvas pixels
                        // so it ignores konva scaling so we need to manually handle pixel scaling
                        const pixelScale = this.attrs.renderConfig.scale *
                             this.attrs.renderConfig.factor;

                        const width = overlay.scale.x * pixelScale;
                        const height = overlay.scale.y * pixelScale;

                        const xstep = width / (overlay.mesh[0].length-1);
                        const ystep = height / (overlay.mesh.length-1);

                        var pixels = ctx.createImageData(width, height);
                        const heatcolors = [[0,0,255],[0,255,0],[255,255,0],[255,0,0]];
                        for (var r = 0; r < overlay.mesh.length - 1; r++) {
                            for (var c = 0; c < overlay.mesh[0].length - 1; c++) {
                                var xstart = xstep * c;
                                var ystart = ystep * r;

                                var interp = new Interpolate(overlay.mesh, r, c, xstart, ystart, xstep, ystep);
                                var interpAlpha = new Interpolate(overlay.mesh_alpha, r, c, xstart, ystart, xstep, ystep);

                                for (var y = ystart; y < ystart + ystep; y++) {
                                    for (var x = xstart; x < xstart + xstep; x++) {

                                        var value = interp.getValue(x, y);

                                        // calculate heatmap color
                                        var v = value*(heatcolors.length - 1);
                                        var id1 = Math.floor(v);
                                        var id2 = id1 + 1;
                                        var frac = v - id1;

                                        if (value <= 0) id1 = id2 = 0;
                                        else if (value >= 1) id1 = id2 = heatcolors.length - 1;

                                        // pixels.data is a 1d array of r,g,b,a values
                                        var idx = 4*((y*width) + x);
                                        for (var i = 0; i < 3; i++) {
                                            pixels.data[idx+i] = (heatcolors[id2][i] - heatcolors[id1][i]) * frac + heatcolors[id1][i];
                                        }

                                        if (overlay.mesh_alpha) {
                                            pixels.data[idx+3] = 255*interpAlpha.getValue(x, y);
                                        } else {
                                            pixels.data[idx+3] = 255;
                                        }
                                    }
                                }
                            }
                        }

                        // Offset the location to the center of the field, then center the image, then set its position
                        var imageX = ((ctx.canvas.width/2) + ((overlay.position.x - overlay.scale.x/2)*scale))*this.attrs.renderConfig.factor;
                        var imageY = ((ctx.canvas.height/2) + ((overlay.position.y - overlay.scale.y/2)*scale))*this.attrs.renderConfig.factor;
                        ctx.putImageData(pixels, imageX, imageY);

                        break;
                    // CUSTOM
                    case 7:
                        //TODO: Figure out how to get ctx into the function
                        Function(overlay.text)();
                        break;
                }
            }
        }
    },
    mounted() {
        this.stage = this.$refs["stage"].getStage();
        this.field = this.$refs["field"].getNode();
        this.game = this.$refs["game"].getNode();
        this.overlay = this.$refs["overlay"].getNode();
    },
    onBeforeUpdate() {
        this.teams.value = [];
    },
    methods: {
        handleDragStart: function(e) {
            e.target.moveToTop();
        },
        handleDrag: function(e) {
            // Send ros messages only, updating state causes layering issues
            // if (e.target === this.$refs.ball.getNode()) {
            //     this.state.ball.x = e.target.x();
            //     this.state.ball.y = e.target.y();
            // } else {
            //     const shape = e.target;
            //     const robot = this.state.teams[shape.attrs.team].robots[shape.attrs.r_id];
            //     robot.x = e.target.x();
            //     robot.y = e.target.y();
            // }
        },
        handleDragEnd: function(e) {
            if (e.target === this.$refs.ball.getNode()) {
                this.movingBall = false;
            }
        },
        handleDown: function(e) {
            if (e.evt.button == 0 && this.state.sim &&
               (e.target === this.$refs.collision.getNode() ||
                e.target === this.$refs.ball.getNode())) {

                const ball = this.$refs.ball.getNode()
                const pos = this.game.getRelativePointerPosition();
                this.state.ball.pose.position = pos;
                ball.x(pos.x);
                ball.y(pos.y);

                this.movingBall = true;
            }
        },
        handleMouse: function(e) {
            if (this.movingBall && !e.target.isDragging()) {

                /* Have to do this to prevent a potential issue if the
                   mouse is moving as you click */
                const ball = this.$refs.ball.getNode()
                const pos = this.game.getRelativePointerPosition();
                this.state.ball.pose.position = pos;
                ball.x(pos.x);
                ball.y(pos.y);
                this.$refs.ball.getNode().startDrag();
            }
        },
        handleUp: function(e) {
            if (e.evt.button == 0 && this.movingBall) {
                this.movingBall = false;
            }
        }
    }
}
</script>
