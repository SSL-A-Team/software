<template>
    <v-stage ref="stage" :config="{
                         width: 100+(state.fieldDimensions.floorWidth*renderConfig.scale),
                         height: 100+(state.fieldDimensions.floorLength*renderConfig.scale),
                         scaleX: .4,
                         scaleY: .4
                         }">

        <!-- layer for field outlines, robots, and the ball -->>
        <v-layer ref="field"
                 @dragstart="handleDragStart" @dragmove="handleDrag"
                 @mousedown="handleDown" @mousemove="handleMouse" @mouseup="handleUp">

            <v-rect ref="background" :config="{
                                    x:0,
                                    y:0,
                                    width: 100+(state.fieldDimensions.floorWidth*renderConfig.scale),
                                    height: 100+(state.fieldDimensions.floorLength*renderConfig.scale),
                                    fill:'green'
                                     }"/>

            <!-- group with its center at the center of the field to make coordinates easy -->>
            <v-group ref="fieldCoords" :config="{
                                         x: (100+(state.fieldDimensions.floorWidth*renderConfig.scale))/2,
                                         y: (100+(state.fieldDimensions.floorLength*renderConfig.scale))/2,
                                         }">
                <v-group ref="fieldMarkings" :config="{
                                             x: 0,
                                             y: 0,
                                             listening: false
                                             }">

                    <v-rect ref="fieldLines" :config="{
                                            x: -state.fieldDimensions.width*renderConfig.scale/2,
                                            y: -state.fieldDimensions.length*renderConfig.scale/2,
                                            width: state.fieldDimensions.width*renderConfig.scale,
                                            height: state.fieldDimensions.length*renderConfig.scale,
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
                                            points: [-state.fieldDimensions.width*renderConfig.scale/2,0,
                                                state.fieldDimensions.width*renderConfig.scale/2, 0],
                                            stroke: 'white',
                                            strokeWidth: 4
                                            }"></v-line>

                    <!-- The goal/box for each team is programtically generated horizontally and then rotated into position -->>
                    <v-group v-for="team in state.teams" :config="{
                                                        x: 0,
                                                        y: 0,
                                                        offsetX: state.fieldDimensions.length*renderConfig.scale/2,
                                                        offsetY: 0,
                                                        rotation: team.defending*90
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

                <div v-for="(team, i) in state.teams">

                    <!-- programatically generate each robot group (a robot shell and its label have to rotate seperately) -->>
                    <v-group v-for="robot in team.robots" :ref="el => {robotShapes[team.color].push(el)}"
                             :config="{ x: robot.x, y:robot.y, draggable: this.state.sim}">

                        <!-- robot shell -->>
                        <v-shape :config="{
                                renderScale: renderConfig.scale,
                                sceneFunc: robotShape,
                                r_id: robot.id,
                                team: robot.team,
                                x: 0,
                                y: 0,
                                offsetX: -.09*renderConfig.scale,
                                offsetY: -.09*renderConfig.scale,
                                rotation: robot.rotation,
                                visible: robot.visible,
                                fill: robot.team,
                                stroke: 'black',
                                strokeWidth: 2
                                }">
                        </v-shape>

                        <!-- text label -->>
                        <v-shape :config="{
                                r_id: robot.id,
                                team: robot.team,
                                sceneFunc: robotText,
                                x: 0,
                                y: 3,
                                stroke: 'black',
                                strokeWidth: 1,
                                fill: 'white'
                                }">
                        </v-shape>
                    </v-group>

                    <v-circle ref="ball" :config="{
                                        x: state.ball.x,
                                        y: state.ball.y,
                                        visible: state.ball.visible,
                                        radius: .022 * renderConfig.scale,
                                        fill: 'orange'
                                        }">

                    </v-circle>
                </div>
            </v-group>


        <!-- layer for only overlays -->>
        </v-layer>
        <v-layer ref="overlay">
            <!-- Need to set up a v-for loop in this with a dynamic way of handling overlays -->>
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
            field: null,
            fieldCoords: null,
            movingBall: false,
            robotShapes: ref({
                blue: [],
                yellow: []
            }),
            robotShape: function(ctx) {
                    const scale = this.attrs.renderScale; //pixels per meter
                    const radius = .09;
                    const sr = scale*radius;

                    const start = (-50/180)*Math.PI;
                    const end =  (230/180)*Math.PI;

                    ctx.beginPath();
                    ctx.arc(-sr, -sr, sr, start, end);
                    ctx.closePath();
                    ctx.fillStrokeShape(this);
            },
            robotText: function(ctx) {
                ctx.fillStyle = this.attrs.team=="yellow" ? "black" : "white";
                ctx.textAlign = "center";
                ctx.textBaseline = "middle";
                ctx.font = "29px sans-serif";
                ctx.fillText(this.attrs.r_id, 0, 0,);
            }
        }
    },
    mounted() {
        this.stage = this.$refs["stage"].getStage();
        this.field = this.$refs["field"].getNode();
        this.fieldCoords = this.$refs["fieldCoords"].getNode();
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
            const robot = this.state.teams[e.target.children[0].attrs.team].robots[e.target.children[0].attrs.r_id];
            const shape = this.robotShapes[robot.team][robot.id].getNode();
            robot.x = shape.attrs.x;
            robot.y = shape.attrs.y;

            //Send command to ros
        },
        handleDown: function(e) {
            if (e.evt.button == 0 && this.state.sim &&
                (e.target === this.$refs.background.getNode() ||
                e.target === this.$refs.ball.getNode())) {
                this.movingBall = true;
                this.$refs.ball.getNode().moveToTop();
                this.state.ball.x = this.fieldCoords.getRelativePointerPosition().x
                this.state.ball.y = this.fieldCoords.getRelativePointerPosition().y

                //Send command to ros
            }
        },
        handleMouse: function(e) {
            if (this.movingBall) {
                this.state.ball.x = this.fieldCoords.getRelativePointerPosition().x
                this.state.ball.y = this.fieldCoords.getRelativePointerPosition().y

                //Send command to ros
            }

        },
        handleUp: function(e) {
            if (e.evt.button == 0) {
                this.movingBall = false;
            }
        }
    }
}
</script>
