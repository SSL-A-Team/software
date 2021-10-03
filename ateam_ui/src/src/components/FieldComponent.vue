<template>
    <v-stage ref="stage" :config="{
                         width: state.fieldDimensions.width*renderConfig.scale,
                         height: state.fieldDimensions.length*renderConfig.scale,
                         }">

        <v-layer ref="field"
                 @dragstart="handleDragStart" @dragmove="handleDrag"
                 @mousedown="handleDown" @mousemove="handleMouse" @mouseup="handleUp"
                 :config="{scaleX: .4, scaleY: .4}">

            <v-rect ref="background" :config="{
                                    x:0,
                                    y:0,
                                    width: state.fieldDimensions.width*renderConfig.scale,
                                    height: state.fieldDimensions.length*renderConfig.scale,
                                    fill:'green'
                                     }"/>

            <div ref="fieldMarkings">
                <!-- <v-rect ref="fieldLines" config:{

                     }>
                     </v-rect> -->
            </div>

            <div v-for="(team, i) in state.teams">
                <v-shape v-for="robot in team.robots" :ref="el => {robotShapes[team.color].push(el)}"
                        :config="{
                        renderScale: renderConfig.scale,
                        sceneFunc: robotShape,
                        r_id: robot.id,
                        ref: robot.team+'_robot'+robot.id,
                        team: robot.team,
                        x: robot.x,
                        y: robot.y,
                        visible: robot.visible,
                        fill: robot.team,
                        draggable: this.state.sim,
                        stroke: 'black',
                        strokeWidth: 2
                        }">
                </v-shape>
                <v-circle ref="ball" :config="{
                                     x: state.ball.x,
                                     y: state.ball.y,
                                     visible: state.ball.visible,
                                     radius: .022 * renderConfig.scale,
                                     fill: 'orange'
                                     }">

                </v-circle>
            </div>
        </v-layer>
        <v-layer ref="overlay">
            <!-- Need to set up a v-for loop in this with a dynamic way of handling overlays -->>
        </v-layer>
    </v-stage>
</template>


<script lang="js">
import { ref, defineComponent, inject, provide, reactive, watchEffect } from 'vue';

export default {
    inject: ['state', 'renderConfig'],
    // setup() {
    //     const teams = ref([]);
    //     return {teams};
    // },
    data() {
        return {
            stage: null,
            overlay: null,
            field: null,
            movingBall: false,
            robotShapes: ref({
                blue: [],
                yellow: []
            }),
            configBackground: {
                x: 0,
                y: 0,
                fill: "green"
            },
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

                    ctx.strokeStyle = "black"; //this will need to support styles later
                    ctx.fillStyle = "white";
                    ctx.textAlign = "center";
                    ctx.textBaseline = "middle"
                    ctx.font = "30px sans-serif";
                    ctx.strokeText(this.getAttr('r_id'), -sr, -sr + 4); // this doesn't work on linux neutralino, not sure why
                    ctx.fillText(this.getAttr('r_id'), -sr, -sr + 4);
            }
        }
    },
    mounted() {
        this.stage = this.$refs["stage"].getStage();
        this.field = this.$refs["field"].getNode();
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
            const robot = this.state.teams[e.target.attrs.team].robots[e.target.attrs.r_id];
            const shape = this.robotShapes[robot.team][robot.id].getNode();
            robot.x = shape.attrs.x;
            robot.y = shape.attrs.y;

            //Send command to ros
        },
        handleDown: function(e) {
            if (this.state.sim &&
                (e.target === this.$refs.background.getNode() ||
                e.target === this.$refs.ball.getNode())) {
                this.movingBall = true;
                this.$refs.ball.getNode().moveToTop();
                this.state.ball.x = e.evt.layerX;
                this.state.ball.y = e.evt.layerY;

                //Send command to ros
            }
        },
        handleMouse: function(e) {
            if (this.movingBall) {
                this.state.ball.x = e.evt.layerX;
                this.state.ball.y = e.evt.layerY;

                //Send command to ros
            }

        },
        handleUp: function(e) {
            this.movingBall = false;
        }
    }
}
</script>
