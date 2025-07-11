<template>
    <canvas ref="canvas" style="width:100%; height:auto; display:block;" />
</template>


<script lang="ts">
import { shallowRef } from 'vue';
import * as PIXI from 'pixi.js';
import { Viewport } from "pixi-viewport";
import { AppState } from "@/state";
import { TeamColor } from "@/team";
import { initializePixi, updateField, drawFieldBoundary, drawFieldLines, drawSideIgnoreOverlay, drawRobots } from "@/field";

export default {
    inject: ['state'],
    data() {
        return {
            pixi: {},
            fieldContainer: null,
            startDragPoint: null
        }
    },
    mounted() {
        // size is based on 140 pixels per meter times the standard max field size including boundaries (13.4 x 10.4m)
        // TODO: Make this properly resize itself
        const width = 1876;
        const height = 1456;
        let pixi = new PIXI.Application({
            width: width,
            height: height,
            background: 'green',
            antialias: true,
            view: this.$refs.canvas
        });
        this.pixi = shallowRef(pixi);
        this.fieldContainer = initializePixi(this.pixi, this.state);

        this.prepareRobotDragging();

        this.pixi.stage.eventMode = "static";
        this.pixi.stage.on("rightdown", this.onRightDown);
        window.addEventListener("keydown", function(event){
            if (event.key == "z") {
                const viewport = pixi.stage.getChildAt(0) as Viewport;
                viewport.setZoom(1, false);
                viewport.moveCenter(0, 0);
            }
        }, false);

        this.state.graphicState.fieldContainer = this.fieldContainer;
        this.state.graphicState.updateField = this.update;
    },
    methods: {
        update: function() {
            updateField(this.state, this.fieldContainer);
        },
        redraw: function() {
            drawFieldLines(this.state, this.fieldContainer.getChildByName("fieldLines"));
            drawFieldBoundary(this.state, this.fieldContainer.getChildByName("fieldBoundary"));
            drawRobots(this.state, this.fieldContainer.getChildByName("robots"));
            drawSideIgnoreOverlay(this.state, this.fieldContainer.getChildByName("fieldUI"));
        },
        onRightDown: function(event: MouseEvent) {
            const stage = this.pixi.stage;
            const viewport = this.pixi.stage.getChildByName("viewport");
            const ballVelLine = this.pixi.stage.getChildByName("ballVelLine") as PIXI.Graphics;

            this.startDragPoint = new PIXI.Point(event.global.x, event.global.y);
            const sdPoint = this.startDragPoint; // so we have access inside the function
            const scale = this.state.renderConfig.scale;
            const state = this.state;
            const defending = this.getDefending;

            this.pixi.stage.on("pointermove", this.rightClickDrag);
            this.pixi.stage.on("rightup", function(event){
                const ballVelText = ballVelLine.getChildByName("ballVelText");
                if (ballVelText) {
                    ballVelLine.removeChild(ballVelText);
                }

                ballVelLine.clear();
                stage.off("pointermove");
                stage.off("rightup");

                const screen_vec = new PIXI.Point(
                    sdPoint.x - event.global.x,
                    sdPoint.y - event.global.y
                );

                const vec = new PIXI.Point(
                    (screen_vec.x * Math.cos(-state.renderConfig.angle * Math.PI / 180) - screen_vec.y * Math.sin(-state.renderConfig.angle * Math.PI / 180)),
                    (screen_vec.x * Math.sin(-state.renderConfig.angle * Math.PI / 180) + screen_vec.y * Math.cos(-state.renderConfig.angle * Math.PI / 180))
                );

                const dist = Math.sqrt(vec.x**2 + vec.y**2);
                if (dist / scale < 0.05) {
                    return;
                }

                let pos = new PIXI.Point();
                viewport.toLocal(sdPoint, null, pos);

                pos = new PIXI.Point(
                    (pos.x * Math.cos(-state.renderConfig.angle * Math.PI / 180) - pos.y * Math.sin(-state.renderConfig.angle * Math.PI / 180)),
                    (pos.x * Math.sin(-state.renderConfig.angle * Math.PI / 180) + pos.y * Math.cos(-state.renderConfig.angle * Math.PI / 180))
                );

                const simulatorCommand = {
                    teleport_ball: [{
                        pose: {
                            position: {
                                x: pos.x * -defending / scale,
                                y: -pos.y / scale
                            },
                        },
                        twist: {
                            linear:{
                                x: 2 * vec.x * -defending / scale,
                                y: -2 * vec.y / scale,
                            }
                        },
                        roll: true
                    }],
                };

                state.sendSimulatorControlPacket(simulatorCommand);
            });

            let pos = new PIXI.Point();
            viewport.toLocal(event.global, null, pos);

            pos = new PIXI.Point(
                (pos.x * Math.cos(-state.renderConfig.angle * Math.PI / 180) - pos.y * Math.sin(-state.renderConfig.angle * Math.PI / 180)),
                (pos.x * Math.sin(-state.renderConfig.angle * Math.PI / 180) + pos.y * Math.cos(-state.renderConfig.angle * Math.PI / 180))
            );

            this.state.world.ball.pose.position.x = pos.x * -this.getDefending / scale;
            this.state.world.ball.pose.position.y = -pos.y / scale;

            const simulatorCommand = {
                teleport_ball: [{
                    pose: this.state.world.ball.pose
                }],
            };

            this.state.sendSimulatorControlPacket(simulatorCommand);
        },
        rightClickDrag: function(event) {
            const viewport = this.pixi.stage.getChildAt(0);

            const ballVelLine = this.pixi.stage.getChildByName("ballVelLine") as PIXI.Graphics;
            ballVelLine.clear();
            const oldBallVelText = ballVelLine.getChildByName("ballVelText");
            if (oldBallVelText) {
                ballVelLine.removeChild(oldBallVelText);
            }

            let pos = event.global;

            let vec = new PIXI.Point();
            vec.x = this.startDragPoint.x - pos.x;
            vec.y = this.startDragPoint.y - pos.y;

            const dist = Math.sqrt(vec.x**2 + vec.y**2);
            const scale = this.state.renderConfig.scale;
            if (dist / scale < 0.05) {
                // Only start the drag routine if you have moved the mouse far enough away from the ball
                return;
            }

            const kickVel = 2 * dist / scale;

            ballVelLine.lineStyle(.08 * scale , "F7F7F750");
            ballVelLine.moveTo(this.startDragPoint.x, this.startDragPoint.y);
            ballVelLine.lineTo(pos.x, pos.y);

            const text = new PIXI.Text(String(kickVel.toFixed(2)) + " m/s", {
                fontSize: 20,
                fill: "F7F7F750"
            });
            text.name = "ballVelText"

            text.anchor.set(0.5, 0.5);

            text.position.x = this.startDragPoint.x + (0.2 * scale * vec.x / dist);
            text.position.y = this.startDragPoint.y + (0.2 * scale * vec.y / dist);

            ballVelLine.addChild(text);

        },
        prepareRobotDragging: function() {
            const state: AppState = this.state;
            const stage = this.pixi.stage;
            const viewport = this.pixi.stage.getChildAt(0);
            var defending = this.getDefending;

            const robotArray = Array.from(state.world.teams.values()).map(i => { return i.robots }).flat()
            const robots = this.fieldContainer.getChildByName("robots").children;
            for (let i = 0; i < robotArray.length; i++) {
                const robot = robots[i] as PIXI.Container;
                robot.on("pointerdown", function(event){
                    if (!robot.visible) {
                        return;
                    }

                    viewport.pause = true;
                    state.draggedRobot = i;

                    stage.on("pointerup", function(event){
                        viewport.pause = false;
                        state.draggedRobot = null;

                        const robotObj = robotArray[i];
                        let pos = new PIXI.Point();
                        viewport.toLocal(event.global, null, pos);

                        pos = new PIXI.Point(
                            (pos.x * Math.cos(-state.renderConfig.angle * Math.PI / 180) - pos.y * Math.sin(-state.renderConfig.angle * Math.PI / 180)),
                            (pos.x * Math.sin(-state.renderConfig.angle * Math.PI / 180) + pos.y * Math.cos(-state.renderConfig.angle * Math.PI / 180))
                        );

                        const scale = state.renderConfig.scale;

                        robotObj.pose.position.x = pos.x * -defending / scale;
                        robotObj.pose.position.y = -pos.y / scale;

                        const simulatorCommand = {
                            teleport_robot: [{
                                id: {
                                    id: [robotObj.id],
                                    team: [{
                                        color: robotObj.team == TeamColor.Yellow ? 1 : 2
                                    }]
                                },
                                pose: robotObj.pose,
                                present: true
                            }],
                        };

                        state.sendSimulatorControlPacket(simulatorCommand);

                        stage.off("pointerup");
                        stage.off("pointermove");
                    });
                    stage.on("pointermove", function(event){
                        let pos = new PIXI.Point();
                        viewport.toLocal(event.global, null, pos);
                        robot.position.x = (pos.x * Math.cos(-state.renderConfig.angle * Math.PI / 180) - pos.y * Math.sin(-state.renderConfig.angle * Math.PI / 180));
                        robot.position.y = (pos.x * Math.sin(-state.renderConfig.angle * Math.PI / 180) + pos.y * Math.cos(-state.renderConfig.angle * Math.PI / 180));
                    });
                });
            }
        }
    },
    computed: {
        getFieldDimensions: function() {
            return this.state.world.field.fieldDimensions;
        },
        getDefending: function() {
            return this.state.world.teams.get(this.state.world.team).defending;
        },
        getIgnoreFieldSide: function() {
            return this.state.world.ignoreSide * -this.getDefending;
        },
        getHoverIgnoreSide: function() {
            return this.state.hoveredFieldIgnoreSide * -this.getDefending;
        },
        getFieldRotation: function() {
            return this.state.renderConfig.angle;
        }
    },
    watch: {
        getFieldDimensions: {
            handler(newValue, oldValue) {
                // This makes me big sad
                if ((JSON.stringify(newValue) != JSON.stringify(oldValue))) {
                    this.redraw();
                }
            },
            deep: true
        },
        getIgnoreFieldSide: {
            handler() {
                let ignoreOverlay = this.fieldContainer.getChildByName("fieldUI").getChildByName("ignoreOverlay");
                let hoverOverlay = this.fieldContainer.getChildByName("fieldUI").getChildByName("hoverOverlay");
                hoverOverlay.visible = false;

                if (this.state.world.ignoreSide == 0) {
                    ignoreOverlay.visible = false;
                } else{
                    ignoreOverlay.visible = true;
                    ignoreOverlay.scale.x = this.getIgnoreFieldSide;
                }
            },
            deep: true
        },
        getHoverIgnoreSide: {
            handler() {
                const viewport = this.pixi.stage.getChildByName("viewport");
                let ignoreOverlay = this.fieldContainer.getChildByName("fieldUI").getChildByName("ignoreOverlay");
                let hoverOverlay = this.fieldContainer.getChildByName("fieldUI").getChildByName("hoverOverlay");

                if (this.state.hoveredFieldIgnoreSide == 0) {
                    hoverOverlay.visible = false;
                    if (this.state.world.ignoreSide != 0) {
                        ignoreOverlay.visible = true;
                    }
                } else {
                    ignoreOverlay.visible = false;
                    if (this.getIgnoreFieldSide != this.getHoverIgnoreSide) {
                        hoverOverlay.visible = true;
                        hoverOverlay.scale.x = this.getHoverIgnoreSide;
                    }
                }
            },
            deep: true
        },
        getFieldRotation: {
            handler() {
                this.fieldContainer.angle = this.getFieldRotation;
                this.redraw();
            },
            deep: false
        }
    }
}
</script>
