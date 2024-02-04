<template>
    <v-container class="d-flex flex-column">
        <v-container class="d-flex flex-column">
            <v-card variant="outlined" class="d-flex my-1 justify-space-around" v-for="robot of this.state.world.teams[this.state.world.team].robots.filter((obj)=> obj.isValid())" :ref="'robotCard' + robot.id" style="outline-offset:-1px" @click.stop="this.state.setJoystickRobot(robot.id)">
                    {{robot.id}}
                    <canvas :ref="'canvas' + robot.id" height=100 width=100 style="width:90px; height:90px;"/>
            </v-card>
        </v-container>
    </v-container>
</template>


<script lang="ts">
import { ref, inject } from "vue";
import { Robot, ErrorLevel } from "@/robot";

export default {
    inject: ['state'],
    mounted() {
        this.update();
    },
    methods: {
        update: function() {
            for(const robot of this.state.world.teams[this.state.world.team].robots) {
                if (robot.isValid() && this.$refs["robotCard" + robot.id]) {
                    let canvas = this.$refs["canvas" + robot.id][0];
                    this.drawStatus(robot, canvas.getContext("2d"));

                    const element = this.$refs["robotCard" + robot.id][0].$el;
                    const errorLevel = robot.errorLevel(this.state.sim);

                    if (errorLevel != ErrorLevel.Critical) {
                        element.getAnimations().forEach((animation) => {animation.cancel()});
                    }

                    let style = "";
                    if (robot.id == this.state.controlled_robot) {
                        style += "background: green;";
                    }

                    switch (errorLevel) {
                        case ErrorLevel.None:
                            if (!robot.visible) {
                                style += " outline: solid 5px blue; outline-offset:-1px";
                            }
                            break;
                        case ErrorLevel.Warning:
                            style += " outline: solid 5px yellow; outline-offset:-1px";
                            break;
                        case ErrorLevel.Error:
                            style +=  " outline: solid 5px red; outline-offset:-1px";
                            break;
                        case ErrorLevel.Critical:
                            if (element.getAnimations().length == 0) {
                                element.animate([
                                    {background: "red", outline: "solid 5px red"}],
                                    {easing: "steps(2, jump-none)", duration: 1000, iterations: Infinity}
                                );
                            }
                            break;
                    }
                    element.style = style;
                }
            }
        },
        drawStatus: function(robot: Robot, ctx: CanvasRenderingContext2D) {
            const scale = 280; // This is seperate from the field based renderConfig.scale
            const radius = .09;
            const sr = scale*radius;

            const start = (-50/180)*Math.PI;
            const end =  (230/180)*Math.PI;

            ctx.resetTransform();
            ctx.translate(50, 50); // Center the canvas
            ctx.clearRect(-50, -50, 100, 100); // Clear the drawing

            // Draw robot shell
            ctx.fillStyle = "DarkBlue";
            ctx.beginPath();
            ctx.arc(0, 0, sr, start, end);
            ctx.closePath();
            ctx.fill();

            // Draw team circle
            ctx.fillStyle = robot.team;
            ctx.beginPath();
            ctx.arc(0, 0, .025*scale, 0, 2*Math.PI);
            ctx.closePath();
            ctx.fill();

            const pos = {
                x: [-.055, .055, -.035, .035],
                y: [-.035, -.035, .055, .055]
            }

            // Order:
            // - front left
            // - front right
            // - back left
            // - back right
            let color_patterns = new Map([
                [0, "PPGP"],
                [1, "GPGP"],
                [2, "GGGP"],
                [3, "PGGP"],
                [4, "PPPG"],
                [5, "GPPG"],
                [6, "GGPG"],
                [7, "PGPG"],
                [8, "GGGG"],
                [9, "PPPP"],
                [10, "PPGG"],
                [11, "GGPP"],
                [12, "GPGG"],
                [13, "GPPP"],
                [14, "PGGG"],
                [15, "PGPP"]
            ]);

            let id_pattern = color_patterns.get(robot.id)

            for (var i = 0; i < 4; i++) {
                ctx.fillStyle = id_pattern[i] == "P" ? 'DeepPink' : 'LawnGreen';
                ctx.beginPath();
                // radius increased by .005 for visibility
                ctx.arc(pos.x[i]*scale, pos.y[i]*scale, .025*scale, 0, 2*Math.PI);
                ctx.closePath();
                ctx.fill();
            }

            // Generate Ball Sense indicator
            if (robot.status.breakbeam_ball_detected) {
                ctx.fillStyle = "orange";
                ctx.beginPath();
                ctx.arc(0, -.09*scale, .009*2.5*scale, 0, 2*Math.PI);
                ctx.closePath();
                ctx.fill();
            }


            // Generate wheel status error indicators
            // TODO: Make these look less terrible

            // Theres probably a better way to do this
            const wheels = {
                startX: [-.075, .075, -.035, .035, -.05],
                startY: [-.095, -.095, .12, .12, -.09],
                endX: [-.12, .12, -.1, .1, .05],
                endY: [-.03, -.03, .07, .07, -.09],
                textX: [-.12, .12, -.12, .12, 0],
                textY: [-.12, -.12, .14, .14, -.13]
            }

            // TODO: Figure out what order the motor numbers use
            // TODO: update this to match dribbler naming convention once it is added
            for (var i = 0; i < 5; i++) {
                let general = robot.status["motor_" + i + "_general_error"];
                let hall = robot.status["motor_" + i + "_hall_error"];
                let encoder = robot.status["motor_" + i + "_encoder_error"];

                if (general || hall || encoder) {
                    ctx.beginPath();
                    ctx.strokeStyle = "red";
                    ctx.lineWidth = .02 * scale;
                    ctx.moveTo(scale*wheels.startX[i], scale*wheels.startY[i]);
                    ctx.lineTo(scale*wheels.endX[i], scale*wheels.endY[i]);
                    ctx.stroke();

                    let errString = "";
                    if (general)  errString = errString + "G";
                    if (hall)  errString = errString + "H";
                    if (encoder)  errString = errString + "E";

                    ctx.fillStyle = "red";
                    ctx.font = "13px Arial";
                    ctx.textAlign = "center";
                    ctx.textBaseline = "middle";
                    ctx.fillText(errString, wheels.textX[i]*scale, wheels.textY[i]*scale);
                }
            }

            if (!robot.status.radio_connected) {
                ctx.fillStyle = "yellow";
                ctx.font = "15px Arial";
                ctx.textAlign = "center";
                ctx.textBaseline = "middle";
                ctx.fillText("DC", .14*scale, -.14*scale);
            }
        }
    },
    computed: {
        getRobotStatuses: function() {
            // If performance is a problem we can reduce this to just status members that would cause a redraw
            return this.state.world.teams[this.state.world.team].robots.map(robot => robot.status);
        },
        getControlledRobot: function() {
            return this.state.controlled_robot;
        }
    },
    watch: {
        getRobotStatuses: {
            handler() {
            },
            deep: true
        },
        getControlledRobot: {
            handler() {
            },
            deep: true
        }
    }
}
</script>
