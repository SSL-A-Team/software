<template>
    <v-container class="d-flex flex-column">
        <p>Goalie ID: {{this.state.getGoalie()}}</p>
            <v-select label="set ID" :items="Array.from({length: 16}, (value, index) => index)" density="compact" variant="solo" @update:modelValue="setGoalie"/>
        <div ref="placeholder">
            Add STATUS, manual control here
        </div>
        <v-container class="d-flex flex-column">
            <v-card variant="outlined" class="d-flex my-1 justify-space-around" v-for="robot of this.state.world.teams[this.state.world.team].robots">
                    {{robot.id}}
                    <canvas ref="canvases" height=100 width=100 style="width:70px; height:70px;"/>
            </v-card>
        </v-container>
    </v-container>
</template>


<script lang="ts">
import { ref, inject } from "vue";
import { Robot } from "@/robot";

export default {
    inject: ['state'],
    mounted() {
        this.update();
    },
    methods: {
        update: function() {
            for(const robot of this.state.world.teams[this.state.world.team].robots) {
                this.drawStatus(robot, this.$refs.canvases[robot.id].getContext("2d"));
            }
        },
        drawStatus: function(robot: Robot, ctx: CanvasRenderingContext2D) {
            const scale = 400; // This is seperate from the field based renderConfig.scale
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
            ctx.beginPath();
            ctx.arc(0, 0, .025*scale, 0, 2*Math.PI);
            ctx.closePath();
            ctx.fillStyle = robot.team;
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
            // TODO: Figure out how much we want this to show i.e. distinguish between general and hall errors? show over temperature warnings?
            // TODO: Make these look less terrible

            // Theres probably a better way to do this
            const wheels = {
                startX: [-.075, .075, -.035, .035],
                startY: [-.095, -.095, .12, .12],
                endX: [-.12, .12, -.1, .1],
                endY: [-.03, -.03, .07, .07]
            }

            // TODO: Figure out what order the motor numbers use
            for (var i = 0; i < 4; i++) {
                let general = robot.status["motor_" + i + "general_error"];
                let hall = robot.status["motor_" + i + "hall_error"];
                if (general || hall) {
                    ctx.beginPath();
                    ctx.strokeStyle = "red";
                    ctx.lineWidth = .02 * scale;
                    ctx.moveTo(scale*wheels.startX[i], scale*wheels.startY[i]);
                    ctx.lineTo(scale*wheels.endX[i], scale*wheels.endY[i]);
                    ctx.stroke();
                }
            }
        }
    },
    computed: {
        getRobotStatuses: function() {
            // If performance is a problem we can reduce this to just status members that would cause a redraw
            return this.state.world.teams[this.state.world.team].robots.map(robot => robot.status);
        }
    },
    watch: {
        getRobotStatuses: {handler() {
            this.update();
        },
        deep: true
        }
    }
}
</script>
