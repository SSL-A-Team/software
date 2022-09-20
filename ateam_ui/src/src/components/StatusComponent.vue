<template>
    <v-container class="d-flex flex-column">
        <div ref="placeholder">
            Add Goalie, STATUS, manual control here
        </div>
        <v-container class="d-flex flex-column">
            <v-card variant="outlined" class="d-flex my-1 justify-space-around" v-for="robot of state.teams[state.team].robots">
                    {{robot.id}}
                    <!-- this might be really bad for performance -->
                    <v-stage ref="stage" :config="{
                                        width: 100,
                                        height: 55,
                                        listening: false
                                        }">
                        <v-layer>
                            <v-rect :config="{
                                            x: 20,
                                            y: 9,
                                            r_id: robot.id,
                                            team: robot.team,
                                            sceneFunc: robotStatus,
                                            fill: 'DarkBlue',
                                            stroke: 'black',
                                            strokeWidth: 2,
                                            cache: true
                                            }"/>
                        </v-layer>
                    </v-stage>
            </v-card>
        </v-container>
    </v-container>
</template>


<script lang="js">
import { ref, inject } from 'vue';

export default {
    inject: ['state', 'renderConfig'],
    data() {
        return {
            blank: null,
            robotStatus: function(ctx) {
                const scale = 200;
                const radius = .09;
                const sr = scale*radius;

                const start = (-50/180)*Math.PI;
                const end =  (230/180)*Math.PI;

                // Draw robot shell
                ctx.beginPath();
                ctx.arc(sr, sr, sr, start, end);
                ctx.closePath();
                ctx.fillStrokeShape(this);

                // Draw team circle
                ctx.beginPath();
                ctx.arc(sr, sr, sr/4, 0, 2*Math.PI);
                ctx.closePath();
                ctx.fillStyle = this.attrs.team;
                ctx.fill();

                const pos = {
                    x: [.5, 1.5, .55, 1.45],
                    y: [.6, .6, 1.5, 1.5]
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

                let id_pattern = color_patterns.get(this.attrs.r_id)

                for (var i = 0; i < 4; i++) {
                    ctx.beginPath();
                    ctx.arc(pos.x[i]*sr, pos.y[i]*sr, sr/5, 0, 2*Math.PI);
                    ctx.closePath();

                    ctx.fillStyle = id_pattern[i] == "P" ? 'DeepPink' : 'LawnGreen';
                    ctx.fill();
                }
            }
        }
    }
}
</script>
