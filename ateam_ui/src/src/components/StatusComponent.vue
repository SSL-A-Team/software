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

                //TODO: I think the order of the binary dots is wrong

                // Draw id circles
                const id = "000"+this.attrs.r_id.toString(2);
                for (var i = 0; i < 4; i++) {
                    ctx.beginPath();
                    ctx.arc(pos.x[i]*sr, pos.y[i]*sr, sr/5, 0, 2*Math.PI);
                    ctx.closePath();

                    ctx.fillStyle = id[id.length - 1 - i] == "0" ? 'DeepPink' : 'LawnGreen';
                    ctx.fill();
                }
            }
        }
    }
}
</script>
