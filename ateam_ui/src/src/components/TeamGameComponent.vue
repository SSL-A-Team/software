<template>
        <v-card variant="outlined" :style="{color: team.color, padding: '5px'}">
           <p> Team: {{team.name}} </p>
           <p style="text-align:center"> Score: {{team.score}} </p>
           <p style="text-align:center"> Yellow: {{team.yellow_card_times.length}} | {{ yellowCardTimer }} </p>
           <p style="text-align:center"> Red: {{team.red_cards}} </p>
           <p style="text-align:center"> Timeout({{team.timeouts}}): {{timeoutTimer}} </p>
        </v-card>
</template>


<script lang="ts">
import { inject } from "vue";
import { AppState } from "@/state";

export default {
    inject: ['state'],
    props: ['team'],
    data() {
        return {
            state: inject('state') as AppState
        }
    },
    computed: {
        getRefState: function() {
            return this.state.world.referee;
        },
        timeoutTimer: function() {
            const totalSeconds = Math.floor(this.team.timeout_time / (1e6));

            let minutes = Math.floor(totalSeconds / 60);
            let seconds = Math.floor(totalSeconds % 60);

            return minutes + ":" + seconds.toString().padStart(2, '0');

        },
        yellowCardTimer: function() {
            let shortestYellow = null;
            this.team.yellow_card_times.forEach(function(time) {
                if (!shortestYellow || time < shortestYellow) {
                    shortestYellow = time;
                }
            });
            if (!shortestYellow) {
                return "N/A";
            }

            const totalSeconds = Math.floor(shortestYellow / (1e6));

            let minutes = Math.floor(totalSeconds / 60);
            let seconds = Math.floor(totalSeconds % 60);

            return minutes + ":" + seconds.toString().padStart(2, '0');
        }
    },
    watch: {
        getRefState: {
            handler() {
            },
            deep: true
        },
    }
}
</script>
