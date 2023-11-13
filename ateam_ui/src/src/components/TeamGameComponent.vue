<template>
        <v-card variant="outlined" :style="{color: this.team.color, padding: '5px'}">
           <p> Team: {{this.team.name}} </p>
           <p style="text-align:center"> Score: {{this.team.score}} </p>
        </v-card>
</template>


<script lang="ts">
import { ref, inject } from "vue";
import { Referee, GameProperty} from "@/referee";

export default {
    inject: ['state'],
    props: ['team'],
    computed: {
        getRefState: function() {
            return this.state.world.referee;
        },
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
