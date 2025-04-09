<template>
        <v-card variant="outlined" :style="{color: team.color, padding: '5px'}">
           <p> Team: {{team.name}} </p>
           <p style="text-align:center"> Score: {{team.score}} </p>
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
