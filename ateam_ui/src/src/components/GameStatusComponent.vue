<template>
    <v-col cols="auto">
        <div v-if="doesStageTimeExist" class="d-flex justify-center">
            {{stageTimeLeft}}
        </div>
        <v-row v-if="state.world.referee.blue.name || state.world.referee.yellow.name" class="nowrap justify-center ms-2 ps-2 mb-2 pb-2" align="center">
            <TeamGameComponent v-bind:team="state.world.referee.blue" style="background: blue"/>
                    <v-card variant="outlined" class="ma-2 pa-2 align-centered" :style="'color: ' + stageProperty.color">
                        {{stageProperty.name}}
                    </v-card>
                    <v-card variant="outlined" class="ma-2 pa-2" :style="'color: ' + commandProperty.color">
                        {{commandProperty.name}}
                    </v-card>
            <TeamGameComponent v-bind:team="state.world.referee.yellow" style="background: yellow; color: black"/>
        </v-row>
    </v-col>
</template>


<script lang="ts">
import { inject } from "vue";
import { AppState } from "@/state";
import { getStageProperty, getCommandProperty } from "@/referee";
import TeamGameComponent from "./TeamGameComponent.vue";

export default {
    inject: ['state'],
    data() {
        return {
            state: inject('state') as AppState
        }
    },
    mounted() {
    },
    methods: {
    },
    computed: {
        getRefState: function() {
            return this.state.world.referee;
        },
        stageProperty: function() {
            return getStageProperty(this.state.world.referee);
        },
        commandProperty: function() {
            return getCommandProperty(this.state.world.referee);
        },
        doesStageTimeExist: function() {
            return this.state.world.referee
                && this.state.world.referee.stage_time_left
                && this.state.world.referee.stage_time_left.length > 0;
        },
        stageTimeLeft: function() {
            const totalSeconds = Math.floor(this.state.world.referee.stage_time_left / (1e6));

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
        }
    },
    components: {
        TeamGameComponent
    }
}
</script>
