<template>
    <v-row v-if="state.world.referee.blue.name || state.world.referee.yellow.name" class="nowrap justify-center ma-2 pa-2" align="center">
        <TeamGameComponent v-bind:team="state.world.referee.blue" style="background: blue"/>
        <v-card variant="outlined" class="ma-2 pa-2 align-centered" :style="'color: ' + stageProperty.color">
            {{stageProperty.name}}
        </v-card>
        <v-card variant="outlined" class="ma-2 pa-2" :style="'color: ' + commandProperty.color">
            {{commandProperty.name}}
        </v-card>
        <TeamGameComponent v-bind:team="state.world.referee.yellow" style="background: yellow; color: black"/>
    </v-row>
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
