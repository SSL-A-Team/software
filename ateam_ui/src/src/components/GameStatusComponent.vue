<template>
    <v-row v-if="this.state.world.referee.blue.name || this.state.world.referee.yellow.name" class="nowrap justify-center ma-2 pa-2" align="center">
        <TeamGameComponent v-bind:team="this.state.world.referee.blue" style="background: blue"/>
        <v-card variant="outlined" class="ma-2 pa-2 align-centered" :style="{color: stageProperty.color}">
            {{this.stageProperty.name}}
        </v-card>
        <v-card variant="outlined" class="ma-2 pa-2" :style="{color: commandProperty.color}">
            {{this.commandProperty.name}}
        </v-card>
        <TeamGameComponent v-bind:team="this.state.world.referee.yellow" style="background: yellow; color: black"/>
    </v-row>
</template>


<script lang="ts">
import { ref, inject } from "vue";
import { Referee, GameProperty} from "@/referee";
import TeamGameComponent from "./TeamGameComponent.vue";

export default {
    inject: ['state'],
    data() {
        return {}
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
            return this.state.world.referee.getStageProperty();
        },
        commandProperty: function() {
            return this.state.world.referee.getCommandProperty();
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
