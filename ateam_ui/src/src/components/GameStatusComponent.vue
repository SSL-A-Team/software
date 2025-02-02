<template>
    <v-row class="nowrap justify-space-between" align="center">
        <v-row align="center">
        <p>Goalie ID: {{this.state.getGoalie()}}</p>
            <v-select v-model="selectedItem" label="set ID" class="flex-grow-0 align-end" :items="Array.from({length: 16}, (value, index) => index)" density="compact" variant="solo" @update:modelValue="setGoalieId"/>
        </v-row>
        <v-spacer/>
    </v-row> 
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
    <v-alert title="Set Goalie Failed" :text="this.state.goalie_service_status[1]" type="error" icon=false density="compact" :model-value="alert"></v-alert>
</template>


<script lang="ts">
import { ref, inject } from "vue";
import { Referee, GameProperty} from "@/referee";
import TeamGameComponent from "./TeamGameComponent.vue";

export default {
    inject: ['state'],
    data() {
        return {
            alert: false,
            selectedItem: null
        }
    },
    mounted() {
    },
    methods: {
        setGoalieId(new_id) {
            this.state.setGoalie(new_id);
        }
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
        },
        goalie_service_status: function() {
            return this.state.goalie_service_status
        }
    },
    watch: {
        getRefState: {
            handler() {
            },
            deep: true
        },
        goalie_service_status: {
            handler() {
                if (this.state.goalie_service_status[0]) {
                    this.alert = false;
                } else {
                    this.alert = true;

                    const state = this;
                    setTimeout(function() {
                        state.alert = false;

                        const goalie = parseInt(state.state.getGoalie());
                        if (isNaN(goalie)) {
                            state.selectedItem = null;
                        } else {
                            state.selectedItem = goalie;
                        }
                    }, 4000);
                }
            },
            deep: true
        }
    },
    components: {
        TeamGameComponent
    }
}
</script>
