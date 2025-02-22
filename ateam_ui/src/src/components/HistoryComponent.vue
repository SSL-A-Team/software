<template>
    <v-slider v-model="historySlider" :min="-this.state.worldHistory.length+1" max="0" step="1"/>
    <v-row class="nowrap justify-center ma-2 pa-2" align="center">
        <v-btn @click.stop= "setPause(true)"> pause </v-btn>
        <v-btn @click.stop= "setPause(false)"> unpause </v-btn>
        <v-btn @click.stop= "goToRealTime()"> go to real time </v-btn>
    </v-row>
</template>


<script lang="ts">
import { ref, inject } from "vue";
import { Referee, GameProperty} from "@/referee";
import { Overlay, OverlayType } from '@/overlay'
import { TeamColor } from '@/team'

export default {
    inject: ['state'],
    data() {
        return {
            historySlider: 0
        }
    },
    mounted() {
    },
    methods: {
        goToRealTime: function() {
            this.state.selectedHistoryFrame = -1;
            this.state.world = this.state.currentWorld;
        },
        setPause: function(shouldPause: boolean) {
            this.state.historyReplayIsPaused = shouldPause;
        },
        loadHistoryFrame: function() {

            if (this.state.selectedHistoryFrame == -1) {
                return;
            }

            // Calculate the correct index into the circular buffer
            let circularBufferIndex = this.state.selectedHistoryFrame - this.state.historyEndIndex;
            if (circularBufferIndex < 0) {
                circularBufferIndex = this.state.worldHistory.length + circularBufferIndex;
            }
            let worldObject = this.state.worldHistory[circularBufferIndex];

            // field
            worldObject.field.drawSideIgnoreOverlay = this.state.world.field.drawSideIgnoreOverlay;
            worldObject.field.drawFieldLines = this.state.world.field.drawFieldLines;
            worldObject.field.initializePixi = this.state.world.field.initializePixi;
            worldObject.field.update = this.state.world.field.update;

            // overlays
            // Overlays are messed up and don't properly handle the lifetime sometimes
            for (const id in worldObject.field.overlays) {

                let overlay_data = worldObject.field.overlays[id];
                worldObject.field.overlays[id] = new Overlay(overlay_data.id, overlay_data, true);
            }

            // ball
            worldObject.ball.update = this.state.world.ball.update;
            worldObject.ball.draw = this.state.world.ball.draw;

            // robots
            for (const color of [TeamColor.Blue, TeamColor.Yellow]) {
                for (let i = 0; i < worldObject.teams[color].robots.length; i++) {
                    worldObject.teams[color].robots[i].isValid = this.state.world.teams[color].robots[i].isValid;
                    worldObject.teams[color].robots[i].rotation = this.state.world.teams[color].robots[i].rotation;
                    worldObject.teams[color].robots[i].setRotation = this.state.world.teams[color].robots[i].setRotation;
                    worldObject.teams[color].robots[i].errorLevel = this.state.world.teams[color].robots[i].errorLevel;
                    worldObject.teams[color].robots[i].update = this.state.world.teams[color].robots[i].update;
                    worldObject.teams[color].robots[i].draw = this.state.world.teams[color].robots[i].draw;
                }
            }

            // referee
            worldObject.referee.getStageProperty = this.state.world.referee.getStageProperty;
            worldObject.referee.getCommandProperty = this.state.world.referee.getCommandProperty;

            this.state.world = worldObject;
        }
    },
    computed: {
        selectedHistoryFrame: function() {
            return this.state.selectedHistoryFrame;
        }
    },
    watch: {
        historySlider: {
            handler() {
                // historySlider goes from 0 at current time to -(history_buffer length - 1) at the earliest recorded frame

                // Does not account for circular buffer
                this.state.selectedHistoryFrame = this.state.worldHistory.length - 1 + this.historySlider;
            },
            deep: false
        },
        selectedHistoryFrame: {
            handler() {
                this.historySlider = this.state.selectedHistoryFrame - (this.state.worldHistory.length - 1);
                this.loadHistoryFrame();
            },
            deep: false
        }
    },
    components: {
    }
}
</script>
