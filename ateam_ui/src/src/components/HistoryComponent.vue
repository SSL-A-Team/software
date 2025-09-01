<template>
    <v-col class="flex-grow-0 flex-shrink-0 justify-center mt-n8 pt-n8 mb-n4 pb-n4">
        <v-slider
            v-model="historySlider"
            :min="-state.worldHistory.length+1"
            max="0"
            step="1"
            v-on:start="startSlider"
            class="px-10 pt-2 pb-n5 mb-n5"
            align="center"
            :color="historyWarningStyle"
        />
        <v-row class="nowrap justify-center mx-3 my-0 px-1 py-0" align="center">
            <v-btn dense class="mx-1" style="max-width: 50;" @click.stop= "rewind()">
                <v-icon icon="mdi-rewind"/>
            </v-btn>
            <v-btn dense class="mx-1" style="max-width: 50;" @click.stop= "stepButton(-1)">
                <v-icon icon="mdi-step-backward"/>
            </v-btn>
            <v-btn dense class="mx-1" style="max-width: 50;" @click.stop= "togglePause()">
                <v-icon :icon="playPauseIcon()"/>
            </v-btn>
            <v-btn dense class="mx-1" style="max-width: 50;" @click.stop= "stepButton(1)">
                <v-icon icon="mdi-step-forward"/>
            </v-btn>
            <v-btn dense class="mx-1" style="max-width: 50;" @click.stop= "fastforward()">
                <v-icon icon="mdi-fast-forward"/>
            </v-btn>
            <v-btn dense class="mx-1" style="max-width: 50;" @click.stop= "goToRealTime()" :color="historyWarningStyle">
                <v-icon icon="mdi-clock-end"/>
            </v-btn>
        </v-row>
    </v-col>
</template>


<script lang="ts">
import { inject } from "vue";
import { AppState } from "@/state";
import * as PIXI from 'pixi.js';

export default {
    inject: ['state'],
    data() {
        return {
            historySlider: 0,
            playbackSpeed: 1.0,
            playbackTimer: null as NodeJS.Timer,
            state: inject('state') as AppState
        }
    },
    mounted() {
    },
    methods: {
        playPauseIcon: function() {
            return (this.state.selectedHistoryFrame == -1 || !this.state.historyReplayIsPaused)? 'mdi-pause' : 'mdi-play';
        },
        togglePause: function() {
            if (this.state.selectedHistoryFrame == -1) {
                this.pause();
                this.state.selectedHistoryFrame = this.state.worldHistory.length - 1;
            } else if (this.state.historyReplayIsPaused) {
                this.playbackSpeed = 1.0;
                this.play();
            } else {
                this.pause();
            }
        },
        play: function() {
            clearInterval(this.playbackTimer);
            this.state.historyReplayIsPaused = false;

            // Playback runs at half framerate
            this.playbackTimer = setInterval(this.playbackUpdate, 20);
        },
        pause: function() {
            this.state.historyReplayIsPaused = true;
            this.playbackSpeed = 1.0;
            clearInterval(this.playbackTimer);
        },
        rewind: function() {
            if (this.state.selectedHistoryFrame == -1) {
                this.playbackSpeed = 1.0;
                this.state.selectedHistoryFrame = this.state.worldHistory.length - 1;
            }
            if (this.playbackSpeed >= 0) {
                this.playbackSpeed = -1.0;
            } else {
                this.playbackSpeed -= 0.5;
            }
            this.play();
        },
        fastforward: function() {
            if (this.playbackSpeed <= 1.0) {
                this.playbackSpeed = 1.5;
            } else {
                this.playbackSpeed += 0.5;
            }
            this.play();
        },
        goToRealTime: function() {
            this.pause();
            this.playbackSpeed = 1.0;
            this.state.selectedHistoryFrame = -1;
        },
        stepButton: function(frames: number) {
            // Pause history replay when stepping by frame
            this.state.historyReplayIsPaused = true;
            this.playbackSpeed = 1.0;

            // If you are in realtime just go to the latest frame
            if (this.state.selectedHistoryFrame == -1) {
                this.state.selectedHistoryFrame = this.state.worldHistory.length - 1;
            }

            this.stepFrames(frames);
        },
        stepFrames: function(frames: number) {
            let intendedFrame = this.state.selectedHistoryFrame + frames;
            if (intendedFrame >= this.state.worldHistory.length) {
                intendedFrame = this.state.worldHistory.length - 1;
            } else if (intendedFrame < 0) {
                intendedFrame = 0;
            }

            this.state.selectedHistoryFrame = intendedFrame;
        },
        startSlider: function(sliderValue: number) {
            // This function moves out of realtime when you first interact with the slider but
            // does not work for dragging
            this.state.historyReplayIsPaused = true;
            this.state.selectedHistoryFrame = this.state.worldHistory.length - 1 + sliderValue;
        },
        playbackUpdate: function() {

            if (this.state.selectedHistoryFrame == -1) {
                this.pause();
            }

            if (!this.state.historyReplayIsPaused) {
                if (this.state.selectedHistoryFrame >= this.state.worldHistory.length - 1 && this.playbackSpeed >= 0) {
                    this.state.historyReplayIsPaused = true;
                } else if (this.state.selectedHistoryFrame <= 0 && this.playbackSpeed <= 0) {
                    this.state.historyReplayIsPaused = true;
                } else {
                    // Rendering every other frame for standard playback rn since it can be kind of laggy
                    this.stepFrames(2 * Math.round(this.playbackSpeed));
                }
            }
        },
        loadHistoryFrame: function() {
            if (this.state.selectedHistoryFrame == -1) {
                this.state.world = this.state.realtimeWorld;
            } else {
                // Calculate the correct index into the circular buffer

                let circularBufferIndex = (this.state.historyEndIndex + 1) + this.state.selectedHistoryFrame;
                if (circularBufferIndex >= this.state.worldHistory.length) {
                    circularBufferIndex = circularBufferIndex - (this.state.worldHistory.length);
                }
                this.state.world = this.state.worldHistory[circularBufferIndex];
                this.state.graphicState.updateField(); // Render the field and overlays
            }

            // Check for orphaned overlay graphics
            const validIds = Array.from(this.state.world.field.overlays.keys());
            for (const container of [this.state.graphicState.underlayContainer, this.state.graphicState.overlayContainer]) {
                const graphics = Array.from(container.children) as PIXI.Graphics[];
                for (const graphic of graphics) {
                    if (!validIds.includes(graphic.name)) {
                        graphic.clear();
                        container.removeChild(graphic);
                    }
                }
            }
        }
    },
    computed: {
        selectedHistoryFrame: function() {
            return this.state.selectedHistoryFrame;
        },
        historyWarningStyle: function() {
            if (this.state.selectedHistoryFrame == -1) {
                return "";
            } else {
                return "red"
            }
        }
    },
    watch: {
        historySlider: {
            handler() {
                // historySlider goes from 0 at current time to -(history buffer length - 1) at the earliest recorded frame

                // This if statement prevents circular loops caused by moving the slider back to the start of the bar when switching to realtime
                // which would then cause the selected frame to switch out of realtime to the last frame in history
                if (this.state.selectedHistoryFrame != -1) {
                    // Does not account for circular buffer
                    this.state.selectedHistoryFrame = this.state.worldHistory.length - 1 + this.historySlider;
                }
            },
            deep: false
        },
        selectedHistoryFrame: {
            handler() {
                if (this.state.selectedHistoryFrame == -1) {
                    this.historySlider = 0;
                } else {
                    this.historySlider = this.state.selectedHistoryFrame - (this.state.worldHistory.length - 1);
                }
                this.loadHistoryFrame();
            },
            deep: false
        }
    },
    components: {
    }
}
</script>
